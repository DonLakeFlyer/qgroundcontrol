/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "TileImageSource.h"

#include <QtCore/QFile>
#include <QtNetwork/QNetworkAccessManager>
#include <QtNetwork/QNetworkReply>

#include <memory>

#include "MapProvider.h"
#include "QGCCacheTile.h"
#include "QGCLoggingCategory.h"
#include "QGCMapEngine.h"
#include "QGCMapTasks.h"
#include "QGCMapUrlEngine.h"
#include "QGeoFileTileCacheQGC.h"
#include "QGeoTileFetcherQGC.h"

QGC_LOGGING_CATEGORY(GeoMapTileImageSourceLog, "GeoMap.TileImageSource")
QGC_LOGGING_CATEGORY(GeoMapTileImageSourceVerboseLog, "GeoMap.TileImageSource.Verbose")

namespace {

// Bing serves a placeholder image instead of an HTTP error where it has no
// imagery; delivering or caching it as a tile would drape it onto patches
bool isBingEmptyTile(int mapId, const QByteArray& data)
{
    static const QByteArray bingNoTileImage = [] {
        QFile file(QStringLiteral(":/res/BingNoTileBytes.dat"));
        return file.open(QFile::ReadOnly) ? file.readAll() : QByteArray();
    }();

    if (bingNoTileImage.isEmpty() || (data != bingNoTileImage)) {
        return false;
    }
    const SharedMapProvider provider = UrlFactory::getMapProviderFromQtMapId(mapId);
    return provider && provider->isBingProvider();
}

}  // namespace

TileImageSource::TileImageSource(const QString& mapType, QObject* parent)
    : QObject(parent),
      _mapType(mapType),
      _mapId(UrlFactory::getQtMapIdFromProviderType(mapType)),
      _networkManager(new QNetworkAccessManager(this))
{
    if (_mapId < 0) {
        qCWarning(GeoMapTileImageSourceLog) << "unknown map type" << _mapType << "- all tile requests will fail";
    }
}

int TileImageSource::requestTileImage(const TileMath::TileKey& key)
{
    const int requestId = ++_requestIdCounter;
    _pending.insert(requestId);

    if (_mapId < 0) {
        _failAsync(requestId);  // unknown provider: don't spam the cache/URL factory per request
        return requestId;
    }
    qCDebug(GeoMapTileImageSourceVerboseLog) << "requestTileImage: key" << key << "requestId" << requestId;

    QGCFetchTileTask* const task = QGeoFileTileCacheQGC::createFetchTileTask(_mapType, key.x, key.y, key.zoom);
    connect(task, &QGCFetchTileTask::tileFetched, this, [this, requestId, key](QGCCacheTile* tile) {
        const std::unique_ptr<QGCCacheTile> guard(tile);  // caller-owned per task contract
        if (!_pending.contains(requestId)) {
            return;                                       // cancelled while the lookup was in flight
        }
        if (!tile || tile->img.isEmpty()) {
            qCDebug(GeoMapTileImageSourceLog) << "cache returned empty tile for" << key;
            _finishFailed(requestId);
            return;
        }
        _deliver(requestId, key, tile->img);
    });
    connect(task, &QGCMapTask::error, this, [this, requestId, key](QGCMapTask::TaskType, const QString&) {
        if (!_pending.contains(requestId)) {
            return;
        }
        qCDebug(GeoMapTileImageSourceVerboseLog) << "tile" << key << "not cached, fetching from network";
        _fetchFromNetwork(requestId, key);
    });

    if (!getQGCMapEngine()->addTask(task)) {
        qCDebug(GeoMapTileImageSourceLog) << "could not queue cache lookup for" << key;
        task->deleteLater();  // never enqueued: the worker will not clean it up
        _failAsync(requestId);
    }
    return requestId;
}

void TileImageSource::_failAsync(int requestId)
{
    // Deliver asynchronously so callers see one consistent flow
    QMetaObject::invokeMethod(
        this,
        [this, requestId] {
            if (_pending.contains(requestId)) {
                _finishFailed(requestId);
            }
        },
        Qt::QueuedConnection);
}

void TileImageSource::cancelRequest(int requestId)
{
    if (!_pending.remove(requestId)) {
        return;
    }
    QNetworkReply* const reply = _activeReplies.take(requestId);
    if (reply) {
        reply->abort();  // finished handler is a no-op once the id is not pending
    }
}

void TileImageSource::_fetchFromNetwork(int requestId, const TileMath::TileKey& key)
{
    const QNetworkRequest request = QGeoTileFetcherQGC::getNetworkRequest(_mapId, key.x, key.y, key.zoom);
    QNetworkReply* const reply = _networkManager->get(request);
    _activeReplies.insert(requestId, reply);

    connect(reply, &QNetworkReply::finished, this, [this, requestId, key, reply] {
        reply->deleteLater();
        _activeReplies.remove(requestId);
        if (!_pending.contains(requestId)) {
            return;  // cancelled (abort also lands here)
        }
        if (reply->error() != QNetworkReply::NoError) {
            qCDebug(GeoMapTileImageSourceLog)
                << "network fetch failed for" << key << reply->error() << reply->errorString() << "httpStatus"
                << reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
            _finishFailed(requestId);
            return;
        }
        const QByteArray data = reply->readAll();
        if (data.isEmpty() || isBingEmptyTile(_mapId, data)) {
            qCDebug(GeoMapTileImageSourceLog)
                << "network fetch for" << key
                << (data.isEmpty() ? "returned empty body" : "returned Bing no-tile placeholder");
            _finishFailed(requestId);
            return;
        }
        QImage image;
        if (!image.loadFromData(data)) {
            qCDebug(GeoMapTileImageSourceLog)
                << "network body for" << key << "failed to decode," << data.size() << "bytes";
            _finishFailed(requestId);  // never cache undecodable bodies (e.g. HTTP-200 error pages)
            return;
        }
        // Store back so the shared cache serves this tile from now on
        qCDebug(GeoMapTileImageSourceVerboseLog) << "tile" << key << "fetched from network, caching";
        QGeoFileTileCacheQGC::cacheTile(_mapType, key.x, key.y, key.zoom, data,
                                        UrlFactory::getImageFormat(_mapType, data));
        _finishSucceeded(requestId, image);
    });
}

void TileImageSource::_deliver(int requestId, const TileMath::TileKey& key, const QByteArray& data)
{
    QImage image;
    if (isBingEmptyTile(_mapId, data) || !image.loadFromData(data)) {
        qCDebug(GeoMapTileImageSourceLog) << "cached tile for" << key << "is unusable (placeholder or decode failure)";
        _finishFailed(requestId);
        return;
    }
    qCDebug(GeoMapTileImageSourceVerboseLog) << "tile" << key << "served from cache";
    _finishSucceeded(requestId, image);
}

void TileImageSource::_finishSucceeded(int requestId, const QImage& image)
{
    _pending.remove(requestId);
    emit tileImageReady(requestId, image);
}

void TileImageSource::_finishFailed(int requestId)
{
    _pending.remove(requestId);
    emit tileImageFailed(requestId);
}
