/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "ElevationTilePyramid.h"

#include <limits>
#include <utility>

#include "QGCLoggingCategory.h"

QGC_LOGGING_CATEGORY(GeoMapElevationTilePyramidVerboseLog, "GeoMap.ElevationTilePyramid.Verbose")

namespace {

/// Applies \a delta to the descendant count of every ancestor of \a key,
/// pruning zeroed entries
void adjustAncestorCounts(QHash<TileMath::TileKey, int>& counts, const TileMath::TileKey& key, int delta)
{
    for (int zoom = key.zoom - 1; zoom >= TileMath::kMinZoom; zoom--) {
        const int shift = key.zoom - zoom;
        const TileMath::TileKey ancestor{key.x >> shift, key.y >> shift, zoom};
        const int count = counts.value(ancestor, 0) + delta;
        if (count > 0) {
            counts.insert(ancestor, count);
        } else {
            counts.remove(ancestor);
        }
    }
}

}  // namespace

bool ElevationTilePyramid::insertTile(const TileMath::TileKey& key, Grid grid)
{
    if (!TileMath::isValidKey(key) || !grid.isValid()) {
        return false;
    }
    const bool replacing = _tiles.contains(key);
    if (!replacing && (_tiles.count() >= kMaxTiles)) {
        _evictLeastRecentlyUsed();
    }
    _tiles.insert(key, std::move(grid));
    _lastUsed.insert(key, ++_useTick);
    if (!replacing) {
        adjustAncestorCounts(_descendantCounts, key, +1);
    }
    return true;
}

void ElevationTilePyramid::_evictLeastRecentlyUsed()
{
    TileMath::TileKey lruKey{};
    qint64 lruTick = std::numeric_limits<qint64>::max();
    for (auto it = _lastUsed.cbegin(); it != _lastUsed.cend(); ++it) {
        if (it.value() < lruTick) {
            lruTick = it.value();
            lruKey = it.key();
        }
    }
    // Verbose: fires per insert once the working set is full
    qCDebug(GeoMapElevationTilePyramidVerboseLog)
        << "evicting least-recently-used tile" << lruKey << "tileCount" << _tiles.count();
    _tiles.remove(lruKey);
    _lastUsed.remove(lruKey);
    adjustAncestorCounts(_descendantCounts, lruKey, -1);
}

ElevationTilePyramid::View ElevationTilePyramid::bestTileFor(const TileMath::TileKey& key) const
{
    // Valid zoom bounds also cap the ancestor-shift distance below UB
    if (!TileMath::isValidKey(key)) {
        return View{};
    }
    _lookupCount++;
    for (int zoom = key.zoom; zoom >= TileMath::kMinZoom; zoom--) {
        const int shift = key.zoom - zoom;
        const TileMath::TileKey candidate{key.x >> shift, key.y >> shift, zoom};
        const auto it = _tiles.constFind(candidate);
        if (it == _tiles.cend()) {
            continue;
        }
        _lastUsed.insert(candidate, ++_useTick);
        const double scale = 1.0 / (1LL << shift);
        return View{&it.value(), candidate,
                    QRectF((key.x - (qint64(candidate.x) << shift)) * scale,
                           (key.y - (qint64(candidate.y) << shift)) * scale, scale, scale)};
    }
    return View{};
}
