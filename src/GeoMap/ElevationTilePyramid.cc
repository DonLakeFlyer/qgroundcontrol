/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "ElevationTilePyramid.h"

#include <utility>

namespace {

/// Slippy domain check: zoom bounds also cap the ancestor-shift distance below UB
bool keyInRange(const TileMath::TileKey& key)
{
    if ((key.zoom < TileMath::kMinZoom) || (key.zoom > TileMath::kMaxZoom)) {
        return false;
    }
    const int tilesAtZoom = 1 << key.zoom;
    return (key.x >= 0) && (key.x < tilesAtZoom) && (key.y >= 0) && (key.y < tilesAtZoom);
}

}  // namespace

bool ElevationTilePyramid::insertTile(const TileMath::TileKey& key, Grid grid)
{
    if (!keyInRange(key) || !grid.isValid()) {
        return false;
    }
    _tiles.insert(key, std::move(grid));
    return true;
}

ElevationTilePyramid::View ElevationTilePyramid::bestTileFor(const TileMath::TileKey& key) const
{
    if (!keyInRange(key)) {
        return View{};
    }
    for (int zoom = key.zoom; zoom >= TileMath::kMinZoom; zoom--) {
        const int shift = key.zoom - zoom;
        const TileMath::TileKey candidate{key.x >> shift, key.y >> shift, zoom};
        const auto it = _tiles.constFind(candidate);
        if (it == _tiles.cend()) {
            continue;
        }
        const double scale = 1.0 / (1LL << shift);
        return View{&it.value(), candidate,
                    QRectF((key.x - (qint64(candidate.x) << shift)) * scale,
                           (key.y - (qint64(candidate.y) << shift)) * scale, scale, scale)};
    }
    return View{};
}
