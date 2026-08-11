/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "HeightField.h"

#include <QtCore/QtMinMax>
#include <cmath>
#include <utility>

namespace {

/// Bilinear height at a unit-UV position within a grid (origin NW corner),
/// sample-center convention, clamped at grid edges
double heightAtUV(const ElevationTilePyramid::Grid& grid, double u, double v)
{
    const int w = grid.width;
    const int h = grid.height;
    const double px = (u * w) - 0.5;
    const double py = (v * h) - 0.5;
    const int x0 = qBound(0, static_cast<int>(std::floor(px)), w - 1);
    const int y0 = qBound(0, static_cast<int>(std::floor(py)), h - 1);
    const int x1 = qMin(x0 + 1, w - 1);
    const int y1 = qMin(y0 + 1, h - 1);
    const double fx = qBound(0.0, px - x0, 1.0);
    const double fy = qBound(0.0, py - y0, 1.0);

    const auto at = [&grid](int x, int y) { return double(grid.heights[(qsizetype(y) * grid.width) + x]); };
    const double north = (at(x0, y0) * (1.0 - fx)) + (at(x1, y0) * fx);
    const double south = (at(x0, y1) * (1.0 - fx)) + (at(x1, y1) * fx);
    return (north * (1.0 - fy)) + (south * fy);
}

}  // namespace

bool HeightField::insertTile(const TileMath::TileKey& key, ElevationTilePyramid::Grid grid)
{
    return _pyramid.insertTile(key, std::move(grid));
}

double HeightField::heightAt(const QPointF& world) const
{
    // Query at the deepest zoom; the pyramid resolves the finest stored cover
    const TileMath::TileKey query = TileMath::tileForWorld(world, TileMath::kMaxZoom);
    const ElevationTilePyramid::View view = _pyramid.bestTileFor(query);
    if (!view.isValid()) {
        return 0.0;
    }

    const QPointF corner = TileMath::tileMinCorner(view.key);
    const double span = TileMath::tileSpanAtZoom(view.key.zoom);
    const double u = (world.x() - corner.x()) / span;
    const double v = ((corner.y() + span) - world.y()) / span;  // grid origin is the NW corner
    return heightAtUV(*view.grid, u, v);
}

QList<float> HeightField::samplePatch(const TileMath::TileKey& key, int gridSize) const
{
    if ((gridSize < 1) || (gridSize > kMaxGridSize) || !TileMath::isValidKey(key)) {
        return QList<float>();
    }

    const QPointF corner = TileMath::tileMinCorner(key);
    const double span = TileMath::tileSpanAtZoom(key.zoom);
    QList<float> heights;
    heights.reserve(qsizetype(gridSize + 1) * (gridSize + 1));
    for (int row = 0; row <= gridSize; row++) {
        const double y = corner.y() + (span * (gridSize - row) / gridSize);
        for (int col = 0; col <= gridSize; col++) {
            const double x = corner.x() + (span * col / gridSize);
            heights.append(static_cast<float>(heightAt(QPointF(x, y))));
        }
    }
    return heights;
}
