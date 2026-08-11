/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#pragma once

#include <QtCore/QList>
#include <QtCore/QPointF>

#include "ElevationTilePyramid.h"
#include "TileMath.h"

/// The one continuous terrain heightfield of the drape design: best-estimate
/// height everywhere, by construction — real data where a tile is stored,
/// ancestor-interpolated estimate where only coarser data exists, zero where
/// nothing is known. There is no "missing" region, only coarser-estimate
/// regions, so any two callers asking for the same world position always get
/// the same answer regardless of which patch they mesh.
///
/// Heights are sampled bilinearly between grid sample centers (pixel-center
/// convention, clamped at tile edges), matching the terrarium tile layout.
///
/// Not thread-safe: confine to one thread or synchronize externally.
class HeightField
{
public:
    /// Sanity cap on patch density: rejects absurd sizes before allocation
    static constexpr int kMaxGridSize = 4096;

    /// Stores a decoded tile in the backing pyramid; invalid keys/grids rejected
    bool insertTile(const TileMath::TileKey& key, ElevationTilePyramid::Grid grid);

    /// Best-estimate height (meters) at a mercator world position; 0.0 where
    /// no stored tile covers the position
    double heightAt(const QPointF& world) const;

    /// The (gridSize+1)^2 vertex heights of a patch, row-major from the NW
    /// corner. Empty for invalid keys or gridSize outside [1, kMaxGridSize].
    QList<float> samplePatch(const TileMath::TileKey& key, int gridSize) const;

    int tileCount() const { return _pyramid.tileCount(); }

private:
    ElevationTilePyramid _pyramid;
};
