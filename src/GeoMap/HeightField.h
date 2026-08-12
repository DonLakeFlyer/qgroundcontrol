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
#include <QtCore/QObject>
#include <QtCore/QPointF>
#include <QtCore/QRectF>

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
class HeightField : public QObject
{
    Q_OBJECT

public:
    explicit HeightField(QObject* parent = nullptr);

    /// Sanity cap on patch density: rejects absurd sizes before allocation
    static constexpr int kMaxGridSize = 4096;

    /// Stores a decoded tile in the backing pyramid; invalid keys/grids
    /// rejected. Emits regionChanged for the tile's world extent on success.
    bool insertTile(const TileMath::TileKey& key, ElevationTilePyramid::Grid grid);

    /// Best-estimate height (meters) at a mercator world position; 0.0 where
    /// no stored tile covers the position
    double heightAt(const QPointF& world) const;

    /// The (gridSize+1)^2 vertex heights of a patch, row-major from the NW
    /// corner. Empty for invalid keys or gridSize outside [1, kMaxGridSize].
    QList<float> samplePatch(const TileMath::TileKey& key, int gridSize) const;

    int tileCount() const { return _pyramid.tileCount(); }

    /// True when the backing pyramid holds this exact tile
    bool hasTile(const TileMath::TileKey& key) const { return _pyramid.hasTile(key); }

signals:
    /// Best-estimate heights changed within this rect (TileMath world meters,
    /// y north; min-corner + positive spans — don't use top()/bottom()).
    /// The rect is closed: patch edge vertices exactly on its boundary sample
    /// the new data, but QRectF::intersects() is false for edge-only contact,
    /// so inflate the patch rect (e.g. marginsAdded) before testing
    void regionChanged(const QRectF& worldRect);

private:
    ElevationTilePyramid _pyramid;
};
