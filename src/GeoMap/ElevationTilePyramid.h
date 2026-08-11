/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#pragma once

#include <QtCore/QHash>
#include <QtCore/QList>
#include <QtCore/QRectF>

#include "TileMath.h"

/// In-memory working set of decoded elevation tiles, keyed by slippy tile.
///
/// This is the synchronous sampling layer of the continuous-drape design: the
/// persistent store remains QGC's shared tile cache database (encoded tiles,
/// async); this pyramid retains what the fetch path has already decoded so a
/// height estimate is answerable immediately at mesh time. Lookup serves a
/// query tile from itself or its nearest stored ancestor with the sub-window
/// to sample — never a descendant — so coverage is continuous wherever any
/// ancestor data exists.
///
/// Not thread-safe: confine to one thread or synchronize externally.
/// No eviction yet — tiles accumulate for the session (follow-up: LRU cap).
class ElevationTilePyramid
{
public:
    /// Decoded elevation samples for one tile, row-major from the NW corner
    struct Grid
    {
        int width = 0;         ///< samples per row
        int height = 0;        ///< rows
        QList<float> heights;  ///< meters

        bool isValid() const { return (width > 0) && (height > 0) && (heights.size() == qsizetype(width) * height); }
    };

    /// Where to sample a query tile: the stored tile and the query's extent
    /// within it (unit UV from the NW corner)
    struct View
    {
        const Grid* grid = nullptr;  ///< null when nothing stored covers the query
        TileMath::TileKey key;       ///< stored tile the view samples
        QRectF subWindow;

        bool isValid() const { return grid != nullptr; }
    };

    /// Stores \a grid for \a key, replacing any previous tile. Invalid grids
    /// are rejected (returns false).
    bool insertTile(const TileMath::TileKey& key, Grid grid);

    bool hasTile(const TileMath::TileKey& key) const { return _tiles.contains(key); }

    /// Finest stored tile covering \a key: the tile itself when present, else
    /// the nearest ancestor. View pointers stay valid until the next insert.
    View bestTileFor(const TileMath::TileKey& key) const;

    int tileCount() const { return static_cast<int>(_tiles.count()); }

private:
    QHash<TileMath::TileKey, Grid> _tiles;
};
