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
#include <QtCore/QVarLengthArray>

#include "ElevationTilePyramid.h"
#include "TileMath.h"

/// Canonical vertex-height resolution for one patch: interior vertices
/// sample the patch's own backing view; boundary vertices resolve by
/// position so every patch sharing the vertex samples bit-identical heights
/// (see HeightField::samplePatch). gridSize must be a power of two so
/// vertex UVs are exact dyadic doubles. Single-use: construct, sample() once.
class PatchSampler
{
public:
    /// Largest gap between a boundary vertex resolved from a different-zoom tile
    /// (typically a neighbor's) and the patch's own view at that vertex: the
    /// step the mesh draws one cell inside the edge
    struct EdgeStep
    {
        float step = 0.0f;      ///< meters
        int row = 0;            ///< vertex of the largest gap
        int col = 0;
        int ownZoom = -1;       ///< zoom of the patch's backing tile; -1 = no data (interior renders 0)
        int boundaryZoom = -1;  ///< zoom of the tile the boundary vertex resolved to
    };

    /// \a edgeStep, when set, receives the patch's largest EdgeStep
    PatchSampler(const ElevationTilePyramid& pyramid, const TileMath::TileKey& key, int gridSize,
                 EdgeStep* edgeStep = nullptr);

    /// The (gridSize+1)^2 vertex heights, row-major from the NW corner
    QList<float> sample();

    /// Bilinear height at a unit-UV position within a grid (origin NW corner),
    /// sample-center convention, clamped at grid edges (see BilinearUV.h)
    static double heightAtUV(const ElevationTilePyramid::Grid& grid, double u, double v);

private:
    float _viewHeight(const ElevationTilePyramid::View& view, qint64 n, qint64 m) const;
    float _boundaryHeight(qint64 n, qint64 m);
    void _trackEdgeStep(const ElevationTilePyramid::View& view, float height, qint64 n, qint64 m);
    ElevationTilePyramid::View _resolveCell(qint64 cx, qint64 cy);
    int _touchingCells(qint64 s, qint64 (&cells)[2]) const;

    /// Memo of resolved views per key.zoom tile: when a tile has no stored
    /// descendant, every cell inside it resolves identically (chain below the
    /// tile is empty, ancestors are shared), so one lookup answers the whole
    /// edge run along it. An invalid view memoizes the same way — repeated
    /// misses over uncovered neighbors stay O(1). A patch's boundary touches
    /// at most 9 such tiles (own + 8 neighbors).
    struct TileMemo
    {
        TileMath::TileKey tile;
        ElevationTilePyramid::View view;
    };

    const ElevationTilePyramid& _pyramid;
    const TileMath::TileKey _key;
    const int _gridSize;
    const int _shiftToMax;
    const ElevationTilePyramid::View _patchView;
    EdgeStep* const _edgeStep;
    QVarLengthArray<TileMemo, 9> _memos;
};
