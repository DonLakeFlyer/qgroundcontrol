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
#include <QtCore/QRectF>
#include <QtQuick3D/QQuick3DGeometry>

#include <array>

#include "TileMath.h"

class HeightField;

Q_MOC_INCLUDE("HeightField.h")

/// Grid mesh for one surface patch of the GeoMap engine.
///
/// Local space: origin at the patch center, x east, y north, z up (meters).
/// The grid has (gridSize+1)^2 vertices spanning [-span/2, +span/2] in x/y,
/// displaced in z by the height grid (row-major from the north-west corner,
/// matching HeightSource). Empty heights produce a flat patch at z=0.
///
/// Skirt geometry hangs from all four edges to hide cracks between adjacent
/// patches of different LOD levels. UVs map (0,0) at the north-west corner to
/// (1,1) at the south-east corner, matching raster tile orientation.
///
/// Known limitation: normals are central differences over this patch's own
/// height grid (one-sided at edges), so lit materials show shading seams at
/// patch borders. Fix when lit terrain matters (M2): deliver a one-sample
/// apron from the HeightSource so edge normals use neighboring data.
class PatchGeometry : public QQuick3DGeometry
{
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(int gridSize READ gridSize WRITE setGridSize NOTIFY gridSizeChanged)
    Q_PROPERTY(qreal span READ span WRITE setSpan NOTIFY spanChanged)
    Q_PROPERTY(QList<float> heights READ heights WRITE setHeights NOTIFY heightsChanged)
    Q_PROPERTY(QList<int> edgeLodDeltas READ edgeLodDeltas WRITE setEdgeLodDeltas NOTIFY edgeLodDeltasChanged)
    Q_PROPERTY(HeightField* heightField READ heightField WRITE setHeightField NOTIFY heightFieldChanged)
    Q_PROPERTY(int tileX READ tileX WRITE setTileX NOTIFY tileKeyChanged)
    Q_PROPERTY(int tileY READ tileY WRITE setTileY NOTIFY tileKeyChanged)
    Q_PROPERTY(int tileZoom READ tileZoom WRITE setTileZoom NOTIFY tileKeyChanged)

public:
    explicit PatchGeometry(QQuick3DObject* parent = nullptr);

    static constexpr int kMinGridSize = 1;
    static constexpr int kMaxGridSize = 256;
    static constexpr double kSkirtDepthFraction = 0.05;  ///< skirt depth as fraction of span

    int gridSize() const { return _gridSize; }

    void setGridSize(int gridSize);

    qreal span() const { return _span; }

    void setSpan(qreal span);

    /// (gridSize+1)^2 heights row-major from the north-west corner; empty = flat
    QList<float> heights() const { return _heights; }

    void setHeights(const QList<float>& heights);

    /// Field to sample vertex heights from; not owned, may be null. Watches
    /// the field's regionChanged to re-resolve constrained edges whose coarse
    /// neighbor data changes without touching this patch's own heights.
    HeightField* heightField() const { return _heightField; }

    void setHeightField(HeightField* heightField);

    /// This patch's own slippy tile key, needed (with the field) to resolve
    /// constrained edges from the coarse neighbor's actual backing tile.
    /// tileZoom < 0 (the default) means no key context: the stitch falls
    /// back to own-sample lerp.
    int tileX() const { return _key.x; }

    void setTileX(int tileX);

    int tileY() const { return _key.y; }

    void setTileY(int tileY);

    int tileZoom() const { return _key.zoom; }

    void setTileZoom(int tileZoom);

    /// Samples the height field at this patch's vertex world positions (tile
    /// \a key at the current gridSize) and rebuilds the mesh. Returns false
    /// when no field is set or the key is invalid.
    bool sampleFromField(const TileMath::TileKey& key);

    /// How many LOD levels coarser each edge's neighbor renders (0 = same or
    /// finer: unconstrained). Edge vertices are constrained to the coarse
    /// neighbor's real rendered edge, resolved from the height field (see
    /// _coarseEdgeSamples) so no T-junction cracks open.
    void setEdgeLodDeltas(int north, int south, int west, int east);

    /// QML-bindable form of the deltas: {north, south, west, east}
    QList<int> edgeLodDeltas() const
    {
        return {_lodDelta[kNorth], _lodDelta[kSouth], _lodDelta[kWest], _lodDelta[kEast]};
    }

    void setEdgeLodDeltas(const QList<int>& deltas);

signals:
    void gridSizeChanged();
    void spanChanged();
    void heightsChanged();
    void edgeLodDeltasChanged();
    void heightFieldChanged();
    void tileKeyChanged();

protected:
    void componentComplete() override;

private:
    /// Edge index order shared by the delta/offset/sample member arrays: N,S,W,E
    enum Edge
    {
        kNorth,
        kSouth,
        kWest,
        kEast,
        kEdgeCount
    };

    void _rebuild();
    /// Defers to one build at componentComplete during QML instantiation;
    /// immediate otherwise (C++ construction is always "complete")
    void _requestRebuild();
    void _fieldRegionChanged(const QRectF& worldRect);
    float _heightAt(int row, int col) const;
    float _rawHeightAt(int row, int col) const;
    void _resolveCoarseEdges();
    QList<float> _coarseEdgeSamples(Edge edge) const;
    float _coarseHeightAt(Edge edge, int alongIdx) const;

    int _gridSize = 16;
    qreal _span = 1000.0;
    QList<float> _heights;
    HeightField* _heightField = nullptr;
    TileMath::TileKey _key{0, 0, -1};  ///< this patch's own key; invalid until sampleFromField
    std::array<int, kEdgeCount> _lodDelta{};

    /// Real neighbor edge samples for each coarser edge, resolved from the
    /// height field at the actual resident ancestor covering that neighbor
    /// (see _resolveCoarseEdges); empty when unconstrained or unresolvable.
    std::array<QList<float>, kEdgeCount> _coarseEdge;
};
