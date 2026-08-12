#include "PatchGeometryTest.h"

#include <QtCore/QRegularExpression>
#include <QtTest/QSignalSpy>

#include <cmath>

#include "HeightField.h"
#include "PatchGeometry.h"

namespace {

constexpr int kGrid = 4;
constexpr float kSpan = 1000.0f;
constexpr int kFloatsPerVertex = 8;  // position 3, normal 3, uv 2

int gridVertexCount(int gridSize)
{
    return (gridSize + 1) * (gridSize + 1);
}

int skirtVertexCount(int gridSize)
{
    return 4 * (gridSize + 1);
}

const float* vertexAt(const QByteArray& data, int index)
{
    return reinterpret_cast<const float*>(data.constData()) + (index * kFloatsPerVertex);
}

QList<float> rampHeights(int gridSize)
{
    // Height = 10 * row: north edge at 0, south edge highest
    QList<float> heights;
    for (int row = 0; row <= gridSize; row++) {
        for (int col = 0; col <= gridSize; col++) {
            heights.append(10.0f * row);
        }
    }
    return heights;
}

ElevationTilePyramid::Grid uniformGrid(float height)
{
    ElevationTilePyramid::Grid grid;
    grid.width = 4;
    grid.height = 4;
    grid.heights = QList<float>(16, height);
    return grid;
}

/// Linear in both axes so bilinear sampling reproduces the plane and edge
/// vertices carry distinct values (uniform grids couldn't catch edge bugs)
ElevationTilePyramid::Grid gradientGrid()
{
    ElevationTilePyramid::Grid grid;
    grid.width = 4;
    grid.height = 4;
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            grid.heights.append(float((col * 10) + (row * 40)));
        }
    }
    return grid;
}

}  // namespace

void PatchGeometryTest::_flatMeshLayout()
{
    PatchGeometry geometry;
    geometry.setGridSize(kGrid);
    geometry.setSpan(kSpan);

    const QByteArray vertexData = geometry.vertexData();
    const int expectedVertices = gridVertexCount(kGrid) + skirtVertexCount(kGrid);
    QCOMPARE(vertexData.size(), expectedVertices * kFloatsPerVertex * static_cast<int>(sizeof(float)));

    const QByteArray indexData = geometry.indexData();
    const int expectedIndices = (kGrid * kGrid * 6) + (4 * kGrid * 6);
    QCOMPARE(indexData.size(), expectedIndices * static_cast<int>(sizeof(quint32)));

    // All indices reference valid vertices
    const quint32* indices = reinterpret_cast<const quint32*>(indexData.constData());
    for (int i = 0; i < expectedIndices; i++) {
        QCOMPARE_LT(indices[i], static_cast<quint32>(expectedVertices));
    }
}

void PatchGeometryTest::_cornerPositionsAndUVs()
{
    PatchGeometry geometry;
    geometry.setGridSize(kGrid);
    geometry.setSpan(kSpan);
    const QByteArray data = geometry.vertexData();

    // Vertex 0 is the north-west corner: (-span/2, +span/2), uv (0,0)
    const float* nw = vertexAt(data, 0);
    QCOMPARE(nw[0], -kSpan / 2);
    QCOMPARE(nw[1], kSpan / 2);
    QCOMPARE(nw[2], 0.0f);
    QCOMPARE(nw[6], 0.0f);
    QCOMPARE(nw[7], 0.0f);

    // Last grid vertex is the south-east corner: (+span/2, -span/2), uv (1,1)
    const float* se = vertexAt(data, gridVertexCount(kGrid) - 1);
    QCOMPARE(se[0], kSpan / 2);
    QCOMPARE(se[1], -kSpan / 2);
    QCOMPARE(se[6], 1.0f);
    QCOMPARE(se[7], 1.0f);
}

void PatchGeometryTest::_heightDisplacement()
{
    PatchGeometry geometry;
    geometry.setGridSize(kGrid);
    geometry.setSpan(kSpan);
    geometry.setHeights(rampHeights(kGrid));
    const QByteArray data = geometry.vertexData();

    // Row-major from NW: vertex (row, col) has z = 10 * row
    for (int row = 0; row <= kGrid; row++) {
        for (int col = 0; col <= kGrid; col++) {
            const float* vertex = vertexAt(data, (row * (kGrid + 1)) + col);
            QCOMPARE(vertex[2], 10.0f * row);
        }
    }
}

void PatchGeometryTest::_mismatchedHeightsRenderFlat()
{
    PatchGeometry geometry;
    geometry.setGridSize(kGrid);
    geometry.setHeights(QList<float>{1.0f, 2.0f, 3.0f});  // wrong count
    const QByteArray data = geometry.vertexData();

    for (int i = 0; i < gridVertexCount(kGrid); i++) {
        QCOMPARE(vertexAt(data, i)[2], 0.0f);
    }
}

void PatchGeometryTest::_skirtVerticesBelowSurface()
{
    PatchGeometry geometry;
    geometry.setGridSize(kGrid);
    geometry.setSpan(kSpan);
    geometry.setHeights(rampHeights(kGrid));
    const QByteArray data = geometry.vertexData();

    const float skirtDepth = kSpan * static_cast<float>(PatchGeometry::kSkirtDepthFraction);
    // First skirt vertex duplicates the NW corner, dropped by skirtDepth
    const float* nwSkirt = vertexAt(data, gridVertexCount(kGrid));
    const float* nw = vertexAt(data, 0);
    QCOMPARE(nwSkirt[0], nw[0]);
    QCOMPARE(nwSkirt[1], nw[1]);
    QCOMPARE(nwSkirt[2], nw[2] - skirtDepth);
    // Same uv/normal as the edge vertex (no seams)
    QCOMPARE(nwSkirt[6], nw[6]);
    QCOMPARE(nwSkirt[7], nw[7]);
}

void PatchGeometryTest::_flatNormalsPointUp()
{
    PatchGeometry geometry;
    geometry.setGridSize(kGrid);
    geometry.setSpan(kSpan);
    const QByteArray data = geometry.vertexData();

    for (int i = 0; i < gridVertexCount(kGrid); i++) {
        const float* vertex = vertexAt(data, i);
        QCOMPARE(vertex[3], 0.0f);
        QCOMPARE(vertex[4], 0.0f);
        QCOMPARE(vertex[5], 1.0f);
    }
}

void PatchGeometryTest::_boundsIncludeSkirt()
{
    PatchGeometry geometry;
    geometry.setGridSize(kGrid);
    geometry.setSpan(kSpan);
    geometry.setHeights(rampHeights(kGrid));

    const float skirtDepth = kSpan * static_cast<float>(PatchGeometry::kSkirtDepthFraction);
    QCOMPARE(geometry.boundsMin().x(), -kSpan / 2);
    QCOMPARE(geometry.boundsMax().x(), kSpan / 2);
    QCOMPARE(geometry.boundsMin().z(), 0.0f - skirtDepth);
    QCOMPARE(geometry.boundsMax().z(), 10.0f * kGrid);
}

void PatchGeometryTest::_gridSizeClamped()
{
    PatchGeometry geometry;
    geometry.setGridSize(0);
    QCOMPARE(geometry.gridSize(), PatchGeometry::kMinGridSize);
    geometry.setGridSize(100000);
    QCOMPARE(geometry.gridSize(), PatchGeometry::kMaxGridSize);
}

void PatchGeometryTest::_sampleFromFieldDisplacesVertices()
{
    HeightField field;
    QVERIFY(field.insertTile(TileMath::TileKey{0, 0, 0}, uniformGrid(100.0f)));

    PatchGeometry geometry;
    geometry.setGridSize(kGrid);
    geometry.setSpan(kSpan);
    geometry.setHeightField(&field);
    QVERIFY(geometry.sampleFromField(TileMath::TileKey{5, 6, 3}));

    const QByteArray data = geometry.vertexData();
    for (int i = 0; i < gridVertexCount(kGrid); i++) {
        QCOMPARE(vertexAt(data, i)[2], 100.0f);
    }
}

void PatchGeometryTest::_sampleFromFieldWithoutFieldWarns()
{
    PatchGeometry geometry;
    geometry.setGridSize(kGrid);

    expectLogMessage("GeoMap.PatchGeometry", QtWarningMsg, QRegularExpression("^sampleFromField rejected:"));
    QVERIFY(!geometry.sampleFromField(TileMath::TileKey{5, 6, 3}));
    verifyExpectedLogMessage();

    // With a field set, an invalid key is rejected by both layers
    HeightField field;
    geometry.setHeightField(&field);
    expectLogMessage("GeoMap.HeightField", QtWarningMsg, QRegularExpression("^samplePatch rejected:"));
    expectLogMessage("GeoMap.PatchGeometry", QtWarningMsg, QRegularExpression("^sampleFromField rejected:"));
    QVERIFY(!geometry.sampleFromField(TileMath::TileKey{0, 0, -1}));
    verifyExpectedLogMessage();
    verifyExpectedLogMessage();
}

void PatchGeometryTest::_adjacentPatchesShareEdgeHeights()
{
    HeightField field;
    QVERIFY(field.insertTile(TileMath::TileKey{0, 0, 0}, gradientGrid()));

    PatchGeometry west;
    west.setGridSize(kGrid);
    west.setSpan(kSpan);
    west.setHeightField(&field);
    QVERIFY(west.sampleFromField(TileMath::TileKey{5, 6, 3}));

    PatchGeometry east;
    east.setGridSize(kGrid);
    east.setSpan(kSpan);
    east.setHeightField(&field);
    QVERIFY(east.sampleFromField(TileMath::TileKey{6, 6, 3}));

    // Core drape invariant: the shared edge samples identical world positions
    // in the same field, so the meshed heights must match bit-for-bit
    const QByteArray westData = west.vertexData();
    const QByteArray eastData = east.vertexData();
    for (int row = 0; row <= kGrid; row++) {
        const float westEdgeZ = vertexAt(westData, (row * (kGrid + 1)) + kGrid)[2];
        const float eastEdgeZ = vertexAt(eastData, row * (kGrid + 1))[2];
        QCOMPARE(westEdgeZ, eastEdgeZ);
    }

    // Sanity: the gradient varies along the edge, so the comparison is real
    QCOMPARE_NE(vertexAt(westData, kGrid)[2], vertexAt(westData, (kGrid * (kGrid + 1)) + kGrid)[2]);
}

void PatchGeometryTest::_resampleAfterFieldGainsData()
{
    HeightField field;
    PatchGeometry geometry;
    geometry.setGridSize(kGrid);
    geometry.setSpan(kSpan);
    geometry.setHeightField(&field);
    QSignalSpy spy(&geometry, &PatchGeometry::heightsChanged);
    QVERIFY(spy.isValid());

    // Empty field: sampling succeeds with the zero best-estimate everywhere
    const TileMath::TileKey key{5, 6, 3};
    QVERIFY(geometry.sampleFromField(key));
    const QByteArray flatData = geometry.vertexData();
    for (int i = 0; i < gridVertexCount(kGrid); i++) {
        QCOMPARE(vertexAt(flatData, i)[2], 0.0f);
    }
    const int emptySampleSignals = spy.count();

    // Field gains data: resampling picks it up and notifies
    QVERIFY(field.insertTile(TileMath::TileKey{0, 0, 0}, uniformGrid(100.0f)));
    QVERIFY(geometry.sampleFromField(key));
    const QByteArray sampledData = geometry.vertexData();
    for (int i = 0; i < gridVertexCount(kGrid); i++) {
        QCOMPARE(vertexAt(sampledData, i)[2], 100.0f);
    }
    QCOMPARE_GT(spy.count(), emptySampleSignals);
}

UT_REGISTER_TEST(PatchGeometryTest, TestLabel::Unit)
