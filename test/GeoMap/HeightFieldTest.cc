#include "HeightFieldTest.h"

#include <QtTest/QSignalSpy>

#include "HeightField.h"

using namespace TileMath;

namespace {

/// 4x4 gradient grid h(row, col) = col*10 + row*40: linear in both axes, so
/// bilinear sampling must reproduce the plane exactly (uniform grids couldn't
/// catch sub-window or axis-swap errors)
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

ElevationTilePyramid::Grid uniformGrid(float height)
{
    ElevationTilePyramid::Grid grid;
    grid.width = 4;
    grid.height = 4;
    grid.heights = QList<float>(16, height);
    return grid;
}

/// World position of a patch vertex, row-major from the NW corner
QPointF vertexWorld(const TileKey& key, int gridSize, int row, int col)
{
    const QPointF corner = tileMinCorner(key);
    const double span = tileSpanAtZoom(key.zoom);
    return QPointF(corner.x() + (span * col / gridSize), corner.y() + (span * (gridSize - row) / gridSize));
}

}  // namespace

void HeightFieldTest::_zeroWhenEmpty()
{
    const HeightField field;
    QCOMPARE(field.tileCount(), 0);
    QCOMPARE(field.heightAt(QPointF(0, 0)), 0.0);
    QCOMPARE(field.heightAt(QPointF(worldSize() / 4, -worldSize() / 4)), 0.0);

    // Patches still mesh on an empty field: full-size, all-zero drape
    const QList<float> heights = field.samplePatch(TileKey{5, 6, 3}, 4);
    QCOMPARE(heights.size(), 25);
    for (const float h : heights) {
        QCOMPARE(h, 0.0f);
    }
}

void HeightFieldTest::_invalidRequests()
{
    HeightField field;
    QVERIFY(!field.insertTile(TileKey{0, 0, -1}, gradientGrid()));
    QVERIFY(!field.insertTile(TileKey{0, 0, 0}, ElevationTilePyramid::Grid{}));
    QCOMPARE(field.tileCount(), 0);

    QVERIFY(field.samplePatch(TileKey{5, 6, 3}, 0).isEmpty());
    QVERIFY(field.samplePatch(TileKey{5, 6, 3}, -1).isEmpty());
    QVERIFY(field.samplePatch(TileKey{5, 6, 3}, HeightField::kMaxGridSize + 1).isEmpty());
    QVERIFY(field.samplePatch(TileKey{0, 0, -1}, 4).isEmpty());
    QVERIFY(field.samplePatch(TileKey{0, 0, kMaxZoom + 1}, 4).isEmpty());
    QVERIFY(field.samplePatch(TileKey{-3, 0, 3}, 4).isEmpty());
    QVERIFY(field.samplePatch(TileKey{0, 8, 3}, 4).isEmpty());
}

void HeightFieldTest::_exactValuesAtSampleCenters()
{
    HeightField field;
    QVERIFY(field.insertTile(TileKey{0, 0, 0}, gradientGrid()));

    // Sample center of grid cell (col 2, row 1) in the z0 tile: u=(2+0.5)/4,
    // v=(1+0.5)/4 from the NW corner
    const double span = worldSize();
    const double half = span / 2.0;
    const QPointF world((0.625 * span) - half, half - (0.375 * span));
    QCOMPARE(field.heightAt(world), 60.0);  // 2*10 + 1*40
}

void HeightFieldTest::_bilinearBetweenCenters()
{
    HeightField field;
    QVERIFY(field.insertTile(TileKey{0, 0, 0}, gradientGrid()));

    // Halfway between the (col 1, row 1) and (col 2, row 1) centers:
    // px = 1.5, py = 1.0 on the plane col*10 + row*40
    const double span = worldSize();
    const double half = span / 2.0;
    const QPointF world((0.5 * span) - half, half - (0.375 * span));
    QCOMPARE(field.heightAt(world), 55.0);
}

void HeightFieldTest::_clampAtTileEdges()
{
    HeightField field;
    QVERIFY(field.insertTile(TileKey{0, 0, 0}, gradientGrid()));

    // Beyond the outermost sample centers the height clamps to the edge value
    const double half = worldSize() / 2.0;
    QCOMPARE(field.heightAt(QPointF(-half, half)), 0.0);    // NW corner: h(0,0)
    QCOMPARE(field.heightAt(QPointF(half, -half)), 150.0);  // SE corner: h(3,3)
}

void HeightFieldTest::_finestTileWins()
{
    HeightField field;
    QVERIFY(field.insertTile(TileKey{0, 0, 0}, uniformGrid(100.0f)));
    QVERIFY(field.insertTile(TileKey{5, 6, 3}, uniformGrid(200.0f)));

    // Inside the fine tile: fine data; outside it: the z0 estimate
    const QPointF insideFine = vertexWorld(TileKey{5, 6, 3}, 2, 1, 1);
    const QPointF outsideFine = vertexWorld(TileKey{1, 1, 3}, 2, 1, 1);
    QCOMPARE(field.heightAt(insideFine), 200.0);
    QCOMPARE(field.heightAt(outsideFine), 100.0);
}

void HeightFieldTest::_sharedEdgeIdentity()
{
    HeightField field;
    QVERIFY(field.insertTile(TileKey{0, 0, 0}, gradientGrid()));

    // Two patches sharing a vertical edge: east column of A and west column
    // of B sample identical world positions, so heights must match exactly
    const int gridSize = 4;
    const QList<float> a = field.samplePatch(TileKey{5, 6, 3}, gridSize);
    const QList<float> b = field.samplePatch(TileKey{6, 6, 3}, gridSize);
    QCOMPARE(a.size(), 25);
    QCOMPARE(b.size(), 25);
    for (int row = 0; row <= gridSize; row++) {
        QCOMPARE(a[(row * (gridSize + 1)) + gridSize], b[row * (gridSize + 1)]);
    }
}

void HeightFieldTest::_crossZoomVertexIdentity()
{
    HeightField field;
    QVERIFY(field.insertTile(TileKey{0, 0, 0}, gradientGrid()));

    // Child patch {10,12,4} is the NW quarter of {5,6,3} and its half-density
    // grid lands exactly on parent vertices (r,c) for r,c in 0..2: coincident
    // positions must sample identical heights across the LOD boundary
    const QList<float> parent = field.samplePatch(TileKey{5, 6, 3}, 4);
    const QList<float> child = field.samplePatch(TileKey{10, 12, 4}, 2);

    for (int row = 0; row <= 2; row++) {
        for (int col = 0; col <= 2; col++) {
            QCOMPARE(child[(row * 3) + col], parent[(row * 5) + col]);
        }
    }
}

void HeightFieldTest::_regionChangedOnInsert()
{
    HeightField field;
    QSignalSpy spy(&field, &HeightField::regionChanged);
    QVERIFY(spy.isValid());

    // Fine tile: rect is exactly the tile's world extent
    const TileKey fineKey{5, 6, 3};
    QVERIFY(field.insertTile(fineKey, uniformGrid(10.0f)));
    QCOMPARE(spy.count(), 1);
    const QPointF fineCorner = tileMinCorner(fineKey);
    const double fineSpan = tileSpanAtZoom(fineKey.zoom);
    QCOMPARE(spy[0][0].toRectF(), QRectF(fineCorner.x(), fineCorner.y(), fineSpan, fineSpan));

    // Ancestor tile: notifies its full (here: whole-world) area
    QVERIFY(field.insertTile(TileKey{0, 0, 0}, uniformGrid(20.0f)));
    QCOMPARE(spy.count(), 2);
    const double world = worldSize();
    QCOMPARE(spy[1][0].toRectF(), QRectF(-world / 2.0, -world / 2.0, world, world));

    // Replacing a tile changes heights under the same area: must re-notify
    QVERIFY(field.insertTile(fineKey, uniformGrid(30.0f)));
    QCOMPARE(spy.count(), 3);
    QCOMPARE(spy[2][0].toRectF(), spy[0][0].toRectF());
}

void HeightFieldTest::_noRegionChangedOnRejectedInsert()
{
    HeightField field;
    QSignalSpy spy(&field, &HeightField::regionChanged);
    QVERIFY(spy.isValid());

    QVERIFY(!field.insertTile(TileKey{0, 0, -1}, uniformGrid(10.0f)));
    QVERIFY(!field.insertTile(TileKey{0, 0, 0}, ElevationTilePyramid::Grid{}));
    QCOMPARE(spy.count(), 0);
}

UT_REGISTER_TEST_LIGHTWEIGHT(HeightFieldTest, TestLabel::Unit)
