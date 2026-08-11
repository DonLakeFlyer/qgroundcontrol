#include "ElevationTilePyramidTest.h"

#include "ElevationTilePyramid.h"

namespace {

/// Uniform-height grid, big enough to be a plausible tile
ElevationTilePyramid::Grid makeGrid(float height, int size = 4)
{
    ElevationTilePyramid::Grid grid;
    grid.width = size;
    grid.height = size;
    grid.heights = QList<float>(qsizetype(size) * size, height);
    return grid;
}

}  // namespace

void ElevationTilePyramidTest::_emptyPyramidHasNoView()
{
    const ElevationTilePyramid pyramid;
    QCOMPARE(pyramid.tileCount(), 0);
    QVERIFY(!pyramid.hasTile(TileMath::TileKey{0, 0, 0}));
    QVERIFY(!pyramid.bestTileFor(TileMath::TileKey{5, 6, 3}).isValid());
}

void ElevationTilePyramidTest::_insertRejectsInvalidGrid()
{
    ElevationTilePyramid pyramid;
    const TileMath::TileKey key{1, 1, 1};

    QVERIFY(!pyramid.insertTile(key, ElevationTilePyramid::Grid{}));

    // Sample count inconsistent with dimensions
    ElevationTilePyramid::Grid bad = makeGrid(10.0f);
    bad.heights.removeLast();
    QVERIFY(!pyramid.insertTile(key, bad));

    // Zero rows with nonzero width
    ElevationTilePyramid::Grid zeroRows = makeGrid(10.0f);
    zeroRows.height = 0;
    QVERIFY(!pyramid.insertTile(key, zeroRows));

    QCOMPARE(pyramid.tileCount(), 0);
    QVERIFY(!pyramid.hasTile(key));
}

void ElevationTilePyramidTest::_rejectsInvalidKey()
{
    ElevationTilePyramid pyramid;
    QVERIFY(!pyramid.insertTile(TileMath::TileKey{0, 0, TileMath::kMaxZoom + 1}, makeGrid(1.0f)));
    QVERIFY(!pyramid.insertTile(TileMath::TileKey{0, 0, -1}, makeGrid(1.0f)));

    // x/y outside the slippy range for the zoom
    QVERIFY(!pyramid.insertTile(TileMath::TileKey{-1, 0, 3}, makeGrid(1.0f)));
    QVERIFY(!pyramid.insertTile(TileMath::TileKey{0, -1, 3}, makeGrid(1.0f)));
    QVERIFY(!pyramid.insertTile(TileMath::TileKey{0, 8, 3}, makeGrid(1.0f)));
    QCOMPARE(pyramid.tileCount(), 0);

    // Absurd zoom must fail cleanly, not walk the shift into UB
    QVERIFY(pyramid.insertTile(TileMath::TileKey{0, 0, 0}, makeGrid(1.0f)));
    QVERIFY(!pyramid.bestTileFor(TileMath::TileKey{0, 0, TileMath::kMaxZoom + 1}).isValid());
    QVERIFY(!pyramid.bestTileFor(TileMath::TileKey{0, 0, 500}).isValid());
    QVERIFY(!pyramid.bestTileFor(TileMath::TileKey{0, 0, -1}).isValid());
    QVERIFY(!pyramid.bestTileFor(TileMath::TileKey{-1, 0, 3}).isValid());
    QVERIFY(!pyramid.bestTileFor(TileMath::TileKey{8, 0, 3}).isValid());
}

void ElevationTilePyramidTest::_selfTileWins()
{
    ElevationTilePyramid pyramid;
    const TileMath::TileKey key{5, 6, 3};
    QVERIFY(pyramid.insertTile(TileMath::TileKey{1, 1, 1}, makeGrid(100.0f)));
    QVERIFY(pyramid.insertTile(key, makeGrid(200.0f)));
    QVERIFY(pyramid.hasTile(key));

    const ElevationTilePyramid::View view = pyramid.bestTileFor(key);
    QVERIFY(view.isValid());
    QCOMPARE(view.key, key);
    QCOMPARE(view.subWindow, QRectF(0, 0, 1, 1));
    QCOMPARE(view.grid->heights.first(), 200.0f);
}

void ElevationTilePyramidTest::_nearestAncestorWins()
{
    ElevationTilePyramid pyramid;
    // Ancestors of (5,6,3): (2,3,2), (1,1,1), (0,0,0)
    QVERIFY(pyramid.insertTile(TileMath::TileKey{0, 0, 0}, makeGrid(1.0f)));
    QVERIFY(pyramid.insertTile(TileMath::TileKey{1, 1, 1}, makeGrid(2.0f)));
    QVERIFY(pyramid.insertTile(TileMath::TileKey{2, 3, 2}, makeGrid(3.0f)));

    const ElevationTilePyramid::View view = pyramid.bestTileFor(TileMath::TileKey{5, 6, 3});
    QVERIFY(view.isValid());
    QCOMPARE(view.key, (TileMath::TileKey{2, 3, 2}));
    QCOMPARE(view.grid->heights.first(), 3.0f);
}

void ElevationTilePyramidTest::_noDescendantFallback()
{
    ElevationTilePyramid pyramid;
    // Child and unrelated sibling of the query — neither covers it
    QVERIFY(pyramid.insertTile(TileMath::TileKey{10, 12, 4}, makeGrid(1.0f)));
    QVERIFY(pyramid.insertTile(TileMath::TileKey{4, 6, 3}, makeGrid(2.0f)));

    QVERIFY(!pyramid.bestTileFor(TileMath::TileKey{5, 6, 3}).isValid());
}

void ElevationTilePyramidTest::_subWindowMath()
{
    ElevationTilePyramid pyramid;
    QVERIFY(pyramid.insertTile(TileMath::TileKey{1, 1, 1}, makeGrid(5.0f)));

    // (5,6,3) within ancestor (1,1,1): shift 2, quarter-tile window
    const ElevationTilePyramid::View view = pyramid.bestTileFor(TileMath::TileKey{5, 6, 3});
    QVERIFY(view.isValid());
    QCOMPARE(view.key, (TileMath::TileKey{1, 1, 1}));
    QCOMPARE(view.subWindow, QRectF(0.25, 0.5, 0.25, 0.25));
}

void ElevationTilePyramidTest::_subWindowDeepZoom()
{
    ElevationTilePyramid pyramid;
    const TileMath::TileKey ancestor{7, 2, 3};
    QVERIFY(pyramid.insertTile(ancestor, makeGrid(5.0f)));

    // 10 zoom levels deeper: NW-most descendant maps to a tiny window at the
    // ancestor's NW corner
    const int shift = 10;
    const double scale = 1.0 / (1 << shift);
    const TileMath::TileKey nwChild{7 << shift, 2 << shift, 3 + shift};
    const ElevationTilePyramid::View nwView = pyramid.bestTileFor(nwChild);
    QVERIFY(nwView.isValid());
    QCOMPARE(nwView.subWindow, QRectF(0, 0, scale, scale));

    // SE-most descendant maps to the far corner
    const TileMath::TileKey seChild{(8 << shift) - 1, (3 << shift) - 1, 3 + shift};
    const ElevationTilePyramid::View seView = pyramid.bestTileFor(seChild);
    QVERIFY(seView.isValid());
    QCOMPARE(seView.subWindow, QRectF(1.0 - scale, 1.0 - scale, scale, scale));
}

void ElevationTilePyramidTest::_insertReplacesTile()
{
    ElevationTilePyramid pyramid;
    const TileMath::TileKey key{5, 6, 3};
    QVERIFY(pyramid.insertTile(key, makeGrid(1.0f)));
    QVERIFY(pyramid.insertTile(key, makeGrid(2.0f)));

    QCOMPARE(pyramid.tileCount(), 1);
    QCOMPARE(pyramid.bestTileFor(key).grid->heights.first(), 2.0f);
}

UT_REGISTER_TEST_LIGHTWEIGHT(ElevationTilePyramidTest, TestLabel::Unit)
