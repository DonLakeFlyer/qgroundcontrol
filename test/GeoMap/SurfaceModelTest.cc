#include "SurfaceModelTest.h"

#include <QtCore/QCoreApplication>
#include <QtCore/QSet>
#include <QtTest/QSignalSpy>

#include <algorithm>
#include <cmath>
#include <utility>

#include "ElevationTilePyramid.h"
#include "GeoMapCamera.h"
#include "HeightField.h"
#include "HeightSource.h"
#include "SurfaceModel.h"
#include "TileMath.h"

namespace {

const QGeoCoordinate kCenter(47.3977419, 8.5455938);
constexpr QSizeF kViewport(800, 600);

/// Constant-height decoded tile grid
ElevationTilePyramid::Grid constantGrid(float height, int size = 8)
{
    ElevationTilePyramid::Grid grid;
    grid.width = size;
    grid.height = size;
    grid.heights = QList<float>(qsizetype(size) * size, height);
    return grid;
}

QRectF tileRect(const TileMath::TileKey& key)
{
    const double span = TileMath::tileSpanAtZoom(key.zoom);
    return QRectF(TileMath::tileMinCorner(key), QSizeF(span, span));
}

/// Inflated-rect region contact, matching the model's re-mesh predicate:
/// patch edge vertices exactly on the region boundary sample the changed
/// data, but QRectF::intersects is false for edge-only contact
bool touchesRegion(const TileMath::TileKey& key, const QRectF& region)
{
    const QRectF rect = tileRect(key);
    const double margin = rect.width() * 1e-6;
    return rect.marginsAdded(QMarginsF(margin, margin, margin, margin)).intersects(region);
}

/// Flat source that records coverage requests (the field stays empty)
class RecordingCoverageSource : public FlatHeightSource
{
public:
    using FlatHeightSource::FlatHeightSource;

    bool requestTile(const TileMath::TileKey& key) override
    {
        requested.append(key);
        return false;
    }

    QList<TileMath::TileKey> requested;
};

/// Multi-octave analytic terrain rough at every patch scale, so any
/// unconstrained LOD edge shows a real height mismatch
class RoughHeightSource : public ProceduralHeightSource
{
public:
    using ProceduralHeightSource::ProceduralHeightSource;

protected:
    float heightAtWorld(const QPointF& world) const override
    {
        const double x = world.x();
        const double y = world.y();
        return static_cast<float>((300.0 * std::sin(x / 20000.0) * std::cos(y / 26000.0)) +
                                  (150.0 * std::sin(x / 1900.0) * std::cos(y / 1300.0)) +
                                  (80.0 * std::sin((x / 280.0) + 1.0) * std::cos(y / 240.0)) +
                                  (40.0 * std::sin(x / 61.0) * std::cos(y / 73.0)));
    }
};

/// Rendered height along a patch edge at fractional grid coordinate t,
/// replicating PatchGeometry: edge vertices with a coarser neighbor are
/// constrained to the lerp of the nearest coincident samples, and the mesh
/// interpolates linearly between vertices. edge is 'N','S','W','E'.
double renderedEdgeHeight(const QList<float>& heights, const QList<int>& deltas, QChar edge, double t)
{
    constexpr int kGrid = SurfaceModel::kGridSize;
    const int vpe = kGrid + 1;
    const auto raw = [&](int idx) {
        switch (edge.unicode()) {
            case u'N':
                return double(heights.at(idx));
            case u'S':
                return double(heights.at((kGrid * vpe) + idx));
            case u'W':
                return double(heights.at(idx * vpe));
            default:
                return double(heights.at((idx * vpe) + kGrid));
        }
    };
    const int deltaIndex = QStringLiteral("NSWE").indexOf(edge);
    const int delta = deltas.at(deltaIndex);
    const auto vertex = [&](int idx) {
        if (delta > 0) {
            const int step = 1 << delta;
            const int base = (idx / step) * step;
            if ((idx != base) && ((kGrid % step) == 0)) {
                const double a = raw(base);
                const double b = raw(base + step);
                return a + ((b - a) * (double(idx - base) / step));
            }
        }
        return raw(idx);
    };
    t = std::clamp(t, 0.0, double(kGrid));
    const int i0 = std::min(int(std::floor(t)), kGrid - 1);
    const double frac = t - i0;
    return (vertex(i0) * (1.0 - frac)) + (vertex(i0 + 1) * frac);
}

/// Largest rendered height mismatch along the shared boundary segment of two
/// edge-adjacent patches (0 when they don't share one). edgeOfA identifies
/// which edge of A faces B.
double sharedEdgeMaxStep(const SurfaceModel& model, const SurfaceModel::Patch& a, const SurfaceModel::Patch& b,
                         QChar edgeOfA)
{
    const QRectF rectA = tileRect(a.key);
    const QRectF rectB = tileRect(b.key);
    const bool horizontal = (edgeOfA == u'N') || (edgeOfA == u'S');  // shared segment runs east-west

    // Overlap of the two rects along the edge direction
    const double lo = horizontal ? std::max(rectA.left(), rectB.left()) : std::max(rectA.top(), rectB.top());
    const double hi = horizontal ? std::min(rectA.right(), rectB.right()) : std::min(rectA.bottom(), rectB.bottom());
    if (hi <= lo) {
        return 0.0;
    }

    const QList<int> deltasA = model.edgeLodDeltas(a.key);
    const QList<int> deltasB = model.edgeLodDeltas(b.key);
    const QChar edgeOfB = (edgeOfA == u'N') ? u'S' : (edgeOfA == u'S') ? u'N' : (edgeOfA == u'W') ? u'E' : u'W';

    constexpr int kSamples = 65;  // denser than any vertex spacing involved
    constexpr int kGrid = SurfaceModel::kGridSize;
    double maxStep = 0.0;
    for (int i = 0; i <= kSamples; i++) {
        const double pos = lo + ((hi - lo) * i / kSamples);
        // Fractional grid coordinate of pos on each patch's edge. Slippy y
        // grows south while world y grows north: row/col index t runs from
        // the NW corner, so along x t follows +x, along y t follows -y.
        const double tA = horizontal ? ((pos - rectA.left()) / rectA.width()) * kGrid
                                     : ((rectA.bottom() - pos) / rectA.height()) * kGrid;
        const double tB = horizontal ? ((pos - rectB.left()) / rectB.width()) * kGrid
                                     : ((rectB.bottom() - pos) / rectB.height()) * kGrid;
        const double hA = renderedEdgeHeight(a.heights, deltasA, edgeOfA, tA);
        const double hB = renderedEdgeHeight(b.heights, deltasB, edgeOfB, tB);
        maxStep = std::max(maxStep, std::abs(hA - hB));
    }
    return maxStep;
}

int maxZoomOf(const QList<SurfaceModel::Patch>& patches)
{
    int maxZoom = 0;
    for (const SurfaceModel::Patch& patch : patches) {
        maxZoom = std::max(maxZoom, patch.key.zoom);
    }
    return maxZoom;
}

/// Zoom of the finest resident patch containing the world point, or -1
int finestResidentZoomAt(const QList<SurfaceModel::Patch>& patches, const QPointF& world)
{
    int zoom = -1;
    for (const SurfaceModel::Patch& patch : patches) {
        if (tileRect(patch.key).contains(world)) {
            zoom = std::max(zoom, patch.key.zoom);
        }
    }
    return zoom;
}

/// Screen-grid coverage check independent of the model's own visibility
/// estimate: every pick-ray ground hit within the coverage contract range
/// must be inside a resident patch. Returns a failure description or empty.
QString coverageHole(const SurfaceModel& model, const GeoMapCamera& camera)
{
    constexpr int kCols = 13;
    constexpr int kRows = 9;
    // Slightly inside the full contract: the range cap itself is exact only
    // along sampled rays
    const double demandRange = SurfaceModel::kMaxRangeMultiplier * camera.distance() * 0.9;
    const QPointF cameraGround = camera.cameraGroundPosition();
    const QList<SurfaceModel::Patch> patches = model.patches();
    for (int row = 0; row < kRows; row++) {
        for (int col = 0; col < kCols; col++) {
            const QPointF screenPos((camera.viewportSize().width() * col) / (kCols - 1),
                                    (camera.viewportSize().height() * row) / (kRows - 1));
            const auto ground = camera.screenToGround(screenPos);
            if (!ground) {
                continue;  // sky
            }
            const double range = std::hypot(ground->x() - cameraGround.x(), ground->y() - cameraGround.y());
            if (range > demandRange) {
                continue;  // beyond the coverage contract
            }
            if (finestResidentZoomAt(patches, *ground) < 0) {
                return QStringLiteral("hole at screen (%1,%2) world (%3,%4) range %5 (tilt %6 dist %7 heading %8)")
                    .arg(screenPos.x())
                    .arg(screenPos.y())
                    .arg(ground->x())
                    .arg(ground->y())
                    .arg(range)
                    .arg(camera.tilt())
                    .arg(camera.distance())
                    .arg(camera.heading());
            }
        }
    }
    return {};
}

}  // namespace

void SurfaceModelTest::_patchRendersEstimateImmediately()
{
    GeoMapCamera camera;
    FlatHeightSource source;
    HeightField field;
    QVERIFY(field.insertTile(TileMath::TileKey{0, 0, 0}, constantGrid(100.0f)));
    SurfaceModel model(&camera, &source, &field);

    camera.setViewportSize(kViewport);
    camera.lookAt(kCenter, 0, 0, 2000);
    model.drainUpdates();

    // No event-loop wait: every patch must carry the field estimate the
    // moment it is added (never flat-zero-with-hole)
    QCOMPARE_GT(model.patchCount(), 0);
    const int expectedHeights = (SurfaceModel::kGridSize + 1) * (SurfaceModel::kGridSize + 1);
    for (const SurfaceModel::Patch& patch : model.patches()) {
        QVERIFY(patch.ready);
        QCOMPARE(patch.heights.count(), expectedHeights);
        QCOMPARE(patch.heights.first(), 100.0f);
    }
}

void SurfaceModelTest::_patchesAreViewsOfField()
{
    GeoMapCamera camera;
    FlatHeightSource source;
    HeightField field;
    ElevationTilePyramid::Grid grid;
    grid.width = 8;
    grid.height = 8;
    for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 8; col++) {
            grid.heights.append(float((row * 31) + (col * 7)));
        }
    }
    QVERIFY(field.insertTile(TileMath::TileKey{0, 0, 0}, std::move(grid)));
    SurfaceModel model(&camera, &source, &field);

    camera.setViewportSize(kViewport);
    camera.lookAt(kCenter, 0, 0, 2000);
    model.drainUpdates();

    // Patches are views: their heights are exactly the field's samples, so
    // any two patches asking about the same world position always agree
    QCOMPARE_GT(model.patchCount(), 0);
    for (const SurfaceModel::Patch& patch : model.patches()) {
        QCOMPARE(patch.heights, field.samplePatch(patch.key, SurfaceModel::kGridSize));
    }
}

void SurfaceModelTest::_reMeshOnDataArrival()
{
    GeoMapCamera camera;
    FlatHeightSource source;
    HeightField field;
    SurfaceModel model(&camera, &source, &field);

    camera.setViewportSize(kViewport);
    camera.lookAt(kCenter, 0, 0, 2000);
    model.drainUpdates();
    QCOMPARE_GT(model.patchCount(), 0);
    for (const SurfaceModel::Patch& patch : model.patches()) {
        QCOMPARE(patch.heights.first(), 0.0f);  // empty field: flat estimate
    }

    QSignalSpy meshSpy(&model, &SurfaceModel::patchMeshChanged);
    QVERIFY(field.insertTile(TileMath::TileKey{0, 0, 0}, constantGrid(100.0f)));

    // Data arrival re-meshes synchronously: the patches lift, no stale flats
    QCOMPARE_GT(meshSpy.count(), 0);
    for (const SurfaceModel::Patch& patch : model.patches()) {
        QCOMPARE(patch.heights.first(), 100.0f);
    }
}

void SurfaceModelTest::_reMeshScopeGate()
{
    GeoMapCamera camera;
    FlatHeightSource source;
    HeightField field;
    SurfaceModel model(&camera, &source, &field);

    camera.setViewportSize(kViewport);
    camera.lookAt(kCenter, 0, 45, 2000);
    model.drainUpdates();
    QCOMPARE_GT(model.patchCount(), 8);

    // A fine tile at the camera position changes a small region only
    const int fineZoom = maxZoomOf(model.patches());
    const TileMath::TileKey fineKey = TileMath::tileForWorld(TileMath::geoToWorld(kCenter), fineZoom);
    const QRectF region = tileRect(fineKey);

    QSignalSpy meshSpy(&model, &SurfaceModel::patchMeshChanged);
    QVERIFY(field.insertTile(fineKey, constantGrid(50.0f)));

    // Gate: exactly the patches touching the changed region re-mesh, once each
    QSet<TileMath::TileKey> remeshed;
    for (const QList<QVariant>& args : meshSpy) {
        remeshed.insert(args.first().value<TileMath::TileKey>());
    }
    int touching = 0;
    for (const SurfaceModel::Patch& patch : model.patches()) {
        if (touchesRegion(patch.key, region)) {
            QVERIFY2(remeshed.contains(patch.key), "patch touching the changed region was not re-meshed");
            touching++;
        } else {
            QVERIFY2(!remeshed.contains(patch.key), "patch outside the changed region was re-meshed");
        }
    }
    QCOMPARE_GT(touching, 0);
    QCOMPARE_LT(touching, model.patchCount());
    QCOMPARE(meshSpy.count(), touching);
}

void SurfaceModelTest::_requestsTileCoverage()
{
    GeoMapCamera camera;
    RecordingCoverageSource source;
    HeightField field;
    source.setHeightField(&field);
    SurfaceModel model(&camera, &source, &field);

    camera.setViewportSize(kViewport);
    camera.lookAt(kCenter, 0, 0, 2000);
    model.drainUpdates();

    QCOMPARE_GT(model.patchCount(), 0);
    for (const SurfaceModel::Patch& patch : model.patches()) {
        QVERIFY2(source.requested.contains(patch.key), "no tile coverage requested for resident patch");
    }
}

void SurfaceModelTest::_noViewportNoPatches()
{
    GeoMapCamera camera;
    FlatHeightSource source;
    HeightField field;
    SurfaceModel model(&camera, &source, &field);

    model.update();
    QCOMPARE(model.patchCount(), 0);
}

void SurfaceModelTest::_unpositionedCameraNoPatches()
{
    GeoMapCamera camera;
    FlatHeightSource source;
    HeightField field;
    SurfaceModel model(&camera, &source, &field);

    // A sized viewport alone must not build patches: the camera still holds
    // the null-island default pose (doomed terrain fetches would follow)
    camera.setViewportSize(kViewport);
    model.drainUpdates();
    QCOMPARE(model.patchCount(), 0);

    camera.setCenter(kCenter);
    model.drainUpdates();
    QCOMPARE_GT(model.patchCount(), 0);
}

void SurfaceModelTest::_coarseWhenFar()
{
    GeoMapCamera camera;
    FlatHeightSource source;
    HeightField field;
    SurfaceModel model(&camera, &source, &field);

    camera.setViewportSize(kViewport);
    camera.lookAt(kCenter, 0, 0, GeoMapCamera::kMaxDistance);
    model.drainUpdates();

    QCOMPARE_GT(model.patchCount(), 0);
    // From max distance the whole world fits the view: only low zoom levels
    QCOMPARE_LE(maxZoomOf(model.patches()), 4);
}

void SurfaceModelTest::_refinesWhenNear()
{
    GeoMapCamera camera;
    FlatHeightSource source;
    HeightField field;
    SurfaceModel model(&camera, &source, &field);

    camera.setViewportSize(kViewport);
    camera.lookAt(kCenter, 0, 0, GeoMapCamera::kMaxDistance);
    model.drainUpdates();
    const int farMaxZoom = maxZoomOf(model.patches());

    camera.lookAt(kCenter, 0, 0, 2000);
    model.drainUpdates();
    const int nearMaxZoom = maxZoomOf(model.patches());

    QCOMPARE_GT(nearMaxZoom, farMaxZoom);
    QCOMPARE_GT(model.patchCount(), 0);
}

void SurfaceModelTest::_patchCountBounded()
{
    GeoMapCamera camera;
    FlatHeightSource source;
    HeightField field;
    SurfaceModel model(&camera, &source, &field);

    camera.setViewportSize(kViewport);
    for (double distance : {50.0, 2000.0, 100000.0, GeoMapCamera::kMaxDistance}) {
        for (double tilt : {0.0, 45.0, GeoMapCamera::kMaxTilt}) {
            camera.lookAt(kCenter, 30, tilt, distance);
            model.drainUpdates();
            QCOMPARE_LE(model.patchCount(), SurfaceModel::kMaxPatches);
        }
    }
}

void SurfaceModelTest::_budgetExhaustedKeepsCoverage()
{
    GeoMapCamera camera;
    FlatHeightSource source;
    HeightField field;
    SurfaceModel model(&camera, &source, &field);

    // A huge viewport shrinks meters-per-pixel so refinement demand far
    // exceeds the patch budget
    camera.setViewportSize(QSizeF(8000, 6000));
    camera.lookAt(kCenter, 0, 45, 2000);
    model.drainUpdates();

    QCOMPARE_LE(model.patchCount(), SurfaceModel::kMaxPatches);
    // A split replaces one patch with up to four children, so an exhausted
    // budget lands within 3 of the cap. Anything lower means demand never
    // hit the cap and this test exercises nothing.
    QCOMPARE_GE(model.patchCount(), SurfaceModel::kMaxPatches - 3);

    // Coverage-preserving cap: every visible ground point must fall inside a
    // resident patch (holes were possible before the budget-aware traversal)
    const QList<SurfaceModel::Patch> patches = model.patches();
    const double maxRange = camera.distance() * SurfaceModel::kMaxRangeMultiplier;
    const double half = TileMath::worldSize() / 2.0;
    constexpr int sampleGrid = 9;
    for (int row = 0; row < sampleGrid; row++) {
        for (int col = 0; col < sampleGrid; col++) {
            const QPointF screenPos((camera.viewportSize().width() * col) / (sampleGrid - 1),
                                    (camera.viewportSize().height() * row) / (sampleGrid - 1));
            const auto ground = camera.groundPointCapped(screenPos, maxRange);
            if (!ground || (qAbs(ground->x()) > half) || (qAbs(ground->y()) > half)) {
                continue;  // horizon miss or capped outside the mercator world
            }
            bool covered = false;
            for (const SurfaceModel::Patch& patch : patches) {
                const double span = TileMath::tileSpanAtZoom(patch.key.zoom);
                const QPointF minCorner = TileMath::tileMinCorner(patch.key);
                if (QRectF(minCorner.x(), minCorner.y(), span, span).contains(*ground)) {
                    covered = true;
                    break;
                }
            }
            QVERIFY2(
                covered,
                qPrintable(
                    QStringLiteral("uncovered ground point at screen (%1, %2)").arg(screenPos.x()).arg(screenPos.y())));
        }
    }
}

void SurfaceModelTest::_diffOnMove()
{
    GeoMapCamera camera;
    FlatHeightSource source;
    HeightField field;
    SurfaceModel model(&camera, &source, &field);

    camera.setViewportSize(kViewport);
    camera.lookAt(kCenter, 0, 0, 2000);
    model.drainUpdates();

    QSignalSpy addedSpy(&model, &SurfaceModel::patchAdded);
    QSignalSpy removedSpy(&model, &SurfaceModel::patchRemoved);

    // Move to the other side of the planet: full set replacement
    camera.setCenter(QGeoCoordinate(-33.8688, 151.2093));
    model.drainUpdates();
    QCOMPARE_GT(addedSpy.count(), 0);
    QCOMPARE_GT(removedSpy.count(), 0);
}

void SurfaceModelTest::_noChurnOnIdenticalUpdate()
{
    GeoMapCamera camera;
    FlatHeightSource source;
    HeightField field;
    SurfaceModel model(&camera, &source, &field);

    camera.setViewportSize(kViewport);
    camera.lookAt(kCenter, 0, 0, 2000);
    model.drainUpdates();

    QSignalSpy addedSpy(&model, &SurfaceModel::patchAdded);
    QSignalSpy removedSpy(&model, &SurfaceModel::patchRemoved);
    model.update();
    QCOMPARE(addedSpy.count(), 0);
    QCOMPARE(removedSpy.count(), 0);
}

void SurfaceModelTest::_cullsInvisibleRegion()
{
    GeoMapCamera camera;
    FlatHeightSource source;
    HeightField field;
    SurfaceModel model(&camera, &source, &field);

    camera.setViewportSize(kViewport);
    camera.lookAt(kCenter, 0, 0, 2000);
    model.drainUpdates();

    // At 2km altitude over Zurich the antipodean hemisphere must not be resident
    const QPointF sydney = TileMath::geoToWorld(QGeoCoordinate(-33.8688, 151.2093));
    for (const SurfaceModel::Patch& patch : model.patches()) {
        const double span = TileMath::tileSpanAtZoom(patch.key.zoom);
        const QPointF minCorner = TileMath::tileMinCorner(patch.key);
        const QRectF rect(minCorner.x(), minCorner.y(), span, span);
        QVERIFY(!rect.contains(sydney) || (patch.key.zoom == 0));
    }
}

void SurfaceModelTest::_coverageMaintainedDuringLodChurn()
{
    GeoMapCamera camera;
    FlatHeightSource source;
    HeightField field;
    SurfaceModel model(&camera, &source, &field);

    camera.setViewportSize(kViewport);
    camera.lookAt(kCenter, 0, 0, GeoMapCamera::kMaxDistance);
    model.drainUpdates();
    QCOMPARE_GT(model.patchCount(), 0);

    // Refine toward the ground one capped pass at a time: after every single
    // pass the resident set must still cover the visible region — a coarse
    // patch may only go once its replacements are resident
    camera.lookAt(kCenter, 0, 0, 2000);
    const double maxRange = camera.distance() * SurfaceModel::kMaxRangeMultiplier;
    const double half = TileMath::worldSize() / 2.0;
    constexpr int sampleGrid = 5;
    int passes = 0;
    do {
        model.update();
        passes++;
        const QList<SurfaceModel::Patch> patches = model.patches();
        for (int row = 0; row < sampleGrid; row++) {
            for (int col = 0; col < sampleGrid; col++) {
                const QPointF screenPos((camera.viewportSize().width() * col) / (sampleGrid - 1),
                                        (camera.viewportSize().height() * row) / (sampleGrid - 1));
                const auto ground = camera.groundPointCapped(screenPos, maxRange);
                if (!ground || (qAbs(ground->x()) > half) || (qAbs(ground->y()) > half)) {
                    continue;
                }
                bool covered = false;
                for (const SurfaceModel::Patch& patch : patches) {
                    if (tileRect(patch.key).contains(*ground)) {
                        covered = true;
                        break;
                    }
                }
                QVERIFY2(covered, qPrintable(QStringLiteral("hole after pass %1 at screen (%2, %3)")
                                                 .arg(passes)
                                                 .arg(screenPos.x())
                                                 .arg(screenPos.y())));
            }
        }
    } while (!model.updateSettled() && (passes < 400));
    QVERIFY(model.updateSettled());
}

void SurfaceModelTest::_renderedEdgesContinuousAcrossLodBoundaries()
{
    // Field report: with real terrain relief, walls ("cliffs") appear along
    // patch boundaries at oblique horizon views. The rendered surface must be
    // C0 continuous across every shared patch edge: coincident samples of the
    // same field plus the PatchGeometry stitch constraint guarantee it for
    // every LOD delta the renderer supports.
    GeoMapCamera camera;
    RoughHeightSource source;
    HeightField field;
    source.setHeightField(&field);
    SurfaceModel model(&camera, &source, &field);
    camera.setViewportSize(kViewport);

    // Oblique poses with a far horizon maximize the LOD spread across the
    // resident set (the screenshot pose family)
    struct Pose
    {
        qreal tilt;
        qreal distance;
    };
    const QList<Pose> poses = {{80, 3000}, {GeoMapCamera::kMaxTilt, 15000}};

    for (const Pose& pose : poses) {
        camera.lookAt(kCenter, 0, pose.tilt, pose.distance);

        // Settle: drain model churn, deliver queued tile inserts, and repeat
        // until a full pass changes nothing (deliveries re-mesh and can
        // re-cull, which can request more tiles)
        QSignalSpy regionSpy(&field, &HeightField::regionChanged);
        int guard = 200;
        do {
            regionSpy.clear();
            model.drainUpdates();
            QCoreApplication::processEvents();
        } while (((regionSpy.count() > 0) || !model.updateSettled()) && (--guard > 0));
        QVERIFY(guard > 0);

        const QList<SurfaceModel::Patch> patches = model.patches();
        QCOMPARE_GT(patches.count(), 8);

        // Every edge-adjacent pair must render the same heights along the
        // shared boundary segment
        struct EdgeSpec
        {
            QChar edge;  // A's edge facing B
        };
        const QList<EdgeSpec> edges = {{u'N'}, {u'S'}, {u'W'}, {u'E'}};
        double worstStep = 0.0;
        QString worst;
        for (const SurfaceModel::Patch& a : patches) {
            const QRectF rectA = tileRect(a.key);
            for (const SurfaceModel::Patch& b : patches) {
                if (a.key == b.key) {
                    continue;
                }
                const QRectF rectB = tileRect(b.key);
                for (const EdgeSpec& spec : edges) {
                    // The world line A's edge lies on; B must sit across it
                    const double edgeLineA = (spec.edge == u'N')   ? rectA.bottom()  // north = max y
                                             : (spec.edge == u'S') ? rectA.top()
                                             : (spec.edge == u'W') ? rectA.left()
                                                                   : rectA.right();
                    const double edgeLineB = (spec.edge == u'N')   ? rectB.top()
                                             : (spec.edge == u'S') ? rectB.bottom()
                                             : (spec.edge == u'W') ? rectB.right()
                                                                   : rectB.left();
                    const double tolerance = std::min(rectA.width(), rectB.width()) * 1e-9;
                    if (std::abs(edgeLineA - edgeLineB) > tolerance) {
                        continue;
                    }
                    const double step = sharedEdgeMaxStep(model, a, b, spec.edge);
                    if (step > worstStep) {
                        worstStep = step;
                        worst = QStringLiteral("%1 m step: z%2 (%3,%4) %5 edge vs z%6 (%7,%8), tilt %9 dist %10")
                                    .arg(step, 0, 'f', 1)
                                    .arg(a.key.zoom)
                                    .arg(a.key.x)
                                    .arg(a.key.y)
                                    .arg(spec.edge)
                                    .arg(b.key.zoom)
                                    .arg(b.key.x)
                                    .arg(b.key.y)
                                    .arg(pose.tilt)
                                    .arg(pose.distance);
                        // TEMP DIAGNOSTIC
                        if (step > 0.5) {
                            qDebug() << "pair A" << a.key.zoom << a.key.x << a.key.y << "edge" << spec.edge << "B"
                                     << b.key.zoom << b.key.x << b.key.y;
                            const QList<int> dA = model.edgeLodDeltas(a.key);
                            const QList<int> dB = model.edgeLodDeltas(b.key);
                            qDebug() << "deltasA" << dA << "deltasB" << dB;
                            const QList<float> freshA = field.samplePatch(a.key, SurfaceModel::kGridSize);
                            const QList<float> freshB = field.samplePatch(b.key, SurfaceModel::kGridSize);
                            qDebug() << "cached==fresh A" << (a.heights == freshA) << "B" << (b.heights == freshB);
                            constexpr int kG = SurfaceModel::kGridSize;
                            const QChar eB = (spec.edge == u'N')   ? u'S'
                                             : (spec.edge == u'S') ? u'N'
                                             : (spec.edge == u'W') ? u'E'
                                                                   : u'W';
                            for (int idx = 0; idx <= kG; idx++) {
                                const double hA = renderedEdgeHeight(a.heights, dA, spec.edge, idx);
                                const double hB = renderedEdgeHeight(b.heights, dB, eB, idx);
                                qDebug() << "idx" << idx << "rawA" << renderedEdgeHeight(a.heights, {0, 0, 0, 0}, spec.edge, idx)
                                         << "rawB" << renderedEdgeHeight(b.heights, {0, 0, 0, 0}, eB, idx) << "rendA" << hA
                                         << "rendB" << hB;
                            }
                        }
                    }
                }
            }
        }
        QVERIFY2(worstStep < 0.5, qPrintable(worst));
    }
}

void SurfaceModelTest::_tallTerrainRecullsOnDataArrival()
{
    // Terrain (500 m) rises far above the camera eye (~229 up at tilt 55,
    // distance 400). Ground-plane culling alone would drop the ground under
    // and just behind the bottom screen edge, leaving a hole where that tall
    // terrain should render. When tall tile data arrives the model must
    // schedule a terrain-aware re-cull without any camera movement.
    GeoMapCamera camera;
    FlatHeightSource source;
    HeightField field;
    SurfaceModel model(&camera, &source, &field);

    camera.setViewportSize(kViewport);
    camera.lookAt(kCenter, 0, 55, 400);
    model.drainUpdates();
    QVERIFY(model.updateSettled());

    QVERIFY(field.insertTile(TileMath::TileKey{0, 0, 0}, constantGrid(500.0f)));
    QVERIFY2(!model.updateSettled(), "tall data arrival did not schedule a re-cull");

    QTRY_VERIFY_WITH_TIMEOUT(model.updateSettled(), 5000);
    const QPointF cameraGround = camera.cameraGroundPosition();
    QVERIFY(model.visibleGroundRect().contains(cameraGround));
    bool renderedUnderCamera = false;
    for (const SurfaceModel::Patch& patch : model.patches()) {
        if (tileRect(patch.key).contains(cameraGround)) {
            renderedUnderCamera = true;
            break;
        }
    }
    QVERIFY2(renderedUnderCamera, "no rendered patch spans the camera ground position");
}

void SurfaceModelTest::_cameraGroundTileResidentOverFlatTerrain()
{
    // The terrain ceiling comes only from resident patches, so tall terrain
    // living solely in never-requested tiles (e.g. a cliff under the camera
    // with flat water everywhere visible) could never be discovered. The
    // visible region must therefore include the camera ground point even
    // when every resident patch is flat.
    GeoMapCamera camera;
    camera.setViewportSize(kViewport);
    camera.lookAt(kCenter, 0, 55, 400);
    const QPointF cameraGround = camera.cameraGroundPosition();

    FlatHeightSource source;
    HeightField field;
    SurfaceModel model(&camera, &source, &field);
    model.drainUpdates();

    QVERIFY(model.visibleGroundRect().contains(cameraGround));

    bool renderedUnderCamera = false;
    for (const SurfaceModel::Patch& patch : model.patches()) {
        if (tileRect(patch.key).contains(cameraGround)) {
            renderedUnderCamera = true;
            break;
        }
    }
    QVERIFY2(renderedUnderCamera, "no rendered patch spans the camera ground position");
}

void SurfaceModelTest::_edgeLodDeltasMatchResidentNeighbors()
{
    GeoMapCamera camera;
    FlatHeightSource source;
    HeightField field;
    SurfaceModel model(&camera, &source, &field);

    // Tilted view: LOD rings guarantee coarser neighbors across ring boundaries
    camera.setViewportSize(kViewport);
    camera.lookAt(kCenter, 0, 45, 2000);
    model.drainUpdates();
    QCOMPARE_GT(model.patchCount(), 8);

    // Semantics: each edge's delta is how many levels coarser the finest
    // resident patch just across that edge renders (same/finer/absent = 0)
    const QList<SurfaceModel::Patch> patches = model.patches();
    int positiveDeltas = 0;
    for (const SurfaceModel::Patch& patch : patches) {
        const QRectF rect = tileRect(patch.key);
        const double eps = rect.width() * 0.01;
        // {N,S,W,E}: world y grows north, slippy y grows south
        const QList<QPointF> acrossPoints = {
            QPointF(rect.center().x(), rect.bottom() + eps),  // north (max y)
            QPointF(rect.center().x(), rect.top() - eps),     // south (min y)
            QPointF(rect.left() - eps, rect.center().y()),    // west
            QPointF(rect.right() + eps, rect.center().y()),   // east
        };
        const QList<int> deltas = model.edgeLodDeltas(patch.key);
        QCOMPARE(deltas.count(), 4);
        for (int edge = 0; edge < 4; edge++) {
            const int neighborZoom = finestResidentZoomAt(patches, acrossPoints[edge]);
            const int expected =
                ((neighborZoom >= 0) && (neighborZoom < patch.key.zoom)) ? (patch.key.zoom - neighborZoom) : 0;
            QVERIFY2(
                deltas[edge] == expected,
                qPrintable(QStringLiteral("edge %1: delta %2, expected %3").arg(edge).arg(deltas[edge]).arg(expected)));
            if (deltas[edge] > 0) {
                positiveDeltas++;
            }
        }
    }
    QCOMPARE_GT(positiveDeltas, 0);  // the view must actually exercise stitching
}

void SurfaceModelTest::_edgeDeltasNotifiedOnNeighborChurn()
{
    GeoMapCamera camera;
    FlatHeightSource source;
    HeightField field;
    SurfaceModel model(&camera, &source, &field);

    camera.setViewportSize(kViewport);
    camera.lookAt(kCenter, 0, 0, GeoMapCamera::kMaxDistance);
    model.drainUpdates();

    // Refinement churn adds/removes neighbors: resident patches whose edge
    // deltas may have changed must be notified so consumers re-pull them
    QSignalSpy deltasSpy(&model, &SurfaceModel::patchEdgeDeltasChanged);
    camera.lookAt(kCenter, 0, 45, 2000);
    model.drainUpdates();

    QCOMPARE_GT(deltasSpy.count(), 0);
    for (const QList<QVariant>& args : deltasSpy) {
        QVERIFY(TileMath::isValidKey(args.first().value<TileMath::TileKey>()));
    }
}

void SurfaceModelTest::_coverageSweepAcrossPoses()
{
    GeoMapCamera camera;
    FlatHeightSource source;
    HeightField field;
    // Real terrain heights change the cull: sweep with data resident too
    QVERIFY(field.insertTile(TileMath::TileKey{0, 0, 0}, constantGrid(400.0f)));
    SurfaceModel model(&camera, &source, &field);
    camera.setViewportSize(kViewport);

    // Sequential poses on purpose: each settle starts from the previous
    // pose's resident set, like interactive use. Oblique tilts stress the
    // AABB visibility estimate the most.
    const QList<qreal> tilts = {0, 30, 55, 70, 80, GeoMapCamera::kMaxTilt};
    const QList<qreal> distances = {200, 2000, 50000, 2000000};
    const QList<qreal> headings = {0, 135};
    for (const qreal heading : headings) {
        for (const qreal distance : distances) {
            for (const qreal tilt : tilts) {
                camera.lookAt(kCenter, heading, tilt, distance);
                model.drainUpdates();
                const QString hole = coverageHole(model, camera);
                QVERIFY2(hole.isEmpty(), qPrintable(hole));
            }
        }
    }
}

void SurfaceModelTest::_coverageAfterInteractiveGesture()
{
    GeoMapCamera camera;
    FlatHeightSource source;
    HeightField field;
    QVERIFY(field.insertTile(TileMath::TileKey{0, 0, 0}, constantGrid(400.0f)));
    SurfaceModel model(&camera, &source, &field);
    camera.setViewportSize(kViewport);

    struct Pose
    {
        QGeoCoordinate center;
        qreal heading;
        qreal tilt;
        qreal distance;
    };

    const QGeoCoordinate kPannedCenter(kCenter.latitude() + 0.05, kCenter.longitude() + 0.05);
    // Drag-like path: tilt down to max oblique, zoom out, rotate, pan, zoom
    // back in. Interpolated per-frame with a single update pass per frame so
    // the camera outruns the add/removal caps, exactly like interactive use.
    const QList<Pose> waypoints = {
        {kCenter, 0, 0, 2000},
        {kCenter, 0, GeoMapCamera::kMaxTilt, 2000},
        {kCenter, 0, GeoMapCamera::kMaxTilt, 200000},
        {kCenter, 180, GeoMapCamera::kMaxTilt, 200000},
        {kPannedCenter, 180, GeoMapCamera::kMaxTilt, 200000},
        {kPannedCenter, 180, 70, 500},
    };
    constexpr int kStepsPerSegment = 30;
    for (int seg = 1; seg < waypoints.count(); seg++) {
        const Pose& from = waypoints[seg - 1];
        const Pose& to = waypoints[seg];
        for (int step = 1; step <= kStepsPerSegment; step++) {
            const qreal t = qreal(step) / kStepsPerSegment;
            const QGeoCoordinate center(
                from.center.latitude() + ((to.center.latitude() - from.center.latitude()) * t),
                from.center.longitude() + ((to.center.longitude() - from.center.longitude()) * t));
            camera.lookAt(center, from.heading + ((to.heading - from.heading) * t),
                          from.tilt + ((to.tilt - from.tilt) * t),
                          from.distance * std::pow(to.distance / from.distance, t));
            model.update();  // one pass per frame: camera moves faster than the model settles
        }
        // Camera stops: the model must clean up to full coverage
        model.drainUpdates();
        QVERIFY(model.updateSettled());
        const QString hole = coverageHole(model, camera);
        QVERIFY2(hole.isEmpty(), qPrintable(QStringLiteral("segment %1: %2").arg(seg).arg(hole)));
    }
}

void SurfaceModelTest::_pinsPatchBackingTiles()
{
    // Tiles that back resident patches (their keys and resolving ancestors)
    // must be pinned against LRU eviction: an evicted backing tile silently
    // coarsens a rendered mesh next to an intact neighbor — a cliff
    GeoMapCamera camera;
    FlatHeightSource source;
    HeightField field;
    SurfaceModel model(&camera, &source, &field);
    camera.setViewportSize(kViewport);
    camera.lookAt(kCenter, 0, 0, 2000);
    model.drainUpdates();
    QCOMPARE_GT(model.patchCount(), 0);

    // Give one resident patch backing data at its own key, then flood with
    // unrelated tiles far past the pyramid cap
    const TileMath::TileKey backingKey = model.patches().first().key;
    QVERIFY(field.insertTile(backingKey, constantGrid(123.0f)));
    for (int i = 0; i < ElevationTilePyramid::kMaxTiles + 8; i++) {
        QVERIFY(field.insertTile(TileMath::TileKey{i, 200, 9}, constantGrid(1.0f)));
    }

    QVERIFY2(field.hasTile(backingKey), "patch-backing tile was evicted");
}

UT_REGISTER_TEST_LIGHTWEIGHT(SurfaceModelTest, TestLabel::Unit)
