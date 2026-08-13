#pragma once

#include "UnitTest.h"

class HeightFieldTest : public UnitTest
{
    Q_OBJECT

private slots:
    void _zeroWhenEmpty();
    void _invalidRequests();
    void _exactValuesAtSampleCenters();
    void _bilinearBetweenCenters();
    void _clampAtTileEdges();
    void _finestTileWins();
    void _pinnedTilesSurviveInsertPressure();
    void _evictionEmitsRegionChanged();
    void _sharedEdgeIdentity();
    void _crossZoomVertexIdentity();
    void _regionChangedOnInsert();
    void _noRegionChangedOnRejectedInsert();
    void _samplePatchLookupCountGate();
    void _samplePatchLookupCountGateMixedZooms();
    void _samplePatchSingleLookupPerPatch();
    void _memoInvalidatedOnInsert();
    void _samplePatchBenchmark();
};
