/****************************************************************************
 *
 * (c) 2009-2020 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "GeoFenceManager.h"
#include "Vehicle.h"
#include "QmlObjectListModel.h"
#include "ParameterManager.h"
#include "QGCApplication.h"
#include "QGCMapPolygon.h"
#include "QGCMapCircle.h"

QGC_LOGGING_CATEGORY(GeoFenceManagerLog, "GeoFenceManagerLog")

GeoFenceManager::GeoFenceManager(Vehicle* vehicle, QObject* parent)
    : PlanManager       (vehicle, MAV_MISSION_TYPE_FENCE, parent)
#if defined(QGC_AIRMAP_ENABLED)
    , _airspaceManager  (qgcApp()->toolbox()->airspaceManager())
#endif
{
    connect(this, &PlanManager::_removeAllComplete, this, &GeoFenceManager::_managerRemoveAllComplete);
    connect(this, &PlanManager::_sendComplete,      this, &GeoFenceManager::_managerSendComplete);
    connect(this, &PlanManager::_loadComplete,      this, &GeoFenceManager::_managerLoadComplete);
}

GeoFenceManager::~GeoFenceManager()
{

}

void GeoFenceManager::sendToVehicle(const QGeoCoordinate&   breachReturn,
                                    QmlObjectListModel&     polygons,
                                    QmlObjectListModel&     circles)
{
    QList<MissionItem*> fenceItems;

    _sendPolygons.clear();
    _sendCircles.clear();

    for (int i=0; i<polygons.count(); i++) {
        _sendPolygons.append(*polygons.value<QGCFencePolygon*>(i));
    }
    for (int i=0; i<circles.count(); i++) {
        _sendCircles.append(*circles.value<QGCFenceCircle*>(i));
    }
    _breachReturnPoint = breachReturn;

    for (int i=0; i<_sendPolygons.count(); i++) {
        const QGCFencePolygon& polygon = _sendPolygons[i];

        for (int j=0; j<polygon.count(); j++) {
            const QGeoCoordinate& vertex = polygon.path()[j].value<QGeoCoordinate>();

            MissionItem* item = new MissionItem(0,
                                                polygon.inclusion() ? MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION : MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
                                                MAV_FRAME_GLOBAL,
                                                polygon.count(),    // vertex count
                                                0, 0, 0,            // param 2-4 unused
                                                vertex.latitude(),
                                                vertex.longitude(),
                                                0,                  // param 7 unused
                                                false,              // autocontinue
                                                false,              // isCurrentItem
                                                this);              // parent
            fenceItems.append(item);
        }
    }

    for (int i=0; i<_sendCircles.count(); i++) {
        QGCFenceCircle& circle = _sendCircles[i];

        MissionItem* item = new MissionItem(0,
                                            circle.inclusion() ? MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION : MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
                                            MAV_FRAME_GLOBAL,
                                            circle.radius()->rawValue().toDouble(),
                                            0, 0, 0,                    // param 2-4 unused
                                            circle.center().latitude(),
                                            circle.center().longitude(),
                                            0,                          // param 7 unused
                                            false,                      // autocontinue
                                            false,                      // isCurrentItem
                                            this);                      // parent
        fenceItems.append(item);
    }

    if (_breachReturnPoint.isValid()) {
        MissionItem* item = new MissionItem(0,
                                            MAV_CMD_NAV_FENCE_RETURN_POINT,
                                            MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                            0, 0, 0, 0,                    // param 1-4 unused
                                            breachReturn.latitude(),
                                            breachReturn.longitude(),
                                            breachReturn.altitude(),
                                            false,                      // autocontinue
                                            false,                      // isCurrentItem
                                            this);                      // parent
        fenceItems.append(item);
    }

    // Plan manager takes control of MissionItems, so no need to delete
    writeMissionItems(fenceItems);
}

void GeoFenceManager::_managerSendComplete(bool error)
{
    if (error) {
        _clear();
    } else {
        _polygons = _sendPolygons;
        _circles = _sendCircles;
    }
    _sendPolygons.clear();
    _sendCircles.clear();
    emit sendComplete(error);
}

void GeoFenceManager::_managerLoadComplete(bool error)
{
    bool loadFailed = false;

    _polygons.clear();
    _circles.clear();

    MAV_CMD expectedCommand = (MAV_CMD)0;
    int expectedVertexCount = 0;
    QGCFencePolygon nextPolygon(true /* inclusion */);
    const QList<MissionItem*>& fenceItems = missionItems();

    for (int i=0; i<fenceItems.count(); i++) {
        MissionItem* item = fenceItems[i];

        MAV_CMD command = item->command();

        if (command == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION || command == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION) {
            if (nextPolygon.count() == 0) {
                // Starting a new polygon
                expectedVertexCount = item->param1();
                expectedCommand = command;
            } else if (expectedVertexCount != item->param1()){
                // In the middle of a polygon, but count suddenly changed
                _displayError(BadPolygonItemFormat, tr("GeoFence load: Vertex count change mid-polygon - actual:expected").arg(item->param1()).arg(expectedVertexCount));
                loadFailed = true;
                break;
            } if (expectedCommand != command) {
                // Command changed before last polygon was completely loaded
                _displayError(BadPolygonItemFormat, tr("GeoFence load: Polygon type changed before last load complete - actual:expected").arg(command).arg(expectedCommand));
                loadFailed = true;
                break;
            }
            nextPolygon.appendVertex(QGeoCoordinate(item->param5(), item->param6()));
            if (nextPolygon.count() == expectedVertexCount) {
                // Polygon is complete
                nextPolygon.setInclusion(command == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION);
                _polygons.append(nextPolygon);
                nextPolygon.clear();
            }
        } else if (command == MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION || command == MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION) {
            if (nextPolygon.count() != 0) {
                // Incomplete polygon
                _displayError(IncompletePolygonLoad, tr("GeoFence load: Incomplete polygon loaded"));
                loadFailed = true;
                break;
            }
            QGCFenceCircle circle(QGeoCoordinate(item->param5(), item->param6()), item->param1(), command == MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION /* inclusion */);
            _circles.append(circle);
        } else if (command == MAV_CMD_NAV_FENCE_RETURN_POINT) {
            _breachReturnPoint = QGeoCoordinate(item->param5(), item->param6(), item->param7());
        } else {
            _displayError(UnsupportedCommand, tr("GeoFence load: Unsupported command %1").arg(item->command()));
            loadFailed = true;
            break;
        }
    }

    if (loadFailed) {
        _clear();
    }

    emit loadComplete(error || loadFailed);
}

void GeoFenceManager::_managerRemoveAllComplete(bool error)
{
    if (!error) {
        _clear();
    }
    emit removeAllComplete(error);
}

bool GeoFenceManager::supported(void) const
{
    return (_vehicle->capabilityBits() & MAV_PROTOCOL_CAPABILITY_MISSION_FENCE) && (_vehicle->maxProtoVersion() >= 200);
}

void GeoFenceManager::_displayError(ErrorCode_t errorCode, const QString& errorString)
{
    qCWarning(GeoFenceManagerLog) << QStringLiteral("_displayError errorCode: %1 errorString: %2").arg(errorCode).arg(errorString);
    qgcApp()->showAppMessage(tr("%1 transfer failed. Error: %2").arg(_planTypeToString()).arg(errorString));
}

void GeoFenceManager::_clear(void)
{
    _polygons.clear();
    _circles.clear();
    _breachReturnPoint = QGeoCoordinate();
}
