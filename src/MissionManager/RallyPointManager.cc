/****************************************************************************
 *
 * (c) 2009-2020 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "RallyPointManager.h"
#include "ParameterManager.h"
#include "Vehicle.h"
#include "QGCApplication.h"

QGC_LOGGING_CATEGORY(RallyPointManagerLog, "RallyPointManagerLog")

RallyPointManager::RallyPointManager(Vehicle* vehicle, QObject* parent)
    : PlanManager(vehicle, MAV_MISSION_TYPE_RALLY, parent)
{
    connect(this, &PlanManager::_removeAllComplete, this, &RallyPointManager::_managerRemoveAllComplete);
    connect(this, &PlanManager::_sendComplete,      this, &RallyPointManager::_managerSendComplete);
    connect(this, &PlanManager::_loadComplete,      this, &RallyPointManager::_managerSendComplete);
}


RallyPointManager::~RallyPointManager()
{

}

void RallyPointManager::sendToVehicle(const QList<QGeoCoordinate>& rgPoints)
{
    _rgSendPoints.clear();
    for (const QGeoCoordinate& rallyPoint: rgPoints) {
        _rgSendPoints.append(rallyPoint);
    }

    QList<MissionItem*> rallyItems;
    for (int i=0; i<rgPoints.count(); i++) {

        MissionItem* item = new MissionItem(0,
                                            MAV_CMD_NAV_RALLY_POINT,
                                            MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                            0, 0, 0, 0,                 // param 1-4 unused
                                            rgPoints[i].latitude(),
                                            rgPoints[i].longitude(),
                                            rgPoints[i].altitude(),
                                            false,                      // autocontinue
                                            false,                      // isCurrentItem
                                            this);                      // parent
        rallyItems.append(item);
    }

    // Plan manager takes control of MissionItems, so no need to delete
    writeMissionItems(rallyItems);
}

bool RallyPointManager::supported(void) const
{
    return (_vehicle->capabilityBits() & MAV_PROTOCOL_CAPABILITY_MISSION_RALLY) && (_vehicle->maxProtoVersion() >= 200);
}

void RallyPointManager::_managerLoadComplete(bool error)
{
    _rgPoints.clear();

    if (!error) {
        const QList<MissionItem*>& rallyItems = missionItems();

        for (int i=0; i<rallyItems.count(); i++) {
            MissionItem* item = rallyItems[i];

            MAV_CMD command = item->command();
            if (command == MAV_CMD_NAV_RALLY_POINT) {
                _rgPoints.append(QGeoCoordinate(item->param5(), item->param6(), item->param7()));
            } else {
                qCDebug(RallyPointManagerLog) << "RallyPointManager load: Unsupported command %1" << command;
                break;
            }
        }
    }

    emit loadComplete(error);
}

void RallyPointManager::_managerSendComplete(bool error)
{
    if (error) {
        _rgPoints.clear();
    } else {
        _rgPoints = _rgSendPoints;
    }
    _rgSendPoints.clear();
    emit sendComplete(error);
}

void RallyPointManager::_managerRemoveAllComplete(bool error)
{
    if (!error) {
        _rgPoints.clear();
    }
    emit removeAllComplete(error);
}
