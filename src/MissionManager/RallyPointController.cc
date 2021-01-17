/****************************************************************************
 *
 * (c) 2009-2020 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "RallyPointController.h"
#include "RallyPoint.h"
#include "Vehicle.h"
#include "FirmwarePlugin.h"
#include "MAVLinkProtocol.h"
#include "QGCApplication.h"
#include "ParameterManager.h"
#include "JsonHelper.h"
#include "SimpleMissionItem.h"
#include "SettingsManager.h"
#include "AppSettings.h"
#include "PlanMasterController.h"

#include <QJsonDocument>
#include <QJsonArray>

QGC_LOGGING_CATEGORY(RallyPointControllerLog, "RallyPointControllerLog")

const char* RallyPointController::_jsonFileTypeValue =  "RallyPoints";
const char* RallyPointController::_jsonPointsKey =      "points";

RallyPointController::RallyPointController(RallyPointManager* rallyPointManager, QObject* parent)
    : PlanElementController (rallyPointManager->vehicle(), parent)
    , _rallyPointManager    (rallyPointManager)
{
    connect(&_points, &QmlObjectListModel::countChanged, this, &RallyPointController::_updateContainsItems);

    connect(_rallyPointManager, &RallyPointManager::sendComplete,       this, &RallyPointController::sendComplete);
    connect(_rallyPointManager, &RallyPointManager::loadComplete,       this, &RallyPointController::_managerLoadComplete);
    connect(_rallyPointManager, &RallyPointManager::inProgressChanged,  this, &RallyPointController::syncInProgressChanged);

    //-- RallyPointController::supported() tests both the capability bit AND the protocol version.
    connect(_vehicle, &Vehicle::capabilityBitsChanged,  this, &RallyPointController::supportedChanged);
    connect(_vehicle, &Vehicle::requestProtocolVersion, this, &RallyPointController::supportedChanged);
}

RallyPointController::~RallyPointController()
{

}

bool RallyPointController::loadFromJson(const QJsonObject& json, QString& errorString)
{
    removeAll();

    errorString.clear();

    if (json.contains(JsonHelper::jsonVersionKey) && json[JsonHelper::jsonVersionKey].toInt() == 1) {
        // We just ignore old version 1 data
        return true;
    }

    QList<JsonHelper::KeyValidateInfo> keyInfoList = {
        { JsonHelper::jsonVersionKey,   QJsonValue::Double, true },
        { _jsonPointsKey,               QJsonValue::Array,  true },
    };
    if (!JsonHelper::validateKeys(json, keyInfoList, errorString)) {
        return false;
    }

    QString errorStr;
    QString errorMessage = tr("Rally: %1");

    if (json[JsonHelper::jsonVersionKey].toInt() != _jsonCurrentVersion) {
        errorString = tr("Rally Points supports version %1").arg(_jsonCurrentVersion);
        return false;
    }

    QList<QGeoCoordinate> rgPoints;
    if (!JsonHelper::loadGeoCoordinateArray(json[_jsonPointsKey], true /* altitudeRequired */, rgPoints, errorStr)) {
        errorString = errorMessage.arg(errorStr);
        return false;
    }

    QObjectList pointList;
    for (int i=0; i<rgPoints.count(); i++) {
        pointList.append(new RallyPoint(rgPoints[i], this));
    }
    _points.swapObjectList(pointList);

    setDirty(false);
    _setFirstPointCurrent();

    return true;
}

void RallyPointController::saveToJson(QJsonObject& json)
{
    json[JsonHelper::jsonVersionKey] = _jsonCurrentVersion;

    QJsonArray rgPoints;
    QJsonValue jsonPoint;
    for (int i=0; i<_points.count(); i++) {
        JsonHelper::saveGeoCoordinate(qobject_cast<RallyPoint*>(_points[i])->coordinate(), true /* writeAltitude */, jsonPoint);
        rgPoints.append(jsonPoint);
    }
    json[_jsonPointsKey] = QJsonValue(rgPoints);
}

void RallyPointController::removeAll(void)
{
    _points.clearAndDeleteContents();
    setDirty(true);
    setCurrentRallyPoint(nullptr);
}

void RallyPointController::removeAllFromVehicle(void)
{
    if (_vehicle->isOfflineEditingVehicle()) {
        qCWarning(RallyPointControllerLog) << "RallyPointController::removeAllFromVehicle called while offline";
    } else if (syncInProgress()) {
        qCWarning(RallyPointControllerLog) << "RallyPointController::removeAllFromVehicle called while syncInProgress";
    } else {
        _rallyPointManager->removeAll();
    }
}

void RallyPointController::loadFromVehicle(void)
{
    if (_vehicle->isOfflineEditingVehicle()) {
        qCWarning(RallyPointControllerLog) << "RallyPointController::loadFromVehicle called while offline";
    } else if (syncInProgress()) {
        qCWarning(RallyPointControllerLog) << "RallyPointController::loadFromVehicle called while syncInProgress";
    } else {
        _rallyPointManager->loadFromVehicle();
    }
}

void RallyPointController::sendToVehicle(void)
{
    if (_vehicle->isOfflineEditingVehicle()) {
        qCWarning(RallyPointControllerLog) << "sendToVehicle Should not be called for offline vehicle";
        emit sendComplete(true);
        return;
    }

    if (!supported()) {
        qCDebug(RallyPointControllerLog) << "sendToVehicle Not supported";
        emit sendComplete(false);
        return;
    }

    qCDebug(RallyPointControllerLog) << "RallyPointController::sendToVehicle";

    setDirty(false);
    QList<QGeoCoordinate> rgPoints;
    for (int i=0; i<_points.count(); i++) {
        rgPoints.append(qobject_cast<RallyPoint*>(_points[i])->coordinate());
    }
    _rallyPointManager->sendToVehicle(rgPoints);
}

bool RallyPointController::syncInProgress(void) const
{
    return _rallyPointManager->inProgress();
}

void RallyPointController::setDirty(bool dirty)
{
    if (dirty != _dirty) {
        _dirty = dirty;
        emit dirtyChanged(dirty);
    }
}

QString RallyPointController::editorQml(void) const
{
    return _rallyPointManager->editorQml();
}

void RallyPointController::_managerLoadComplete(bool error)
{
    _points.clearAndDeleteContents();

    if (!error) {
        _points.beginReset();
        for (int i=0; i<_rallyPointManager->points().count(); i++) {
            _points.append(new RallyPoint(_rallyPointManager->points()[i], this));
        }
        _points.endReset();
    }

    setDirty(false);
    _setFirstPointCurrent();

    emit loadComplete(error);
}

void RallyPointController::addPoint(QGeoCoordinate point)
{
    double defaultAlt;
    if (_points.count()) {
        defaultAlt = qobject_cast<RallyPoint*>(_points[_points.count() - 1])->coordinate().altitude();
    } else {
        if(_vehicle->fixedWing()) {
            defaultAlt = qgcApp()->toolbox()->settingsManager()->appSettings()->defaultMissionItemAltitude()->rawValue().toDouble();
        }
        else {
            defaultAlt = RallyPoint::getDefaultFactAltitude();
        }
    }
    point.setAltitude(defaultAlt);
    RallyPoint* newPoint = new RallyPoint(point, this);
    _points.append(newPoint);
    setCurrentRallyPoint(newPoint);
    setDirty(true);
}

bool RallyPointController::supported(void) const
{
    return (_vehicle->capabilityBits() & MAV_PROTOCOL_CAPABILITY_MISSION_RALLY) && (_vehicle->maxProtoVersion() >= 200);
}

void RallyPointController::removePoint(QObject* rallyPoint)
{
    int foundIndex = 0;
    for (foundIndex=0; foundIndex<_points.count(); foundIndex++) {
        if (_points[foundIndex] == rallyPoint) {
            _points.removeOne(rallyPoint);
            rallyPoint->deleteLater();
        }
    }

    if (_points.count()) {
        int newIndex = qMin(foundIndex, _points.count() - 1);
        newIndex = qMax(newIndex, 0);
        setCurrentRallyPoint(_points[newIndex]);
    } else {
        setCurrentRallyPoint(nullptr);
    }
}

void RallyPointController::setCurrentRallyPoint(QObject* rallyPoint)
{
    if (_currentRallyPoint != rallyPoint) {
        _currentRallyPoint = rallyPoint;
        emit currentRallyPointChanged(rallyPoint);
    }
}

void RallyPointController::_setFirstPointCurrent(void)
{
    setCurrentRallyPoint(_points.count() ? _points[0] : nullptr);
}

bool RallyPointController::containsItems(void) const
{
    return _points.count() > 0;
}

void RallyPointController::_updateContainsItems(void)
{
    emit containsItemsChanged(containsItems());
}

bool RallyPointController::isEmpty(void) const
{
    return _points.count() == 0;
}

QList<QGeoCoordinate> RallyPointController::getRallyPoints(void)
{
    QList<QGeoCoordinate> rgPoints;

    for (int i=0; i<_points.count(); i++) {
        rgPoints.append(qobject_cast<RallyPoint*>(_points[i])->coordinate());
    }

    return rgPoints;
}
