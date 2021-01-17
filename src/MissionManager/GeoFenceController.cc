/****************************************************************************
 *
 * (c) 2009-2020 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "GeoFenceController.h"
#include "Vehicle.h"
#include "FirmwarePlugin.h"
#include "MAVLinkProtocol.h"
#include "QGCApplication.h"
#include "ParameterManager.h"
#include "JsonHelper.h"
#include "QGCQGeoCoordinate.h"
#include "AppSettings.h"
#include "PlanMasterController.h"
#include "SettingsManager.h"
#include "AppSettings.h"

#include <QJsonDocument>
#include <QJsonArray>

QGC_LOGGING_CATEGORY(GeoFenceControllerLog, "GeoFenceControllerLog")

QMap<QString, FactMetaData*> GeoFenceController::_metaDataMap;

const char* GeoFenceController::_jsonFileTypeValue =        "GeoFence";
const char* GeoFenceController::_jsonBreachReturnKey =      "breachReturn";
const char* GeoFenceController::_jsonPolygonsKey =          "polygons";
const char* GeoFenceController::_jsonCirclesKey =           "circles";

const char* GeoFenceController::_breachReturnAltitudeFactName = "Altitude";

const char* GeoFenceController::_px4ParamCircularFence =    "GF_MAX_HOR_DIST";

GeoFenceController::GeoFenceController(GeoFenceManager* geoFenceManager, QObject* parent)
    : PlanElementController         (geoFenceManager->vehicle(), parent)
    , _geoFenceManager              (geoFenceManager)
    , _breachReturnAltitudeFact     (0, _breachReturnAltitudeFactName, FactMetaData::valueTypeDouble)
    , _breachReturnDefaultAltitude  (qgcApp()->toolbox()->settingsManager()->appSettings()->defaultMissionItemAltitude()->rawValue().toDouble())
{
    if (_metaDataMap.isEmpty()) {
        _metaDataMap = FactMetaData::createMapFromJsonFile(QStringLiteral(":/json/BreachReturn.FactMetaData.json"), nullptr /* metaDataParent */);
    }

    _breachReturnAltitudeFact.setMetaData(_metaDataMap[_breachReturnAltitudeFactName]);
    _breachReturnAltitudeFact.setRawValue(_breachReturnDefaultAltitude);

    connect(&_polygons, &QmlObjectListModel::countChanged, this, &GeoFenceController::_updateContainsItems);
    connect(&_circles,  &QmlObjectListModel::countChanged, this, &GeoFenceController::_updateContainsItems);

    connect(this,                       &GeoFenceController::breachReturnPointChanged,  this, &GeoFenceController::_setDirty);
    connect(&_breachReturnAltitudeFact, &Fact::rawValueChanged,                         this, &GeoFenceController::_setDirty);
    connect(&_polygons,                 &QmlObjectListModel::dirtyChanged,              this, &GeoFenceController::_setDirty);
    connect(&_circles,                  &QmlObjectListModel::dirtyChanged,              this, &GeoFenceController::_setDirty);

    connect(_geoFenceManager, &GeoFenceManager::sendComplete,      this, &GeoFenceController::sendComplete);
    connect(_geoFenceManager, &GeoFenceManager::loadComplete,      this, &GeoFenceController::_managerLoadComplete);

    connect(_geoFenceManager, &GeoFenceManager::inProgressChanged, this, &GeoFenceController::syncInProgressChanged);

    //-- GeoFenceController::supported() tests both the capability bit AND the protocol version.
    connect(_vehicle, &Vehicle::capabilityBitsChanged,  this, &GeoFenceController::supportedChanged);
    connect(_vehicle, &Vehicle::requestProtocolVersion, this, &GeoFenceController::supportedChanged);

    connect(_vehicle->parameterManager(), &ParameterManager::parametersReadyChanged, this, &GeoFenceController::_parametersReady);
    _parametersReady();
}

GeoFenceController::~GeoFenceController()
{

}

void GeoFenceController::setBreachReturnPoint(const QGeoCoordinate& breachReturnPoint)
{
    if (_breachReturnPoint != breachReturnPoint) {
        _breachReturnPoint = breachReturnPoint;
        setDirty(true);
        emit breachReturnPointChanged(breachReturnPoint);
    }
}

bool GeoFenceController::loadFromJson(const QJsonObject& json, QString& errorString)
{
    removeAll();

    errorString.clear();

    if (!json.contains(JsonHelper::jsonVersionKey) ||
            (json.contains(JsonHelper::jsonVersionKey) && json[JsonHelper::jsonVersionKey].toInt() == 1)) {
        // We just ignore old version 1 or prior data
        return true;
    }

    QList<JsonHelper::KeyValidateInfo> keyInfoList = {
        { JsonHelper::jsonVersionKey,   QJsonValue::Double, true },
        { _jsonCirclesKey,              QJsonValue::Array,  true },
        { _jsonPolygonsKey,             QJsonValue::Array,  true },
        { _jsonBreachReturnKey,         QJsonValue::Array,  false },
    };
    if (!JsonHelper::validateKeys(json, keyInfoList, errorString)) {
        return false;
    }

    if (json[JsonHelper::jsonVersionKey].toInt() != _jsonCurrentVersion) {
        errorString = tr("GeoFence supports version %1").arg(_jsonCurrentVersion);
        return false;
    }

    QJsonArray jsonPolygonArray = json[_jsonPolygonsKey].toArray();
    for (const QJsonValue jsonPolygonValue: jsonPolygonArray) {
        if (jsonPolygonValue.type() != QJsonValue::Object) {
            errorString = tr("GeoFence polygon not stored as object");
            return false;
        }

        QGCFencePolygon* fencePolygon = new QGCFencePolygon(false /* inclusion */, this /* parent */);
        if (!fencePolygon->loadFromJson(jsonPolygonValue.toObject(), true /* required */, errorString)) {
            return false;
        }
        _polygons.append(fencePolygon);
    }

    QJsonArray jsonCircleArray = json[_jsonCirclesKey].toArray();
    for (const QJsonValue jsonCircleValue: jsonCircleArray) {
        if (jsonCircleValue.type() != QJsonValue::Object) {
            errorString = tr("GeoFence circle not stored as object");
            return false;
        }

        QGCFenceCircle* fenceCircle = new QGCFenceCircle(this /* parent */);
        if (!fenceCircle->loadFromJson(jsonCircleValue.toObject(), errorString)) {
            return false;
        }
        _circles.append(fenceCircle);
    }

    if (json.contains(_jsonBreachReturnKey)) {
        if (!JsonHelper::loadGeoCoordinate(json[_jsonBreachReturnKey], true /* altitudeRequred */, _breachReturnPoint, errorString)) {
            return false;
        }
        _breachReturnAltitudeFact.setRawValue(_breachReturnPoint.altitude());
    } else {
        _breachReturnPoint = QGeoCoordinate();
        _breachReturnAltitudeFact.setRawValue(_breachReturnDefaultAltitude);
    }
    emit breachReturnPointChanged(_breachReturnPoint);

    setDirty(false);

    return true;
}

void GeoFenceController::saveToJson(QJsonObject& json)
{
    json[JsonHelper::jsonVersionKey] = _jsonCurrentVersion;

    QJsonArray jsonPolygonArray;
    for (int i=0; i<_polygons.count(); i++) {
        QJsonObject jsonPolygon;
        QGCFencePolygon* fencePolygon = _polygons.value<QGCFencePolygon*>(i);
        fencePolygon->saveToJson(jsonPolygon);
        jsonPolygonArray.append(jsonPolygon);
    }
    json[_jsonPolygonsKey] = jsonPolygonArray;

    QJsonArray jsonCircleArray;
    for (int i=0; i<_circles.count(); i++) {
        QJsonObject jsonCircle;
        QGCFenceCircle* fenceCircle = _circles.value<QGCFenceCircle*>(i);
        fenceCircle->saveToJson(jsonCircle);
        jsonCircleArray.append(jsonCircle);
    }
    json[_jsonCirclesKey] = jsonCircleArray;

    if (_breachReturnPoint.isValid()) {
        QJsonValue jsonCoordinate;

        _breachReturnPoint.setAltitude(_breachReturnAltitudeFact.rawValue().toDouble());
        JsonHelper::saveGeoCoordinate(_breachReturnPoint, true /* writeAltitude */, jsonCoordinate);
        json[_jsonBreachReturnKey] = jsonCoordinate;
    }
}

void GeoFenceController::removeAll(void)
{    
    setBreachReturnPoint(QGeoCoordinate());
    _polygons.clearAndDeleteContents();
    _circles.clearAndDeleteContents();
}

void GeoFenceController::removeAllFromVehicle(void)
{
    if (_vehicle->isOfflineEditingVehicle()) {
        qCWarning(GeoFenceControllerLog) << "GeoFenceController::removeAllFromVehicle called while offline";
        emit removeAllComplete(true);
    } else if (syncInProgress()) {
        qCWarning(GeoFenceControllerLog) << "GeoFenceController::removeAllFromVehicle called while syncInProgress";
        emit removeAllComplete(true);
    } else {
        _geoFenceManager->removeAll();
    }
}

void GeoFenceController::loadFromVehicle(void)
{
    if (_vehicle->isOfflineEditingVehicle()) {
        qCWarning(GeoFenceControllerLog) << "GeoFenceController::loadFromVehicle called while offline";
    } else if (syncInProgress()) {
        qCWarning(GeoFenceControllerLog) << "GeoFenceController::loadFromVehicle called while syncInProgress";
    } else {
        _geoFenceManager->loadFromVehicle();
    }
}

void GeoFenceController::sendToVehicle(void)
{
    if (!supported()) {
        qCDebug(GeoFenceControllerLog) << "sendToVehicle Not supported";
        emit sendComplete(false);
        return;
    }
    if (_vehicle->isOfflineEditingVehicle()) {
        qCWarning(GeoFenceControllerLog) << "sendToVehicle Called while offline";
        emit sendComplete(true);
        return;
    }
    if (syncInProgress()) {
        qCWarning(GeoFenceControllerLog) << "sendToVehicle Called while syncInProgress";
        emit sendComplete(true);
        return;
    }

    qCDebug(GeoFenceControllerLog) << "sendToVehicle";
    _geoFenceManager->sendToVehicle(_breachReturnPoint, _polygons, _circles);
    setDirty(false);
}

bool GeoFenceController::syncInProgress(void) const
{
    return _geoFenceManager->inProgress();
}

bool GeoFenceController::dirty(void) const
{
    return _dirty;
}


void GeoFenceController::setDirty(bool dirty)
{
    if (dirty != _dirty) {
        _dirty = dirty;
        if (!dirty) {
            for (int i=0; i<_polygons.count(); i++) {
                QGCFencePolygon* polygon = _polygons.value<QGCFencePolygon*>(i);
                polygon->setDirty(false);
            }
            for (int i=0; i<_circles.count(); i++) {
                QGCFenceCircle* circle = _circles.value<QGCFenceCircle*>(i);
                circle->setDirty(false);
            }
        }
        emit dirtyChanged(dirty);
    }
}

void GeoFenceController::_polygonDirtyChanged(bool dirty)
{
    if (dirty) {
        setDirty(true);
    }
}

void GeoFenceController::_setDirty(void)
{
    setDirty(true);
}

bool GeoFenceController::containsItems(void) const
{
    return _polygons.count() > 0 || _circles.count() > 0;
}

void GeoFenceController::_updateContainsItems(void)
{
    emit containsItemsChanged(containsItems());
}

void GeoFenceController::addInclusionPolygon(QGeoCoordinate topLeft, QGeoCoordinate bottomRight)
{
    QGeoCoordinate topRight(topLeft.latitude(), bottomRight.longitude());
    QGeoCoordinate bottomLeft(bottomRight.latitude(), topLeft.longitude());

    double halfWidthMeters = topLeft.distanceTo(topRight) / 2.0;
    double halfHeightMeters = topLeft.distanceTo(bottomLeft) / 2.0;

    QGeoCoordinate centerLeftEdge = topLeft.atDistanceAndAzimuth(halfHeightMeters, 180);
    QGeoCoordinate centerTopEdge = topLeft.atDistanceAndAzimuth(halfWidthMeters, 90);
    QGeoCoordinate center(centerLeftEdge.latitude(), centerTopEdge.longitude());

    // Initial polygon is inset to take 3/4s of viewport with max width/height of 3000 meters
    halfWidthMeters =   qMin(halfWidthMeters * 0.75, 1500.0);
    halfHeightMeters =  qMin(halfHeightMeters * 0.75, 1500.0);

    // Initial polygon has max width and height of 3000 meters
    topLeft =           center.atDistanceAndAzimuth(halfWidthMeters, -90).atDistanceAndAzimuth(halfHeightMeters, 0);
    topRight =          center.atDistanceAndAzimuth(halfWidthMeters, 90).atDistanceAndAzimuth(halfHeightMeters, 0);
    bottomLeft =        center.atDistanceAndAzimuth(halfWidthMeters, -90).atDistanceAndAzimuth(halfHeightMeters, 180);
    bottomRight =       center.atDistanceAndAzimuth(halfWidthMeters, 90).atDistanceAndAzimuth(halfHeightMeters, 180);

    QGCFencePolygon* polygon = new QGCFencePolygon(true /* inclusion */, this);
    polygon->appendVertex(topLeft);
    polygon->appendVertex(topRight);
    polygon->appendVertex(bottomRight);
    polygon->appendVertex(bottomLeft);
    _polygons.append(polygon);

    clearAllInteractive();
    polygon->setInteractive(true);
}

void GeoFenceController::addInclusionCircle(QGeoCoordinate topLeft, QGeoCoordinate bottomRight)
{
    QGeoCoordinate topRight(topLeft.latitude(), bottomRight.longitude());
    QGeoCoordinate bottomLeft(bottomRight.latitude(), topLeft.longitude());

    // Initial radius is inset to take 3/4s of viewport and max of 1500 meters
    double halfWidthMeters = topLeft.distanceTo(topRight) / 2.0;
    double halfHeightMeters = topLeft.distanceTo(bottomLeft) / 2.0;
    double radius = qMin(qMin(halfWidthMeters, halfHeightMeters) * 0.75, 1500.0);

    QGeoCoordinate centerLeftEdge = topLeft.atDistanceAndAzimuth(halfHeightMeters, 180);
    QGeoCoordinate centerTopEdge = topLeft.atDistanceAndAzimuth(halfWidthMeters, 90);
    QGeoCoordinate center(centerLeftEdge.latitude(), centerTopEdge.longitude());

    QGCFenceCircle* circle = new QGCFenceCircle(center, radius, true /* inclusion */, this);
    _circles.append(circle);

    clearAllInteractive();
    circle->setInteractive(true);
}

void GeoFenceController::deletePolygon(int index)
{
    if (index < 0 || index > _polygons.count() - 1) {
        return;
    }

    QGCFencePolygon* polygon = qobject_cast<QGCFencePolygon*>(_polygons.removeAt(index));
    polygon->deleteLater();
}

void GeoFenceController::deleteCircle(int index)
{
    if (index < 0 || index > _circles.count() - 1) {
        return;
    }

    QGCFenceCircle* circle = qobject_cast<QGCFenceCircle*>(_circles.removeAt(index));
    circle->deleteLater();
}

void GeoFenceController::clearAllInteractive(void)
{
    for (int i=0; i<_polygons.count(); i++) {
        _polygons.value<QGCFencePolygon*>(i)->setInteractive(false);
    }
    for (int i=0; i<_circles.count(); i++) {
        _circles.value<QGCFenceCircle*>(i)->setInteractive(false);
    }
}

bool GeoFenceController::supported(void) const
{
    return (_vehicle->capabilityBits() & MAV_PROTOCOL_CAPABILITY_MISSION_FENCE) && (_vehicle->maxProtoVersion() >= 200);
}

// Hack for PX4
double GeoFenceController::paramCircularFence(void)
{
    if (_vehicle->isOfflineEditingVehicle() || !_vehicle->parameterManager()->parameterExists(FactSystem::defaultComponentId, _px4ParamCircularFence)) {
        return 0;
    }

    return _vehicle->parameterManager()->getParameter(FactSystem::defaultComponentId, _px4ParamCircularFence)->rawValue().toDouble();
}

void GeoFenceController::_parametersReady(void)
{
    if (_vehicle->isOfflineEditingVehicle() || !_vehicle->parameterManager()->parameterExists(FactSystem::defaultComponentId, _px4ParamCircularFence)) {
        emit paramCircularFenceChanged();
        return;
    }

    disconnect(_vehicle->parameterManager(), &ParameterManager::parametersReadyChanged, this, &GeoFenceController::_parametersReady);

    _px4ParamCircularFenceFact = _vehicle->parameterManager()->getParameter(FactSystem::defaultComponentId, _px4ParamCircularFence);
    connect(_px4ParamCircularFenceFact, &Fact::rawValueChanged, this, &GeoFenceController::paramCircularFenceChanged);
    emit paramCircularFenceChanged();
}

bool GeoFenceController::isEmpty(void) const
{
    return _polygons.count() == 0 && _circles.count() == 0 && !_breachReturnPoint.isValid();

}

void GeoFenceController::_managerLoadComplete(bool error)
{
    _polygons.clearAndDeleteContents();
    _circles.clearAndDeleteContents();

    if (error) {
        _breachReturnPoint = QGeoCoordinate();
        emit breachReturnPointChanged(_breachReturnPoint);
        _breachReturnAltitudeFact.setRawValue(_breachReturnDefaultAltitude);
    } else {
        _polygons.beginReset();
        for (int i=0; i<_geoFenceManager->polygons().count(); i++) {
            _polygons.append(new QGCFencePolygon(_geoFenceManager->polygons()[i], this));
        }
        _polygons.endReset();

        _circles.beginReset();
        for (int i=0; i<_geoFenceManager->circles().count(); i++) {
            _circles.append(new QGCFenceCircle(_geoFenceManager->circles()[i], this));
        }
        _circles.endReset();

        _breachReturnPoint = _geoFenceManager->breachReturnPoint();
        emit breachReturnPointChanged(_breachReturnPoint);
        if (_breachReturnPoint.isValid()) {
            _breachReturnAltitudeFact.setRawValue(_breachReturnPoint.altitude());
        } else {
            _breachReturnAltitudeFact.setRawValue(_breachReturnDefaultAltitude);
        }
    }

    setDirty(false);

    emit loadComplete(error);
}
