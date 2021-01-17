/****************************************************************************
 *
 * (c) 2009-2020 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "PlanMasterController.h"
#include "QGCApplication.h"
#include "QGCCorePlugin.h"
#include "MultiVehicleManager.h"
#include "SettingsManager.h"
#include "AppSettings.h"
#include "JsonHelper.h"
#include "MissionManager.h"
#include "KMLPlanDomDocument.h"
#include "SurveyPlanCreator.h"
#include "StructureScanPlanCreator.h"
#include "CorridorScanPlanCreator.h"
#include "BlankPlanCreator.h"
#include "MissionController.h"
#include "GeoFenceController.h"
#include "RallyPointController.h"
#include "MissionManager.h"
#include "GeoFenceManager.h"
#include "RallyPointManager.h"
#if defined(QGC_AIRMAP_ENABLED)
#include "AirspaceFlightPlanProvider.h"
#endif

#include <QDomDocument>
#include <QJsonDocument>
#include <QFileInfo>

QGC_LOGGING_CATEGORY(PlanMasterControllerLog, "PlanMasterControllerLog")

const int   PlanMasterController::kPlanFileVersion =            1;
const char* PlanMasterController::kPlanFileType =               "Plan";
const char* PlanMasterController::kJsonMissionObjectKey =       "mission";
const char* PlanMasterController::kJsonGeoFenceObjectKey =      "geoFence";
const char* PlanMasterController::kJsonRallyPointsObjectKey =   "rallyPoints";

PlanMasterController::PlanMasterController(Vehicle* vehicle)
    : QObject               (vehicle)
    , _vehicle              (vehicle)
    , _missionManager       (new MissionManager(vehicle, this))
    , _geoFenceManager      (new GeoFenceManager(vehicle, this))
    , _rallyPointManager    (new RallyPointManager(vehicle, this))
    , _missionController    (new MissionController(_missionManager, this))
    , _geoFenceController   (new GeoFenceController(_geoFenceManager, this))
    , _rallyPointController (new RallyPointController(_rallyPointManager, this))
    , _sendStateMachine     (this)
    , _receiveStateMachine  (this)
{
    connect(_missionController,    &MissionController::dirtyChanged,               this, &PlanMasterController::dirtyChanged);
    connect(_geoFenceController,   &GeoFenceController::dirtyChanged,              this, &PlanMasterController::dirtyChanged);
    connect(_rallyPointController, &RallyPointController::dirtyChanged,            this, &PlanMasterController::dirtyChanged);

    connect(_missionController,    &MissionController::containsItemsChanged,       this, &PlanMasterController::containsItemsChanged);
    connect(_geoFenceController,   &GeoFenceController::containsItemsChanged,      this, &PlanMasterController::containsItemsChanged);
    connect(_rallyPointController, &RallyPointController::containsItemsChanged,    this, &PlanMasterController::containsItemsChanged);

    connect(_missionController,    &MissionController::syncInProgressChanged,      this, &PlanMasterController::syncInProgressChanged);
    connect(_geoFenceController,   &GeoFenceController::syncInProgressChanged,     this, &PlanMasterController::syncInProgressChanged);
    connect(_rallyPointController, &RallyPointController::syncInProgressChanged,   this, &PlanMasterController::syncInProgressChanged);

    _setupPlanCreatorsList();
}

PlanMasterController::~PlanMasterController()
{

}

void PlanMasterController::loadFromVehicle(void)
{
    if (_vehicle->isOfflineEditingVehicle()) {
        emit loadComplete();
        return;
    }
    if (syncInProgress()) {
        qCWarning(PlanMasterControllerLog) << "PlanMasterController::loadFromVehicle called while syncInProgress";
        emit loadComplete();
        return;
    }

    WeakLinkInterfacePtr weakLink = _vehicle->vehicleLinkManager()->primaryLink();
    if (weakLink.expired()) {
        emit loadComplete();
        return;
    } else {
        SharedLinkInterfacePtr sharedLink = weakLink.lock();
        if (sharedLink->linkConfiguration()->isHighLatency()) {
            qgcApp()->showAppMessage(tr("Download not supported on high latency links."));
            emit loadComplete();
            return;
        }
    }

    qCDebug(PlanMasterControllerLog) << "loadFromVehicle starting state machine";
    _receiveStateMachine.start();
}


void PlanMasterController::_sendToVehicle(void)
{
    WeakLinkInterfacePtr weakLink = _vehicle->vehicleLinkManager()->primaryLink();
    if (weakLink.expired()) {
        // Vehicle is shutting down
        return;
    } else {
        SharedLinkInterfacePtr sharedLink = weakLink.lock();
        if (sharedLink->linkConfiguration()->isHighLatency()) {
            qgcApp()->showAppMessage(tr("Upload not supported on high latency links."));
            return;
        }
    }

    if (_vehicle->isOfflineEditingVehicle()) {
        qCDebug(PlanMasterControllerLog) << "_sendToVehicle - Skipping for offline vehicle";
    } else if (syncInProgress()) {
        qCWarning(PlanMasterControllerLog) << "_sendToVehicle called while syncInProgress";
    } else {
        qCDebug(PlanMasterControllerLog) << "_sendToVehicle state machine started";
        _sendStateMachine.start();
    }
}

void PlanMasterController::loadFromFileAndSendToVehicle(const QString& filename)
{
    QString errorString;
    QString errorMessage = tr("Error loading Plan file (%1). %2").arg(filename).arg("%1");

    if (filename.isEmpty()) {
        return;
    }

    QFileInfo fileInfo(filename);
    QFile file(filename);

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        errorString = file.errorString() + QStringLiteral(" ") + filename;
        qgcApp()->showAppMessage(errorMessage.arg(errorString));
        return;
    }

    bool success = false;
    if (fileInfo.suffix() == AppSettings::missionFileExtension) {
        if (!_missionController->loadJsonFile(file, errorString)) {
            qgcApp()->showAppMessage(errorMessage.arg(errorString));
        } else {
            success = true;
        }
    } else if (fileInfo.suffix() == AppSettings::waypointsFileExtension || fileInfo.suffix() == QStringLiteral("txt")) {
        if (!_missionController->loadTextFile(file, errorString)) {
            qgcApp()->showAppMessage(errorMessage.arg(errorString));
        } else {
            success = true;
        }
    } else {
        QJsonDocument   jsonDoc;
        QByteArray      bytes = file.readAll();

        if (!JsonHelper::isJsonFile(bytes, jsonDoc, errorString)) {
            qgcApp()->showAppMessage(errorMessage.arg(errorString));
            return;
        }

        QJsonObject json = jsonDoc.object();
        //-- Allow plugins to pre process the load
        qgcApp()->toolbox()->corePlugin()->preLoadFromJson(this, json);

        int version;
        if (!JsonHelper::validateExternalQGCJsonFile(json, kPlanFileType, kPlanFileVersion, kPlanFileVersion, version, errorString)) {
            qgcApp()->showAppMessage(errorMessage.arg(errorString));
            return;
        }

        QList<JsonHelper::KeyValidateInfo> rgKeyInfo = {
            { kJsonMissionObjectKey,        QJsonValue::Object, true },
            { kJsonGeoFenceObjectKey,       QJsonValue::Object, true },
            { kJsonRallyPointsObjectKey,    QJsonValue::Object, true },
        };
        if (!JsonHelper::validateKeys(json, rgKeyInfo, errorString)) {
            qgcApp()->showAppMessage(errorMessage.arg(errorString));
            return;
        }

        if (!_missionController->loadFromJson(json[kJsonMissionObjectKey].toObject(), errorString) ||
                !_geoFenceController->loadFromJson(json[kJsonGeoFenceObjectKey].toObject(), errorString) ||
                !_rallyPointController->loadFromJson(json[kJsonRallyPointsObjectKey].toObject(), errorString)) {
            qgcApp()->showAppMessage(errorMessage.arg(errorString));
        } else {
            //-- Allow plugins to post process the load
            qgcApp()->toolbox()->corePlugin()->postLoadFromJson(this, json);
            success = true;
        }
    }

    if (success){
        _currentPlanFile = QString::asprintf("%s/%s.%s", fileInfo.path().toLocal8Bit().data(), fileInfo.completeBaseName().toLocal8Bit().data(), AppSettings::planFileExtension);
        emit currentPlanFileChanged();
        _sendToVehicle();
    } else {
        removeAll();
    }
}

QJsonDocument PlanMasterController::saveToJson()
{
    QJsonObject planJson;
    QJsonObject missionJson;
    QJsonObject fenceJson;
    QJsonObject rallyJson;

    qgcApp()->toolbox()->corePlugin()->preSaveToJson(this, planJson);

    JsonHelper::saveQGCJsonFileHeader(planJson, kPlanFileType, kPlanFileVersion);

    qgcApp()->toolbox()->corePlugin()->preSaveToMissionJson(this, missionJson);
    _missionController->saveToJson(missionJson);
    qgcApp()->toolbox()->corePlugin()->postSaveToMissionJson(this, missionJson);

    _geoFenceController->saveToJson(fenceJson);
    _rallyPointController->saveToJson(rallyJson);

    planJson[kJsonMissionObjectKey] = missionJson;
    planJson[kJsonGeoFenceObjectKey] = fenceJson;
    planJson[kJsonRallyPointsObjectKey] = rallyJson;

    qgcApp()->toolbox()->corePlugin()->postSaveToJson(this, planJson);

    return QJsonDocument(planJson);
}

void PlanMasterController::saveToFileAndSendToVehicle(void)
{
    if (_currentPlanFile.isEmpty()) {
        return;
    }

    QString planFilename = _currentPlanFile;
    if (!QFileInfo(_currentPlanFile).fileName().contains(".")) {
        planFilename += QString(".%1").arg(fileExtension());
    }

    QFile file(planFilename);

    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qgcApp()->showAppMessage(tr("Plan save error %1 : %2").arg(_currentPlanFile).arg(file.errorString()));
        _currentPlanFile.clear();
        emit currentPlanFileChanged();
    } else {
        QJsonDocument saveDoc = saveToJson();
        file.write(saveDoc.toJson());
        if(_currentPlanFile != planFilename) {
            _currentPlanFile = planFilename;
            emit currentPlanFileChanged();
        }
    }

    setDirty(false);

    _sendToVehicle();
}

void PlanMasterController::saveToKml(const QString& filename)
{
    if (filename.isEmpty()) {
        return;
    }

    QString kmlFilename = filename;
    if (!QFileInfo(filename).fileName().contains(".")) {
        kmlFilename += QString(".%1").arg(kmlFileExtension());
    }

    QFile file(kmlFilename);

    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qgcApp()->showAppMessage(tr("KML save error %1 : %2").arg(filename).arg(file.errorString()));
    } else {
        KMLPlanDomDocument planKML;
        _missionController->addMissionToKML(planKML);
        QTextStream stream(&file);
        stream << planKML.toString();
        file.close();
    }
}

void PlanMasterController::removeAll(void)
{
    _missionController->removeAll();
    _geoFenceController->removeAll();
    _rallyPointController->removeAll();

    if (!_vehicle->isOfflineEditingVehicle()) {
        _missionController->removeAllFromVehicle();
        _geoFenceController->removeAllFromVehicle();
        _rallyPointController->removeAllFromVehicle();
    }

    setDirty(false);

    _currentPlanFile.clear();
    emit currentPlanFileChanged();
}

bool PlanMasterController::containsItems(void) const
{
    return _missionController->containsItems() || _geoFenceController->containsItems() || _rallyPointController->containsItems();
}

bool PlanMasterController::dirty(void) const
{
    return _missionController->dirty() || _geoFenceController->dirty() || _rallyPointController->dirty();
}

void PlanMasterController::setDirty(bool dirty)
{
    _missionController->setDirty(dirty);
    _geoFenceController->setDirty(dirty);
    _rallyPointController->setDirty(dirty);
}

QString PlanMasterController::fileExtension(void) const
{
    return AppSettings::planFileExtension;
}

QString PlanMasterController::kmlFileExtension(void) const
{
    return AppSettings::kmlFileExtension;
}

QStringList PlanMasterController::loadNameFilters(void) const
{
    QStringList filters;

    filters << tr("Supported types (*.%1 *.%2 *.%3 *.%4)").arg(AppSettings::planFileExtension).arg(AppSettings::missionFileExtension).arg(AppSettings::waypointsFileExtension).arg("txt") <<
               tr("All Files (*)");
    return filters;
}


QStringList PlanMasterController::saveNameFilters(void) const
{
    QStringList filters;

    filters << tr("Plan Files (*.%1)").arg(fileExtension()) << tr("All Files (*)");
    return filters;
}

void PlanMasterController::_showPlanFromManagerVehicle(void)
{
#if 0
    if (!_vehicle->initialPlanRequestComplete() && !syncInProgress()) {
        // Something went wrong with initial load. All controllers are idle, so just force it off
        _vehicle->forceInitialPlanRequestComplete();
    }

    // The crazy if structure is to handle the load propagating by itself through the system
    if (!_missionController->showPlanFromManagerVehicle()) {
        if (!_geoFenceController->showPlanFromManagerVehicle()) {
            _rallyPointController->showPlanFromManagerVehicle();
        }
    }
#endif
}

bool PlanMasterController::syncInProgress(void) const
{
    return _missionController->syncInProgress() ||
            _geoFenceController->syncInProgress() ||
            _rallyPointController->syncInProgress();
}

bool PlanMasterController::isEmpty(void) const
{
    return _missionController->isEmpty() &&
            _geoFenceController->isEmpty() &&
            _rallyPointController->isEmpty();
}

void PlanMasterController::_setupPlanCreatorsList(void)
{
    _planCreators.beginReset();

    _planCreators.append(new BlankPlanCreator(this, this));
    _planCreators.append(new SurveyPlanCreator(this, this));
    _planCreators.append(new CorridorScanPlanCreator(this, this));

    if (!_vehicle->fixedWing()) {
        _planCreators.append(new StructureScanPlanCreator(this, this));
    }

    _planCreators.endReset();
}

int PlanMasterController::readyForSaveState(void) const
{
    return _missionController->readyForSaveState();
}

const StateMachine::StateFn PMCSendStateMachine::_rgStates[] = {
    PMCSendStateMachine::_stateSendMission,
    PMCSendStateMachine::_stateSendGeoFence,
    PMCSendStateMachine::_stateSendRallyPoints,
};

const int PMCSendStateMachine::_cStates = sizeof(PMCSendStateMachine::_rgStates) / sizeof(PMCSendStateMachine::_rgStates[0]);

PMCSendStateMachine::PMCSendStateMachine(PlanMasterController* masterController)
    : _masterController(masterController)
{

}

int PMCSendStateMachine::stateCount(void) const
{
    return _cStates;
}

const StateMachine::StateFn* PMCSendStateMachine::rgStates(void) const
{
    return _rgStates;
}

void PMCSendStateMachine::_stateSendMission(StateMachine* stateMachine)
{
    PMCSendStateMachine*    sendStateMachine    = qobject_cast<PMCSendStateMachine*>(stateMachine);
    MissionController*      missionController   = sendStateMachine->_masterController->missionController();

    qCDebug(PlanMasterControllerLog) << "_stateSendMission";

    connect(missionController, &PlanElementController::sendComplete, sendStateMachine, &PMCSendStateMachine::_advanceOnSendComplete);
    missionController->sendToVehicle();
}

void PMCSendStateMachine::_stateSendGeoFence(StateMachine* stateMachine)
{
    PMCSendStateMachine*    sendStateMachine    = qobject_cast<PMCSendStateMachine*>(stateMachine);
    GeoFenceController*     geoFenceController  = sendStateMachine->_masterController->geoFenceController();

    qCDebug(PlanMasterControllerLog) << "_stateSendGeoFence";

    connect(geoFenceController, &PlanElementController::sendComplete, sendStateMachine, &PMCSendStateMachine::_advanceOnSendComplete);
    geoFenceController->sendToVehicle();
}

void PMCSendStateMachine::_stateSendRallyPoints(StateMachine* stateMachine)
{
    PMCSendStateMachine*    sendStateMachine        = qobject_cast<PMCSendStateMachine*>(stateMachine);
    RallyPointController*   rallyPointController    = sendStateMachine->_masterController->rallyPointController();

    qCDebug(PlanMasterControllerLog) << "_stateSendGeoFence";

    connect(rallyPointController, &PlanElementController::sendComplete, sendStateMachine, &PMCSendStateMachine::_advanceOnSendComplete);
    rallyPointController->sendToVehicle();
}

void PMCSendStateMachine::_advanceOnSendComplete(bool /*error*/)
{
    PlanElementController* elementController = qobject_cast<PlanElementController*>(sender());

    disconnect(elementController, &PlanElementController::sendComplete, this, &PMCSendStateMachine::_advanceOnSendComplete);
    advance();
}

void PMCSendStateMachine::statesCompleted (void) const
{
    emit _masterController->sendComplete();
}

const StateMachine::StateFn PMCReceiveStateMachine::_rgStates[] = {
    PMCReceiveStateMachine::_stateReceiveMission,
    PMCReceiveStateMachine::_stateReceiveGeoFence,
    PMCReceiveStateMachine::_stateReceiveRallyPoints,
};

const int PMCReceiveStateMachine::_cStates = sizeof(PMCReceiveStateMachine::_rgStates) / sizeof(PMCReceiveStateMachine::_rgStates[0]);

PMCReceiveStateMachine::PMCReceiveStateMachine(PlanMasterController* masterController)
    : _masterController(masterController)
{

}

int PMCReceiveStateMachine::stateCount(void) const
{
    return _cStates;
}

const StateMachine::StateFn* PMCReceiveStateMachine::rgStates(void) const
{
    return _rgStates;
}

void PMCReceiveStateMachine::_stateReceiveMission(StateMachine* stateMachine)
{
    PMCReceiveStateMachine* receiveStateMachine = qobject_cast<PMCReceiveStateMachine*>(stateMachine);
    MissionController*      missionController   = receiveStateMachine->_masterController->missionController();

    qCDebug(PlanMasterControllerLog) << "_stateReceiveMission";

    connect(missionController, &PlanElementController::loadComplete, receiveStateMachine, &PMCReceiveStateMachine::_advanceOnLoadComplete);
    missionController->loadFromVehicle();
}

void PMCReceiveStateMachine::_stateReceiveGeoFence(StateMachine* stateMachine)
{
    PMCReceiveStateMachine* receiveStateMachine = qobject_cast<PMCReceiveStateMachine*>(stateMachine);
    GeoFenceController*     geoFenceController  = receiveStateMachine->_masterController->geoFenceController();

    qCDebug(PlanMasterControllerLog) << "_stateReceiveGeoFence";

    connect(geoFenceController, &PlanElementController::loadComplete, receiveStateMachine, &PMCReceiveStateMachine::_advanceOnLoadComplete);
    geoFenceController->loadFromVehicle();
}

void PMCReceiveStateMachine::_stateReceiveRallyPoints(StateMachine* stateMachine)
{
    PMCReceiveStateMachine* receiveStateMachine     = qobject_cast<PMCReceiveStateMachine*>(stateMachine);
    RallyPointController*   rallyPointController    = receiveStateMachine->_masterController->rallyPointController();

    qCDebug(PlanMasterControllerLog) << "_stateReceiveRallyPoints";

    connect(rallyPointController, &PlanElementController::loadComplete, receiveStateMachine, &PMCReceiveStateMachine::_advanceOnLoadComplete);
    rallyPointController->loadFromVehicle();
}

void PMCReceiveStateMachine::_advanceOnLoadComplete(bool /*error*/)
{
    PlanElementController* elementController = qobject_cast<PlanElementController*>(sender());

    disconnect(elementController, &PlanElementController::loadComplete, this, &PMCReceiveStateMachine::_advanceOnLoadComplete);
    advance();
}

void PMCReceiveStateMachine::statesCompleted (void) const
{
    emit _masterController->loadComplete();
}
