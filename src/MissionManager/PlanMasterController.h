/****************************************************************************
 *
 * (c) 2009-2020 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#pragma once

#include <QObject>

#include "QGCLoggingCategory.h"
#include "QmlObjectListModel.h"
#include "StateMachine.h"

class Vehicle;
class MissionController;
class GeoFenceController;
class RallyPointController;
class MissionManager;
class GeoFenceManager;
class RallyPointManager;

Q_DECLARE_LOGGING_CATEGORY(PlanMasterControllerLog)

class PlanMasterController;

/// Controls all interactions of mission, fence/rally points between vehicle and ui

class PMCSendStateMachine : public StateMachine {
    Q_OBJECT

public:
    PMCSendStateMachine(PlanMasterController* masterController);

    int             stateCount      (void) const final;
    const StateFn*  rgStates        (void) const final;
    void            statesCompleted (void) const final;

private slots:
    void _advanceOnSendComplete     (bool error);

private:
    static void _stateSendMission       (StateMachine* stateMachine);
    static void _stateSendGeoFence      (StateMachine* stateMachine);
    static void _stateSendRallyPoints   (StateMachine* stateMachine);

    PlanMasterController*   _masterController = nullptr;

    static const StateFn    _rgStates[];
    static const int        _cStates;
};

class PMCReceiveStateMachine : public StateMachine {
    Q_OBJECT

public:
    PMCReceiveStateMachine(PlanMasterController* masterController);

    int             stateCount      (void) const final;
    const StateFn*  rgStates        (void) const final;
    void            statesCompleted (void) const final;

private slots:
    void _advanceOnLoadComplete     (bool error);

private:
    static void _stateReceiveMission    (StateMachine* stateMachine);
    static void _stateReceiveGeoFence   (StateMachine* stateMachine);
    static void _stateReceiveRallyPoints(StateMachine* stateMachine);

    PlanMasterController*   _masterController = nullptr;

    static const StateFn    _rgStates[];
    static const int        _cStates;
};

class PlanMasterController : public QObject
{
    Q_OBJECT
    
public:
    PlanMasterController(Vehicle* vehicle);
    ~PlanMasterController();

    Q_PROPERTY(Vehicle*                 vehicle                 READ vehicle                                CONSTANT)
    Q_PROPERTY(MissionController*       missionController       READ missionController                      CONSTANT)
    Q_PROPERTY(GeoFenceController*      geoFenceController      READ geoFenceController                     CONSTANT)
    Q_PROPERTY(RallyPointController*    rallyPointController    READ rallyPointController                   CONSTANT)
    Q_PROPERTY(bool                     containsItems           READ containsItems                          NOTIFY containsItemsChanged)    ///< true: Elemement is non-empty
    Q_PROPERTY(bool                     syncInProgress          READ syncInProgress                         NOTIFY syncInProgressChanged)   ///< true: Information is currently being saved/sent, false: no active save/send in progress
    Q_PROPERTY(bool                     dirty                   READ dirty                  WRITE setDirty  NOTIFY dirtyChanged)            ///< true: Unsaved/sent changes are present, false: no changes since last save/send
    Q_PROPERTY(QString                  fileExtension           READ fileExtension                          CONSTANT)                       ///< File extension for missions
    Q_PROPERTY(QString                  kmlFileExtension        READ kmlFileExtension                       CONSTANT)
    Q_PROPERTY(QString                  currentPlanFile         READ currentPlanFile                        NOTIFY currentPlanFileChanged)
    Q_PROPERTY(QStringList              loadNameFilters         READ loadNameFilters                        CONSTANT)                       ///< File filter list loading plan files
    Q_PROPERTY(QStringList              saveNameFilters         READ saveNameFilters                        CONSTANT)                       ///< File filter list saving plan files
    Q_PROPERTY(QmlObjectListModel*      planCreators            READ planCreators                           CONSTANT)

    /// Determines if the plan has all information needed to be saved or sent to the vehicle.
    /// IMPORTANT NOTE: The return value is a VisualMissionItem::ReadForSaveState value. It is an int here to work around
    /// a nightmare of circular header dependency problems.
    Q_INVOKABLE int readyForSaveState(void) const;

    Q_INVOKABLE void loadFromVehicle                (void);                     ///< Signals loadComplete when done
    Q_INVOKABLE void loadFromFileAndSendToVehicle   (const QString& filename);
    Q_INVOKABLE void saveToFileAndSendToVehicle     (void);                     ///< Signals sendComplete is also sent to vehicle
    Q_INVOKABLE void saveToKml                      (const QString& filename);
    Q_INVOKABLE void removeAll                      (void);                     ///< Removes all from controllers and vehicle

    Vehicle*                vehicle             (void) { return _vehicle; }
    MissionController*      missionController   (void) { return _missionController; }
    GeoFenceController*     geoFenceController  (void) { return _geoFenceController; }
    RallyPointController*   rallyPointController(void) { return _rallyPointController; }
    MissionManager*         missionManager      (void) { return _missionManager; }
    GeoFenceManager*        geoFenceManager     (void) { return _geoFenceManager; }
    RallyPointManager*      rallyPointManager   (void) { return _rallyPointManager; }
    QmlObjectListModel*     planCreators        (void) { return &_planCreators; }

    bool        containsItems   (void) const;
    bool        syncInProgress  (void) const;
    bool        dirty           (void) const;
    void        setDirty        (bool dirty);
    QString     fileExtension   (void) const;
    QString     kmlFileExtension(void) const;
    QString     currentPlanFile (void) const { return _currentPlanFile; }
    QStringList loadNameFilters (void) const;
    QStringList saveNameFilters (void) const;
    bool        isEmpty         (void) const;

    QJsonDocument saveToJson    ();

    static const int    kPlanFileVersion;
    static const char*  kPlanFileType;
    static const char*  kJsonMissionObjectKey;
    static const char*  kJsonGeoFenceObjectKey;
    static const char*  kJsonRallyPointsObjectKey;

signals:
    void containsItemsChanged   (bool containsItems);
    void syncInProgressChanged  (void);
    void dirtyChanged           (bool dirty);
    void currentPlanFileChanged (void);
    void sendComplete           (void);
    void loadComplete           (void);

private slots:
    void _setupPlanCreatorsList     (void);
#if defined(QGC_AIRMAP_ENABLED)
    void _startFlightPlanning       (void);
#endif

private:
    void _showPlanFromManagerVehicle(void);
    void _sendToVehicle             (void);

    Vehicle*                _vehicle                = nullptr;
    MissionManager*         _missionManager         = nullptr;
    GeoFenceManager*        _geoFenceManager        = nullptr;
    RallyPointManager*      _rallyPointManager      = nullptr;
    MissionController*      _missionController      = nullptr;
    GeoFenceController*     _geoFenceController     = nullptr;
    RallyPointController*   _rallyPointController   = nullptr;
    PMCSendStateMachine     _sendStateMachine;
    PMCReceiveStateMachine  _receiveStateMachine;
    QString                 _currentPlanFile;
    QmlObjectListModel      _planCreators;
};
