/****************************************************************************
 *
 * (c) 2009-2020 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#pragma once

#include "LinkConfiguration.h"
#include "QGCMAVLink.h"

class MockConfiguration : public LinkConfiguration
{
    Q_OBJECT

public:
    MockConfiguration(const QString& name);
    MockConfiguration(MockConfiguration* source);

    Q_PROPERTY(int      firmware            READ firmware           WRITE setFirmware           NOTIFY firmwareChanged)
    Q_PROPERTY(int      vehicle             READ vehicle            WRITE setVehicle            NOTIFY vehicleChanged)
    Q_PROPERTY(bool     sendStatus          READ sendStatusText     WRITE setSendStatusText     NOTIFY sendStatusChanged)
    Q_PROPERTY(bool     incrementVehicleId  READ incrementVehicleId WRITE setIncrementVehicleId NOTIFY incrementVehicleIdChanged)
    Q_PROPERTY(bool     twoVehicles         READ twoVehicles        WRITE setTwoVehicles        NOTIFY twoVehiclesChanged)

    int     firmware                (void)                      { return (int)_firmwareType; }
    void    setFirmware             (int type)                  { _firmwareType = (MAV_AUTOPILOT)type; emit firmwareChanged(); }
    int     vehicle                 (void)                      { return (int)_vehicleType; }
    bool    incrementVehicleId      (void)                      { return _incrementVehicleId; }
    bool    twoVehicles             (void)                      { return _twoVehicles; }
    void    setVehicle              (int type)                  { _vehicleType = (MAV_TYPE)type; emit vehicleChanged(); }
    void    setIncrementVehicleId   (bool incrementVehicleId)   { _incrementVehicleId = incrementVehicleId; emit incrementVehicleIdChanged(); }
    void    setTwoVehicles          (bool twoVehicles)          { _twoVehicles = twoVehicles; emit twoVehiclesChanged(); }

    MAV_AUTOPILOT   firmwareType        (void)                          { return _firmwareType; }
    MAV_TYPE        vehicleType         (void)                          { return _vehicleType; }
    bool            sendStatusText      (void)                          { return _sendStatusText; }

    void            setFirmwareType     (MAV_AUTOPILOT firmwareType)    { _firmwareType = firmwareType; emit firmwareChanged(); }
    void            setVehicleType      (MAV_TYPE vehicleType)          { _vehicleType = vehicleType; emit vehicleChanged(); }
    void            setSendStatusText   (bool sendStatusText)           { _sendStatusText = sendStatusText; emit sendStatusChanged(); }

    typedef enum {
        FailNone,                           // No failures
        FailParamNoReponseToRequestList,    // Do no respond to PARAM_REQUEST_LIST
        FailMissingParamOnInitialReqest,    // Not all params are sent on initial request, should still succeed since QGC will re-query missing params
        FailMissingParamOnAllRequests,      // Not all params are sent on initial request, QGC retries will fail as well
    } FailureMode_t;
    FailureMode_t failureMode(void) { return _failureMode; }
    void setFailureMode(FailureMode_t failureMode) { _failureMode = failureMode; }

    // Overrides from LinkConfiguration
    LinkType    type            (void) override                                         { return LinkConfiguration::TypeMock; }
    void        copyFrom        (LinkConfiguration* source) override;
    void        loadSettings    (QSettings& settings, const QString& root) override;
    void        saveSettings    (QSettings& settings, const QString& root) override;
    QString     settingsURL     (void) override                                         { return "MockLinkSettings.qml"; }
    QString     settingsTitle   (void) override                                         { return tr("Mock Link Settings"); }

signals:
    void firmwareChanged            (void);
    void vehicleChanged             (void);
    void sendStatusChanged          (void);
    void incrementVehicleIdChanged  (void);
    void twoVehiclesChanged         (void);

private:
    MAV_AUTOPILOT   _firmwareType       = MAV_AUTOPILOT_PX4;
    MAV_TYPE        _vehicleType        = MAV_TYPE_QUADROTOR;
    bool            _sendStatusText     = false;
    FailureMode_t   _failureMode        = FailNone;
    bool            _incrementVehicleId = true;                 ///< false: can be used to create multiple links to the same vehicle
    bool            _twoVehicles        = false;                ///< true: expose two vehicles from a single link

    static const char* _firmwareTypeKey;
    static const char* _vehicleTypeKey;
    static const char* _sendStatusTextKey;
    static const char* _incrementVehicleIdKey;
    static const char* _failureModeKey;
    static const char* _twoVehiclesKey;
};
