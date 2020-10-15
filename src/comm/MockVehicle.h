/****************************************************************************
 *
 * (c) 2009-2020 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#pragma once

#include <QElapsedTimer>
#include <QMap>
#include <QLoggingCategory>
#include <QGeoCoordinate>

#include "MockLinkMissionItemHandler.h"
#include "MockLinkFTP.h"
#include "QGCMAVLink.h"
#include "MockConfiguration.h"

class MockLink;

class MockVehicle : public QObject
{
    Q_OBJECT

    friend class MockLink;

public:
    MockVehicle(MockLink* mockLink, int vehicleId);
    ~MockVehicle(void);

    int             vehicleId           (void)                                          { return _vehicleId; }
    MAV_AUTOPILOT   getFirmwareType     (void)                                          { return _firmwareType; }
    void            setFirmwareType     (MAV_AUTOPILOT autopilot)                       { _firmwareType = autopilot; }
    void            setSendStatusText   (bool sendStatusText)                           { _sendStatusText = sendStatusText; }
    void            setFailureMode      (MockConfiguration::FailureMode_t failureMode)  { _failureMode = failureMode; }

    /// APM stack has strange handling of the first item of the mission list. If it has no
    /// onboard mission items, sometimes it sends back a home position in position 0 and
    /// sometimes it doesn't. Don't ask. This option allows you to configure that behavior
    /// for unit testing.
    void setAPMMissionResponseMode(bool sendHomePositionOnEmptyList) { _apmSendHomePositionOnEmptyList = sendHomePositionOnEmptyList; }

    void emitRemoteControlChannelRawChanged(int channel, uint16_t raw);

    /// Sends the specified mavlink message to QGC
    void respondWithMavlinkMessage(const mavlink_message_t& msg);

    MockLinkFTP* mockLinkFTP(void) { return &_mockLinkFTP; }

    /// Sets a failure mode for unit testingqgcm
    ///     @param failureMode Type of failure to simulate
    ///     @param failureAckResult Error to send if one the ack error modes
    void setMissionItemFailureMode(MockLinkMissionItemHandler::FailureMode_t failureMode, MAV_MISSION_RESULT failureAckResult);

    /// Called to send a MISSION_ACK message while the MissionManager is in idle state
    void sendUnexpectedMissionAck(MAV_MISSION_RESULT ackType) { _missionItemHandler.sendUnexpectedMissionAck(ackType); }

    /// Called to send a MISSION_ITEM message while the MissionManager is in idle state
    void sendUnexpectedMissionItem(void) { _missionItemHandler.sendUnexpectedMissionItem(); }

    /// Called to send a MISSION_REQUEST message while the MissionManager is in idle state
    void sendUnexpectedMissionRequest(void) { _missionItemHandler.sendUnexpectedMissionRequest(); }

    void sendUnexpectedCommandAck(MAV_CMD command, MAV_RESULT ackResult);

    /// Reset the state of the MissionItemHandler to no items, no transactions in progress.
    void resetMissionItemHandler(void) { _missionItemHandler.reset(); }

    /// Returns the filename for the simulated log file. Only available after a download is requested.
    QString logDownloadFile(void) { return _logDownloadFilename; }

    void mavlinkMessageReceived(const mavlink_message_t& message);

    // Special commands for testing COMMAND_LONG handlers
    static const MAV_CMD MAV_CMD_MOCKLINK_ALWAYS_RESULT_ACCEPTED            = MAV_CMD_USER_1;
    static const MAV_CMD MAV_CMD_MOCKLINK_ALWAYS_RESULT_FAILED              = MAV_CMD_USER_2;
    static const MAV_CMD MAV_CMD_MOCKLINK_SECOND_ATTEMPT_RESULT_ACCEPTED    = MAV_CMD_USER_3;
    static const MAV_CMD MAV_CMD_MOCKLINK_SECOND_ATTEMPT_RESULT_FAILED      = MAV_CMD_USER_4;
    static const MAV_CMD MAV_CMD_MOCKLINK_NO_RESPONSE                       = MAV_CMD_USER_5;

    // Special message ids for testing requestMessage support
    typedef enum {
        FailRequestMessageNone,
        FailRequestMessageCommandAcceptedMsgNotSent,
        FailRequestMessageCommandUnsupported,
        FailRequestMessageCommandNoResponse,
        FailRequestMessageCommandAcceptedSecondAttempMsgSent,
    } RequestMessageFailureMode_t;
    void setRequestMessageFailureMode(RequestMessageFailureMode_t failureMode) { _requestMessageFailureMode = failureMode; }

signals:
    void highLatencyTransmissionEnabledChanged(bool highLatencyTransmissionEnabled);

private slots:
    void _run1HzTasks(void);
    void _run10HzTasks(void);
    void _run500HzTasks(void);

private:
    void _sendHeartBeat                 (void);
    void _sendHighLatency2              (void);
    void _loadParams                    (void);
    void _handleHeartBeat               (const mavlink_message_t& msg);
    void _handleSetMode                 (const mavlink_message_t& msg);
    void _handleParamRequestList        (const mavlink_message_t& msg);
    void _handleParamSet                (const mavlink_message_t& msg);
    void _handleParamRequestRead        (const mavlink_message_t& msg);
    void _handleFTP                     (const mavlink_message_t& msg);
    void _handleCommandLong             (const mavlink_message_t& msg);
    void _handleManualControl           (const mavlink_message_t& msg);
    void _handlePreFlightCalibration    (const mavlink_command_long_t& request);
    void _handleLogRequestList          (const mavlink_message_t& msg);
    void _handleLogRequestData          (const mavlink_message_t& msg);
    void _handleParamMapRC              (const mavlink_message_t& msg);
    bool _handleRequestMessage          (const mavlink_command_long_t& request, bool& noAck);
    float _floatUnionForParam           (int componentId, const QString& paramName);
    void _setParamFloatUnionIntoMap     (int componentId, const QString& paramName, float paramFloat);
    void _sendHomePosition              (void);
    void _sendGpsRawInt                 (void);
    void _sendVibration                 (void);
    void _sendSysStatus                 (void);
    void _sendBatteryStatus             (void);
    void _sendStatusTextMessages        (void);
    void _sendChunkedStatusText         (uint16_t chunkId, bool missingChunks);
    void _respondWithAutopilotVersion   (void);
    void _sendRCChannels                (void);
    void _paramRequestListWorker        (void);
    void _logDownloadWorker             (void);
    void _sendADSBVehicles              (void);
    void _moveADSBVehicle               (void);
    void _sendVersionMetaData           (void);
    void _sendParameterMetaData         (void);

    MockLink*                   _mockLink                       = nullptr;
    MockConfiguration*          _mockConfig                     = nullptr;
    uint8_t                     _vehicleId                      = 0;
    uint8_t                     _mavBaseMode                    = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint32_t                    _mavCustomMode;
    uint8_t                     _mavState                       = MAV_STATE_STANDBY;

    QElapsedTimer               _runningTime;
    static const int32_t        _batteryMaxTimeRemaining        = 15 * 60;
    int8_t                      _battery1PctRemaining           = 100;
    int32_t                     _battery1TimeRemaining          = _batteryMaxTimeRemaining;
    MAV_BATTERY_CHARGE_STATE    _battery1ChargeState            = MAV_BATTERY_CHARGE_STATE_OK;
    int8_t                      _battery2PctRemaining           = 100;
    int32_t                     _battery2TimeRemaining          = _batteryMaxTimeRemaining;
    MAV_BATTERY_CHARGE_STATE    _battery2ChargeState            = MAV_BATTERY_CHARGE_STATE_OK;

    MAV_AUTOPILOT               _firmwareType;
    MAV_TYPE                    _vehicleType                    = MAV_TYPE_QUADROTOR;
    double                      _vehicleLatitude;
    double                      _vehicleLongitude;
    double                      _vehicleAltitude;
    bool                        _highLatencyTransmissionEnabled = true;

    MockLinkMissionItemHandler  _missionItemHandler;
    MockLinkFTP                 _mockLinkFTP;

    bool _sendStatusText = false;
    bool _apmSendHomePositionOnEmptyList = false;
    MockConfiguration::FailureMode_t _failureMode = MockConfiguration::FailNone;

    int _sendHomePositionDelayCount = 10;
    int _sendGPSPositionDelayCount = 100;

    int _currentParamRequestListComponentIndex = -1; // Current component index for param request list workflow, -1 for no request in progress
    int _currentParamRequestListParamIndex = -1;     // Current parameter index for param request list workflow

    static const uint16_t _logDownloadLogId     = 0;    ///< Id of siumulated log file
    static const uint32_t _logDownloadFileSize  = 1000; ///< Size of simulated log file

    QString     _logDownloadFilename;       ///< Filename for log download which is in progress
    uint32_t    _logDownloadCurrentOffset = 0;  ///< Current offset we are sending from
    uint32_t    _logDownloadBytesRemaining = 0; ///< Number of bytes still to send, 0 = send inactive

    QGeoCoordinate  _adsbVehicleCoordinate;
    double          _adsbAngle              = 0;

    RequestMessageFailureMode_t _requestMessageFailureMode = FailRequestMessageNone;

    QMap<int, QMap<QString, QVariant>>          _mapParamName2Value;
    QMap<int, QMap<QString, MAV_PARAM_TYPE>>    _mapParamName2MavParamType;

    QTimer  _timer1HzTasks;
    QTimer  _timer10HzTasks;
    QTimer  _timer500HzTasks;

    static double       _defaultVehicleLatitude;
    static double       _defaultVehicleLongitude;
    static double       _defaultVehicleAltitude;
    static const char*  _failParam;
};

