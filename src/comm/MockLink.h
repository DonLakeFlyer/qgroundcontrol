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
#include "LinkInterface.h"

class LinkManager;
class MockVehicle;

class MockLink : public LinkInterface
{
    Q_OBJECT

public:
    MockLink(SharedLinkConfigurationPtr& config);
    ~MockLink(void);

    MockConfiguration*  mockConfig  (void) { return _mockConfig; }
    MockVehicle*        vehicle1    (void) { return _vehicle1; }
    MockVehicle*        vehicle2    (void) { return _vehicle2; }

    /// APM stack has strange handling of the first item of the mission list. If it has no
    /// onboard mission items, sometimes it sends back a home position in position 0 and
    /// sometimes it doesn't. Don't ask. This option allows you to configure that behavior
    /// for unit testing.
    void setAPMMissionResponseMode(bool sendHomePositionOnEmptyList);

    void emitRemoteControlChannelRawChanged(int channel, uint16_t raw);

    /// Sends the specified mavlink message to QGC
    void respondWithMavlinkMessage(const mavlink_message_t& msg);

    /// Sets a failure mode for unit testingqgcm
    ///     @param failureMode Type of failure to simulate
    ///     @param failureAckResult Error to send if one the ack error modes
    void setMissionItemFailureMode(MockLinkMissionItemHandler::FailureMode_t failureMode, MAV_MISSION_RESULT failureAckResult);

    /// Called to send a MISSION_ACK message while the MissionManager is in idle state
    void sendUnexpectedMissionAck(MAV_MISSION_RESULT ackType);

    /// Called to send a MISSION_ITEM message while the MissionManager is in idle state
    void sendUnexpectedMissionItem(void);

    /// Called to send a MISSION_REQUEST message while the MissionManager is in idle state
    void sendUnexpectedMissionRequest(void);

    void sendUnexpectedCommandAck(MAV_CMD command, MAV_RESULT ackResult);

    /// Reset the state of the MissionItemHandler to no items, no transactions in progress.
    void resetMissionItemHandler(void);

    /// Returns the filename for the simulated log file. Only available after a download is requested.
    QString logDownloadFile(void);

    Q_INVOKABLE void setCommLost                    (bool commLost)   { _commLost = commLost; }
    Q_INVOKABLE void simulateConnectionRemoved      (void);
    static MockLink* startPX4MockLink               (bool sendStatusText, MockConfiguration::FailureMode_t failureMode = MockConfiguration::FailNone);
    static MockLink* startGenericMockLink           (bool sendStatusText, MockConfiguration::FailureMode_t failureMode = MockConfiguration::FailNone);
    static MockLink* startNoInitialConnectMockLink  (bool sendStatusText, MockConfiguration::FailureMode_t failureMode = MockConfiguration::FailNone);
    static MockLink* startAPMArduCopterMockLink     (bool sendStatusText, MockConfiguration::FailureMode_t failureMode = MockConfiguration::FailNone);
    static MockLink* startAPMArduPlaneMockLink      (bool sendStatusText, MockConfiguration::FailureMode_t failureMode = MockConfiguration::FailNone);
    static MockLink* startAPMArduSubMockLink        (bool sendStatusText, MockConfiguration::FailureMode_t failureMode = MockConfiguration::FailNone);
    static MockLink* startAPMArduRoverMockLink      (bool sendStatusText, MockConfiguration::FailureMode_t failureMode = MockConfiguration::FailNone);

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

    // Overrides from LinkInterface
    QString getName             (void) const override { return _name; }
    bool    isConnected         (void) const override { return _connected; }
    void    disconnect          (void) override;

signals:
    void writeBytesQueuedSignal                 (const QByteArray bytes);
    void highLatencyTransmissionEnabledChanged  (bool highLatencyTransmissionEnabled);

private slots:
    // LinkInterface overrides
    void _writeBytes(const QByteArray bytes) final;

    void _writeBytesQueued(const QByteArray bytes);

private:
    // LinkInterface overrides
    bool _connect(void) override;

    // QThread overrides
    void run(void) final;

    static MockLink* _startMockLinkWorker(QString configName, MAV_AUTOPILOT firmwareType, MAV_TYPE vehicleType, bool sendStatusText, MockConfiguration::FailureMode_t failureMode);
    static MockLink* _startMockLink(MockConfiguration* mockConfig);

    MockConfiguration*          _mockConfig     = nullptr;
    MockVehicle*                _vehicle1       = nullptr;
    MockVehicle*                _vehicle2       = nullptr;
    QString                     _name;
    bool                        _connected      = false;
    int                         _mavlinkChannel = 0;
    bool                        _commLost       = false;

    RequestMessageFailureMode_t _requestMessageFailureMode = FailRequestMessageNone;

    static int          _nextVehicleSystemId;
};

