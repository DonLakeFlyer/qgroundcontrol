/****************************************************************************
 *
 * (c) 2009-2020 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "MockLink.h"
#include "MockVehicle.h"
#include "QGCLoggingCategory.h"
#include "QGCApplication.h"
#include "LinkManager.h"

#ifdef UNITTEST_BUILD
#include "UnitTest.h"
#endif

#include <QTimer>
#include <QDebug>
#include <QFile>

#include <string.h>

int MockLink::_nextVehicleSystemId =        128;

MockLink::MockLink(SharedLinkConfigurationPtr& config)
    : LinkInterface (config)
    , _mockConfig   (qobject_cast<MockConfiguration*>(config.get()))
    , _name         ("MockLink")
{
    qDebug() << "MockLink" << this;

    moveToThread(this);

    QObject::connect(this, &MockLink::writeBytesQueuedSignal, this, &MockLink::_writeBytesQueued, Qt::QueuedConnection);
}

MockLink::~MockLink(void)
{
    disconnect();
    qDebug() << "~MockLink" << this;
}

bool MockLink::_connect(void)
{
    if (!_connected) {
        _connected = true;
        // MockLinks use Mavlink 2.0
        mavlink_status_t* mavlinkStatus = mavlink_get_channel_status(_mavlinkChannel);
        mavlinkStatus->flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
        start();
        emit connected();
    }

    return true;
}

void MockLink::disconnect(void)
{
    if (_connected) {
        _connected = false;
        quit();
        wait();
        emit disconnected();
    }
}

void MockLink::run(void)
{
    MockConfiguration* mockConfig = qobject_cast<MockConfiguration*>(linkConfiguration().get());

    int vehicleId = _nextVehicleSystemId;
    if (mockConfig->incrementVehicleId()) {
        _nextVehicleSystemId++;
    }

     _vehicle1 = new MockVehicle(this, vehicleId);
    if (mockConfig->twoVehicles()) {
        _vehicle2 = new MockVehicle(this, vehicleId+1);
    }

    exec();

    delete _vehicle1;
    delete _vehicle2;
}

void MockLink::respondWithMavlinkMessage(const mavlink_message_t& msg)
{
    if (!_commLost) {
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

        int cBuffer = mavlink_msg_to_send_buffer(buffer, &msg);
        QByteArray bytes((char *)buffer, cBuffer);
        emit bytesReceived(this, bytes);
    }
}

/// @brief Called when QGC wants to write bytes to the MAV
void MockLink::_writeBytes(const QByteArray bytes)
{
    // This prevents the responses to mavlink messages from being sent until the _writeBytes returns.
    emit writeBytesQueuedSignal(bytes);
}

void MockLink::_writeBytesQueued(const QByteArray bytes)
{
    mavlink_message_t   msg;
    mavlink_status_t    status;

    for (qint64 i=0; i<bytes.count(); i++) {
        if (!mavlink_parse_char(_mavlinkChannel, bytes.at(i), &msg, &status)) {
            continue;
        }

        _vehicle1->mavlinkMessageReceived(msg);
        if (_vehicle2) {
            _vehicle2->mavlinkMessageReceived(msg);
        }
    }
}

MockLink* MockLink::_startMockLink(MockConfiguration* mockConfig)
{
    LinkManager* linkMgr = qgcApp()->toolbox()->linkManager();

    mockConfig->setDynamic(true);
    SharedLinkConfigurationPtr config = linkMgr->addConfiguration(mockConfig);

    if (linkMgr->createConnectedLink(config)) {
        return qobject_cast<MockLink*>(config->link());
    } else {
        return nullptr;
    }
}

MockLink* MockLink::_startMockLinkWorker(QString configName, MAV_AUTOPILOT firmwareType, MAV_TYPE vehicleType, bool sendStatusText, MockConfiguration::FailureMode_t failureMode)
{
    MockConfiguration* mockConfig = new MockConfiguration(configName);

    mockConfig->setFirmwareType(firmwareType);
    mockConfig->setVehicleType(vehicleType);
    mockConfig->setSendStatusText(sendStatusText);
    mockConfig->setFailureMode(failureMode);

    return _startMockLink(mockConfig);
}

MockLink*  MockLink::startPX4MockLink(bool sendStatusText, MockConfiguration::FailureMode_t failureMode)
{
    return _startMockLinkWorker("PX4 MultiRotor MockLink", MAV_AUTOPILOT_PX4, MAV_TYPE_QUADROTOR, sendStatusText, failureMode);
}

MockLink*  MockLink::startGenericMockLink(bool sendStatusText, MockConfiguration::FailureMode_t failureMode)
{
    return _startMockLinkWorker("Generic MockLink", MAV_AUTOPILOT_GENERIC, MAV_TYPE_QUADROTOR, sendStatusText, failureMode);
}

MockLink* MockLink::startNoInitialConnectMockLink(bool sendStatusText, MockConfiguration::FailureMode_t failureMode)
{
    return _startMockLinkWorker("No Initial Connect MockLink", MAV_AUTOPILOT_PX4, MAV_TYPE_GENERIC, sendStatusText, failureMode);
}

MockLink*  MockLink::startAPMArduCopterMockLink(bool sendStatusText, MockConfiguration::FailureMode_t failureMode)
{
    return _startMockLinkWorker("ArduCopter MockLink",MAV_AUTOPILOT_ARDUPILOTMEGA, MAV_TYPE_QUADROTOR, sendStatusText, failureMode);
}

MockLink*  MockLink::startAPMArduPlaneMockLink(bool sendStatusText, MockConfiguration::FailureMode_t failureMode)
{
    return _startMockLinkWorker("ArduPlane MockLink", MAV_AUTOPILOT_ARDUPILOTMEGA, MAV_TYPE_FIXED_WING, sendStatusText, failureMode);
}

MockLink*  MockLink::startAPMArduSubMockLink(bool sendStatusText, MockConfiguration::FailureMode_t failureMode)
{
    return _startMockLinkWorker("ArduSub MockLink", MAV_AUTOPILOT_ARDUPILOTMEGA, MAV_TYPE_SUBMARINE, sendStatusText, failureMode);
}

MockLink*  MockLink::startAPMArduRoverMockLink(bool sendStatusText, MockConfiguration::FailureMode_t failureMode)
{
    return _startMockLinkWorker("ArduRover MockLink", MAV_AUTOPILOT_ARDUPILOTMEGA, MAV_TYPE_GROUND_ROVER, sendStatusText, failureMode);
}

void MockLink::setAPMMissionResponseMode(bool sendHomePositionOnEmptyList)
{
    _vehicle1->setAPMMissionResponseMode(sendHomePositionOnEmptyList);
}
void MockLink::sendUnexpectedMissionAck(MAV_MISSION_RESULT ackType)
{
    _vehicle1->_missionItemHandler.sendUnexpectedMissionAck(ackType);
}

void MockLink::sendUnexpectedMissionItem(void)
{
    _vehicle1->_missionItemHandler.sendUnexpectedMissionItem();
}

void MockLink::sendUnexpectedMissionRequest(void)
{
    _vehicle1->_missionItemHandler.sendUnexpectedMissionRequest();
}

void MockLink::resetMissionItemHandler(void)
{
    _vehicle1->_missionItemHandler.reset();
}

QString MockLink::logDownloadFile(void)
{
    return _vehicle1->_logDownloadFilename;
}

void MockLink::simulateConnectionRemoved(void)
{
    _commLost = true;
    _connectionRemoved();
}
