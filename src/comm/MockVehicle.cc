/****************************************************************************
 *
 * (c) 2009-2020 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "MockVehicle.h"
#include "MockLink.h"
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

// FIXME: Hack to work around clean headers
#include "FirmwarePlugin/PX4/px4_custom_mode.h"

QGC_LOGGING_CATEGORY(MockLinkLog, "MockLinkLog")
QGC_LOGGING_CATEGORY(MockLinkVerboseLog, "MockLinkVerboseLog")

// Vehicle position is set close to default Gazebo vehicle location. This allows for multi-vehicle
// testing of a gazebo vehicle and a mocklink vehicle
#if 1
double      MockVehicle::_defaultVehicleLatitude =     47.397;
double      MockVehicle::_defaultVehicleLongitude =    8.5455;
double      MockVehicle::_defaultVehicleAltitude =     488.056;
#else
double      MockVehicle::_defaultVehicleLatitude =     47.6333022928789;
double      MockVehicle::_defaultVehicleLongitude =    -122.08833157994995;
double      MockVehicle::_defaultVehicleAltitude =     19.0;
#endif
const char* MockVehicle::_failParam =                  "COM_FLTMODE6";

MockVehicle::MockVehicle(MockLink* mockLink, int vehicleId)
    : QObject               (mockLink)
    , _mockLink             (mockLink)
    , _mockConfig           (mockLink->mockConfig())
    , _vehicleId            (vehicleId)
    , _vehicleAltitude      (_defaultVehicleAltitude)
    , _missionItemHandler   (mockLink, vehicleId, qgcApp()->toolbox()->mavlinkProtocol()->getSystemId(), qgcApp()->toolbox()->mavlinkProtocol()->getComponentId())
    , _mockLinkFTP          (mockLink, vehicleId)
{
    qDebug() << "MockVehicle" << this;

    _firmwareType       = _mockConfig->firmwareType();
    _vehicleType        = _mockConfig->vehicleType();
    _sendStatusText     = _mockConfig->sendStatusText();
    _failureMode        = _mockConfig->failureMode();
    _vehicleLatitude    = _defaultVehicleLatitude + ((_vehicleId - 128) * 0.0001);
    _vehicleLongitude   = _defaultVehicleLongitude + ((_vehicleId - 128) * 0.0001);

    union px4_custom_mode   px4_cm;
    px4_cm.data = 0;
    px4_cm.main_mode = PX4_CUSTOM_MAIN_MODE_MANUAL;
    _mavCustomMode = px4_cm.data;

    _loadParams();

    _adsbVehicleCoordinate = QGeoCoordinate(_vehicleLatitude, _vehicleLongitude).atDistanceAndAzimuth(1000, _adsbAngle);
    _adsbVehicleCoordinate.setAltitude(100);
    _runningTime.start();

    QObject::connect(&_timer1HzTasks,   &QTimer::timeout, this, &MockVehicle::_run1HzTasks);
    QObject::connect(&_timer10HzTasks,  &QTimer::timeout, this, &MockVehicle::_run10HzTasks);
    QObject::connect(&_timer500HzTasks, &QTimer::timeout, this, &MockVehicle::_run500HzTasks);

    _timer1HzTasks.start(1000);
    _timer10HzTasks.start(100);
    _timer500HzTasks.start(2);

    // Send first set right away
    _run1HzTasks();
    _run10HzTasks();
    _run500HzTasks();

}

MockVehicle::~MockVehicle(void)
{
    if (!_logDownloadFilename.isEmpty()) {
        QFile::remove(_logDownloadFilename);
    }
    _missionItemHandler.shutdown();
}

void MockVehicle::_run1HzTasks(void)
{
    if (_mockConfig->isHighLatency() && _highLatencyTransmissionEnabled) {
        _sendHighLatency2();
    } else {
        _sendVibration();
        _sendBatteryStatus();
        _sendSysStatus();
        _sendADSBVehicles();
        if (!qgcApp()->runningUnitTests()) {
            // Sending RC Channels during unit test breaks RC tests which does it's own RC simulation
            _sendRCChannels();
        }
        if (_sendHomePositionDelayCount > 0) {
            // We delay home position for better testing
            _sendHomePositionDelayCount--;
        } else {
            _sendHomePosition();
        }
        if (_sendStatusText) {
            _sendStatusText = false;
            _sendStatusTextMessages();
        }
    }
}

void MockVehicle::_run10HzTasks(void)
{
    if (_mockConfig->isHighLatency()) {
        return;
    }

    _sendHeartBeat();
    if (_sendGPSPositionDelayCount > 0) {
        // We delay gps position for better testing
        _sendGPSPositionDelayCount--;
    } else {
        _sendGpsRawInt();
    }
}

void MockVehicle::_run500HzTasks(void)
{
    if (_mockConfig->isHighLatency()) {
        return;
    }

    _paramRequestListWorker();
    _logDownloadWorker();
}

void MockVehicle::_loadParams(void)
{
    QFile paramFile;

    if (_firmwareType == MAV_AUTOPILOT_ARDUPILOTMEGA) {
        if (_vehicleType == MAV_TYPE_FIXED_WING) {
            paramFile.setFileName(":/FirmwarePlugin/APM/Plane.OfflineEditing.params");
        } else if (_vehicleType == MAV_TYPE_SUBMARINE ) {
            paramFile.setFileName(":/MockVehicle/APMArduSubMockLink.params");
        } else if (_vehicleType == MAV_TYPE_GROUND_ROVER ) {
            paramFile.setFileName(":/FirmwarePlugin/APM/Rover.OfflineEditing.params");
        } else {
            paramFile.setFileName(":/FirmwarePlugin/APM/Copter.OfflineEditing.params");
        }
    } else {
        paramFile.setFileName(":/MockVehicle/PX4MockLink.params");
    }


    bool success = paramFile.open(QFile::ReadOnly);
    Q_UNUSED(success);
    Q_ASSERT(success);

    QTextStream paramStream(&paramFile);

    while (!paramStream.atEnd()) {
        QString line = paramStream.readLine();

        if (line.startsWith("#")) {
            continue;
        }

        QStringList paramData = line.split("\t");
        Q_ASSERT(paramData.count() == 5);

        int compId = paramData.at(1).toInt();
        QString paramName = paramData.at(2);
        QString valStr = paramData.at(3);
        uint paramType = paramData.at(4).toUInt();

        QVariant paramValue;
        switch (paramType) {
        case MAV_PARAM_TYPE_REAL32:
            paramValue = QVariant(valStr.toFloat());
            break;
        case MAV_PARAM_TYPE_UINT32:
            paramValue = QVariant(valStr.toUInt());
            break;
        case MAV_PARAM_TYPE_INT32:
            paramValue = QVariant(valStr.toInt());
            break;
        case MAV_PARAM_TYPE_UINT16:
            paramValue = QVariant((quint16)valStr.toUInt());
            break;
        case MAV_PARAM_TYPE_INT16:
            paramValue = QVariant((qint16)valStr.toInt());
            break;
        case MAV_PARAM_TYPE_UINT8:
            paramValue = QVariant((quint8)valStr.toUInt());
            break;
        case MAV_PARAM_TYPE_INT8:
            paramValue = QVariant((qint8)valStr.toUInt());
            break;
        default:
            qCritical() << "Unknown type" << paramType;
            paramValue = QVariant(valStr.toInt());
            break;
        }

        qCDebug(MockLinkVerboseLog) << "Loading param" << paramName << paramValue;

        _mapParamName2Value[compId][paramName] = paramValue;
        _mapParamName2MavParamType[compId][paramName] = static_cast<MAV_PARAM_TYPE>(paramType);
    }
}

void MockVehicle::_sendHeartBeat(void)
{
    mavlink_message_t   msg;

    mavlink_msg_heartbeat_pack_chan(_vehicleId,
                                    MAV_COMP_ID_AUTOPILOT1,
                                    _mockLink->mavlinkChannel(),
                                    &msg,
                                    _vehicleType,        // MAV_TYPE
                                    _firmwareType,      // MAV_AUTOPILOT
                                    _mavBaseMode,        // MAV_MODE
                                    _mavCustomMode,      // custom mode
                                    _mavState);          // MAV_STATE

    respondWithMavlinkMessage(msg);
}

void MockVehicle::_sendHighLatency2(void)
{
    mavlink_message_t   msg;

    union px4_custom_mode   px4_cm;
    px4_cm.data = _mavCustomMode;

    qDebug() << "Sending" << _mavCustomMode;
    mavlink_msg_high_latency2_pack_chan(_vehicleId,
                                        MAV_COMP_ID_AUTOPILOT1,
                                        _mockLink->mavlinkChannel(),
                                        &msg,
                                        0,                          // timestamp
                                        _vehicleType,               // MAV_TYPE
                                        _firmwareType,              // MAV_AUTOPILOT
                                        px4_cm.custom_mode_hl,      // custom_mode
                                        (int32_t)(_vehicleLatitude  * 1E7),
                                        (int32_t)(_vehicleLongitude * 1E7),
                                        (int16_t)_vehicleAltitude,
                                        (int16_t)_vehicleAltitude,  // target_altitude,
                                        0,                          // heading
                                        0,                          // target_heading
                                        0,                          // target_distance
                                        0,                          // throttle
                                        0,                          // airspeed
                                        0,                          // airspeed_sp
                                        0,                          // groundspeed
                                        0,                          // windspeed,
                                        0,                          // wind_heading
                                        UINT8_MAX,                  // eph not known
                                        UINT8_MAX,                  // epv not known
                                        0,                          // temperature_air
                                        0,                          // climb_rate
                                        -1,                         // battery, do not use?
                                        0,                          // wp_num
                                        0,                          // failure_flags
                                        0, 0, 0);                   // custom0, custom1, custom2
    respondWithMavlinkMessage(msg);
}

void MockVehicle::_sendSysStatus(void)
{
    mavlink_message_t   msg;
    mavlink_msg_sys_status_pack_chan(
                _vehicleId,
                MAV_COMP_ID_AUTOPILOT1,
                static_cast<uint8_t>(_mockLink->mavlinkChannel()),
                &msg,
                0,          // onboard_control_sensors_present
                0,          // onboard_control_sensors_enabled
                0,          // onboard_control_sensors_health
                250,        // load
                4200 * 4,   // voltage_battery
                8000,       // current_battery
                _battery1PctRemaining, // battery_remaining
                0,0,0,0,0,0);
    respondWithMavlinkMessage(msg);
}

void MockVehicle::_sendBatteryStatus(void)
{
    if(_battery1PctRemaining > 1) {
        _battery1PctRemaining = static_cast<int8_t>(100 - (_runningTime.elapsed() / 1000));
        _battery1TimeRemaining = static_cast<double>(_batteryMaxTimeRemaining) * (static_cast<double>(_battery1PctRemaining) / 100.0);
        if (_battery1PctRemaining > 50) {
            _battery1ChargeState = MAV_BATTERY_CHARGE_STATE_OK;
        } else if (_battery1PctRemaining > 30) {
            _battery1ChargeState = MAV_BATTERY_CHARGE_STATE_LOW;
        } else if (_battery1PctRemaining > 20) {
            _battery1ChargeState = MAV_BATTERY_CHARGE_STATE_CRITICAL;
        } else {
            _battery1ChargeState = MAV_BATTERY_CHARGE_STATE_EMERGENCY;
        }
    }
    if(_battery2PctRemaining > 1) {
        _battery2PctRemaining = static_cast<int8_t>(100 - ((_runningTime.elapsed() / 1000) / 2));
        _battery2TimeRemaining = static_cast<double>(_batteryMaxTimeRemaining) * (static_cast<double>(_battery2PctRemaining) / 100.0);
        if (_battery2PctRemaining > 50) {
            _battery2ChargeState = MAV_BATTERY_CHARGE_STATE_OK;
        } else if (_battery2PctRemaining > 30) {
            _battery2ChargeState = MAV_BATTERY_CHARGE_STATE_LOW;
        } else if (_battery2PctRemaining > 20) {
            _battery2ChargeState = MAV_BATTERY_CHARGE_STATE_CRITICAL;
        } else {
            _battery2ChargeState = MAV_BATTERY_CHARGE_STATE_EMERGENCY;
        }
    }

    mavlink_message_t   msg;
    uint16_t            rgVoltages[10];
    uint16_t            rgVoltagesNone[10];
    uint16_t            rgVoltagesExtNone[4];

    for (int i=0; i<10; i++) {
        rgVoltages[i]       = UINT16_MAX;
        rgVoltagesNone[i]   = UINT16_MAX;
    }
    rgVoltages[0] = rgVoltages[1] = rgVoltages[2] = 4200;
    memset(rgVoltagesExtNone, 0, sizeof(rgVoltagesExtNone));

    mavlink_msg_battery_status_pack_chan(
                _vehicleId,
                MAV_COMP_ID_AUTOPILOT1,
                static_cast<uint8_t>(_mockLink->mavlinkChannel()),
                &msg,
                1,                          // battery id
                MAV_BATTERY_FUNCTION_ALL,
                MAV_BATTERY_TYPE_LIPO,
                2100,                       // temp cdegC
                rgVoltages,
                600,                        // battery cA
                100,                        // current consumed mAh
                -1,                         // energy consumed not supported
                _battery1PctRemaining,
                _battery1TimeRemaining,
                _battery1ChargeState,
                rgVoltagesExtNone);
    respondWithMavlinkMessage(msg);

    mavlink_msg_battery_status_pack_chan(
                _vehicleId,
                MAV_COMP_ID_AUTOPILOT1,
                static_cast<uint8_t>(_mockLink->mavlinkChannel()),
                &msg,
                2,                          // battery id
                MAV_BATTERY_FUNCTION_ALL,
                MAV_BATTERY_TYPE_LIPO,
                INT16_MAX,                  // temp cdegC
                rgVoltagesNone,
                600,                        // battery cA
                100,                        // current consumed mAh
                -1,                         // energy consumed not supported
                _battery2PctRemaining,
                _battery2TimeRemaining,
                _battery2ChargeState,
                rgVoltagesExtNone);
    respondWithMavlinkMessage(msg);
}

void MockVehicle::_sendVibration(void)
{
    mavlink_message_t   msg;

    mavlink_msg_vibration_pack_chan(_vehicleId,
                                    MAV_COMP_ID_AUTOPILOT1,
                                    _mockLink->mavlinkChannel(),
                                    &msg,
                                    0,       // time_usec
                                    50.5,    // vibration_x,
                                    10.5,    // vibration_y,
                                    60.0,    // vibration_z,
                                    1,       // clipping_0
                                    2,       // clipping_0
                                    3);      // clipping_0

    respondWithMavlinkMessage(msg);
}

/// @brief Handle incoming bytes which are meant to be handled by the mavlink protocol
void MockVehicle::mavlinkMessageReceived(const mavlink_message_t& msg)
{
    if (_missionItemHandler.handleMessage(msg)) {
        return;
    }

    switch (msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
        _handleHeartBeat(msg);
        break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        _handleParamRequestList(msg);
        break;
    case MAVLINK_MSG_ID_SET_MODE:
        _handleSetMode(msg);
        break;
    case MAVLINK_MSG_ID_PARAM_SET:
        _handleParamSet(msg);
        break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        _handleParamRequestRead(msg);
        break;
    case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL:
        _handleFTP(msg);
        break;
    case MAVLINK_MSG_ID_COMMAND_LONG:
        _handleCommandLong(msg);
        break;
    case MAVLINK_MSG_ID_MANUAL_CONTROL:
        _handleManualControl(msg);
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
        _handleLogRequestList(msg);
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
        _handleLogRequestData(msg);
        break;
    case MAVLINK_MSG_ID_PARAM_MAP_RC:
        _handleParamMapRC(msg);
        break;
    default:
        break;
    }
}

void MockVehicle::_handleHeartBeat(const mavlink_message_t& msg)
{
    Q_UNUSED(msg);
    qCDebug(MockLinkLog) << "Heartbeat";
}

void MockVehicle::_handleParamMapRC(const mavlink_message_t& msg)
{
    mavlink_param_map_rc_t paramMapRC;
    mavlink_msg_param_map_rc_decode(&msg, &paramMapRC);

    const QString paramName(QString::fromLocal8Bit(paramMapRC.param_id, static_cast<int>(strnlen(paramMapRC.param_id, MAVLINK_MSG_PARAM_MAP_RC_FIELD_PARAM_ID_LEN))));

    if (paramMapRC.param_index == -1) {
        qDebug() << QStringLiteral("MockVehicle - PARAM_MAP_RC: param(%1) tuningID(%2) centerValue(%3) scale(%4) min(%5) max(%6)").arg(paramName).arg(paramMapRC.parameter_rc_channel_index).arg(paramMapRC.param_value0).arg(paramMapRC.scale).arg(paramMapRC.param_value_min).arg(paramMapRC.param_value_max);
    } else if (paramMapRC.param_index == -2) {
        qDebug() << QStringLiteral("MockVehicle - PARAM_MAP_RC: Clear tuningID(%1)").arg(paramMapRC.parameter_rc_channel_index);
    } else {
        qWarning() << QStringLiteral("MockVehicle - PARAM_MAP_RC: Unsupported param_index(%1)").arg(paramMapRC.param_index);
    }
}

void MockVehicle::_handleSetMode(const mavlink_message_t& msg)
{
    mavlink_set_mode_t request;
    mavlink_msg_set_mode_decode(&msg, &request);

    Q_ASSERT(request.target_system == _vehicleId);

    _mavBaseMode = request.base_mode;
    _mavCustomMode = request.custom_mode;
}

void MockVehicle::_handleManualControl(const mavlink_message_t& msg)
{
    mavlink_manual_control_t manualControl;
    mavlink_msg_manual_control_decode(&msg, &manualControl);

    qCDebug(MockLinkLog) << "MANUAL_CONTROL" << manualControl.x << manualControl.y << manualControl.z << manualControl.r;
}

void MockVehicle::_setParamFloatUnionIntoMap(int componentId, const QString& paramName, float paramFloat)
{
    mavlink_param_union_t   valueUnion;

    Q_ASSERT(_mapParamName2Value.contains(componentId));
    Q_ASSERT(_mapParamName2Value[componentId].contains(paramName));
    Q_ASSERT(_mapParamName2MavParamType[componentId].contains(paramName));

    valueUnion.param_float = paramFloat;

    MAV_PARAM_TYPE paramType = _mapParamName2MavParamType[componentId][paramName];

    QVariant paramVariant;

    switch (paramType) {
    case MAV_PARAM_TYPE_REAL32:
        paramVariant = QVariant::fromValue(valueUnion.param_float);
        break;

    case MAV_PARAM_TYPE_UINT32:
        paramVariant = QVariant::fromValue(valueUnion.param_uint32);
        break;

    case MAV_PARAM_TYPE_INT32:
        paramVariant = QVariant::fromValue(valueUnion.param_int32);
        break;

    case MAV_PARAM_TYPE_UINT16:
        paramVariant = QVariant::fromValue(valueUnion.param_uint16);
        break;

    case MAV_PARAM_TYPE_INT16:
        paramVariant = QVariant::fromValue(valueUnion.param_int16);
        break;

    case MAV_PARAM_TYPE_UINT8:
        paramVariant = QVariant::fromValue(valueUnion.param_uint8);
        break;

    case MAV_PARAM_TYPE_INT8:
        paramVariant = QVariant::fromValue(valueUnion.param_int8);
        break;

    default:
        qCritical() << "Invalid parameter type" << paramType;
        paramVariant = QVariant::fromValue(valueUnion.param_int32);
        break;
    }

    qCDebug(MockLinkLog) << "_setParamFloatUnionIntoMap" << paramName << paramVariant;
    _mapParamName2Value[componentId][paramName] = paramVariant;
}

/// Convert from a parameter variant to the float value from mavlink_param_union_t
float MockVehicle::_floatUnionForParam(int componentId, const QString& paramName)
{
    mavlink_param_union_t   valueUnion;

    Q_ASSERT(_mapParamName2Value.contains(componentId));
    Q_ASSERT(_mapParamName2Value[componentId].contains(paramName));
    Q_ASSERT(_mapParamName2MavParamType[componentId].contains(paramName));

    MAV_PARAM_TYPE paramType = _mapParamName2MavParamType[componentId][paramName];
    QVariant paramVar = _mapParamName2Value[componentId][paramName];

    switch (paramType) {
    case MAV_PARAM_TYPE_REAL32:
        valueUnion.param_float = paramVar.toFloat();
        break;

    case MAV_PARAM_TYPE_UINT32:
        if (_firmwareType == MAV_AUTOPILOT_ARDUPILOTMEGA) {
            valueUnion.param_float = paramVar.toUInt();
        } else {
            valueUnion.param_uint32 = paramVar.toUInt();
        }
        break;

    case MAV_PARAM_TYPE_INT32:
        if (_firmwareType == MAV_AUTOPILOT_ARDUPILOTMEGA) {
            valueUnion.param_float = paramVar.toInt();
        } else {
            valueUnion.param_int32 = paramVar.toInt();
        }
        break;

    case MAV_PARAM_TYPE_UINT16:
        if (_firmwareType == MAV_AUTOPILOT_ARDUPILOTMEGA) {
            valueUnion.param_float = paramVar.toUInt();
        } else {
            valueUnion.param_uint16 = paramVar.toUInt();
        }
        break;

    case MAV_PARAM_TYPE_INT16:
        if (_firmwareType == MAV_AUTOPILOT_ARDUPILOTMEGA) {
            valueUnion.param_float = paramVar.toInt();
        } else {
            valueUnion.param_int16 = paramVar.toInt();
        }
        break;

    case MAV_PARAM_TYPE_UINT8:
        if (_firmwareType == MAV_AUTOPILOT_ARDUPILOTMEGA) {
            valueUnion.param_float = paramVar.toUInt();
        } else {
            valueUnion.param_uint8 = paramVar.toUInt();
        }
        break;

    case MAV_PARAM_TYPE_INT8:
        if (_firmwareType == MAV_AUTOPILOT_ARDUPILOTMEGA) {
            valueUnion.param_float = (unsigned char)paramVar.toChar().toLatin1();
        } else {
            valueUnion.param_int8 = (unsigned char)paramVar.toChar().toLatin1();
        }
        break;

    default:
        if (_firmwareType == MAV_AUTOPILOT_ARDUPILOTMEGA) {
            valueUnion.param_float = paramVar.toInt();
        } else {
            valueUnion.param_int32 = paramVar.toInt();
        }
        qCritical() << "Invalid parameter type" << paramType;
    }

    return valueUnion.param_float;
}

void MockVehicle::_handleParamRequestList(const mavlink_message_t& msg)
{
    if (_failureMode == MockConfiguration::FailParamNoReponseToRequestList) {
        return;
    }

    mavlink_param_request_list_t request;

    mavlink_msg_param_request_list_decode(&msg, &request);

    Q_ASSERT(request.target_system == _vehicleId);
    Q_ASSERT(request.target_component == MAV_COMP_ID_ALL);

    // Start the worker routine
    _currentParamRequestListComponentIndex = 0;
    _currentParamRequestListParamIndex = 0;
}

/// Sends the next parameter to the vehicle
void MockVehicle::_paramRequestListWorker(void)
{
    if (_currentParamRequestListComponentIndex == -1) {
        // Initial request complete
        return;
    }

    int componentId = _mapParamName2Value.keys()[_currentParamRequestListComponentIndex];
    int cParameters = _mapParamName2Value[componentId].count();
    QString paramName = _mapParamName2Value[componentId].keys()[_currentParamRequestListParamIndex];

    if ((_failureMode == MockConfiguration::FailMissingParamOnInitialReqest || _failureMode == MockConfiguration::FailMissingParamOnAllRequests) && paramName == _failParam) {
        qCDebug(MockLinkLog) << "Skipping param send:" << paramName;
    } else {

        char paramId[MAVLINK_MSG_ID_PARAM_VALUE_LEN];
        mavlink_message_t responseMsg;

        Q_ASSERT(_mapParamName2Value[componentId].contains(paramName));
        Q_ASSERT(_mapParamName2MavParamType[componentId].contains(paramName));

        MAV_PARAM_TYPE paramType = _mapParamName2MavParamType[componentId][paramName];

        Q_ASSERT(paramName.length() <= MAVLINK_MSG_ID_PARAM_VALUE_LEN);
        strncpy(paramId, paramName.toLocal8Bit().constData(), MAVLINK_MSG_ID_PARAM_VALUE_LEN);

        qCDebug(MockLinkLog) << "Sending msg_param_value" << componentId << paramId << paramType << _mapParamName2Value[componentId][paramId];

        mavlink_msg_param_value_pack_chan(_vehicleId,
                                          componentId,                                   // component id
                                          _mockLink->mavlinkChannel(),
                                          &responseMsg,                                  // Outgoing message
                                          paramId,                                       // Parameter name
                                          _floatUnionForParam(componentId, paramName),   // Parameter value
                                          paramType,                                     // MAV_PARAM_TYPE
                                          cParameters,                                   // Total number of parameters
                                          _currentParamRequestListParamIndex);           // Index of this parameter
        respondWithMavlinkMessage(responseMsg);
    }

    // Move to next param index
    if (++_currentParamRequestListParamIndex >= cParameters) {
        // We've sent the last parameter for this component, move to next component
        if (++_currentParamRequestListComponentIndex >= _mapParamName2Value.keys().count()) {
            // We've finished sending the last parameter for the last component, request is complete
            _currentParamRequestListComponentIndex = -1;
        } else {
            _currentParamRequestListParamIndex = 0;
        }
    }
}

void MockVehicle::_handleParamSet(const mavlink_message_t& msg)
{
    mavlink_param_set_t request;
    mavlink_msg_param_set_decode(&msg, &request);

    Q_ASSERT(request.target_system == _vehicleId);
    int componentId = request.target_component;

    // Param may not be null terminated if exactly fits
    char paramId[MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN + 1];
    paramId[MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN] = 0;
    strncpy(paramId, request.param_id, MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN);

    qCDebug(MockLinkLog) << "_handleParamSet" << componentId << paramId << request.param_type;

    Q_ASSERT(_mapParamName2Value.contains(componentId));
    Q_ASSERT(_mapParamName2MavParamType.contains(componentId));
    Q_ASSERT(_mapParamName2Value[componentId].contains(paramId));
    Q_ASSERT(request.param_type == _mapParamName2MavParamType[componentId][paramId]);

    // Save the new value
    _setParamFloatUnionIntoMap(componentId, paramId, request.param_value);

    // Respond with a param_value to ack
    mavlink_message_t responseMsg;
    mavlink_msg_param_value_pack_chan(_vehicleId,
                                      componentId,                                               // component id
                                      _mockLink->mavlinkChannel(),
                                      &responseMsg,                                              // Outgoing message
                                      paramId,                                                   // Parameter name
                                      request.param_value,                                       // Send same value back
                                      request.param_type,                                        // Send same type back
                                      _mapParamName2Value[componentId].count(),                  // Total number of parameters
                                      _mapParamName2Value[componentId].keys().indexOf(paramId)); // Index of this parameter
    respondWithMavlinkMessage(responseMsg);
}

void MockVehicle::_handleParamRequestRead(const mavlink_message_t& msg)
{
    mavlink_message_t   responseMsg;
    mavlink_param_request_read_t request;
    mavlink_msg_param_request_read_decode(&msg, &request);

    const QString paramName(QString::fromLocal8Bit(request.param_id, static_cast<int>(strnlen(request.param_id, MAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN))));
    int componentId = request.target_component;

    // special case for magic _HASH_CHECK value
    if (request.target_component == MAV_COMP_ID_ALL && paramName == "_HASH_CHECK") {
        mavlink_param_union_t   valueUnion;
        valueUnion.type = MAV_PARAM_TYPE_UINT32;
        valueUnion.param_uint32 = 0;
        // Special case of magic hash check value
        mavlink_msg_param_value_pack_chan(_vehicleId,
                                          componentId,
                                          _mockLink->mavlinkChannel(),
                                          &responseMsg,
                                          request.param_id,
                                          valueUnion.param_float,
                                          MAV_PARAM_TYPE_UINT32,
                                          0,
                                          -1);
        respondWithMavlinkMessage(responseMsg);
        return;
    }

    Q_ASSERT(_mapParamName2Value.contains(componentId));

    char paramId[MAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN + 1];
    paramId[0] = 0;

    Q_ASSERT(request.target_system == _vehicleId);

    if (request.param_index == -1) {
        // Request is by param name. Param may not be null terminated if exactly fits
        strncpy(paramId, request.param_id, MAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN);
    } else {
        // Request is by index

        Q_ASSERT(request.param_index >= 0 && request.param_index < _mapParamName2Value[componentId].count());

        QString key = _mapParamName2Value[componentId].keys().at(request.param_index);
        Q_ASSERT(key.length() <= MAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN);
        strcpy(paramId, key.toLocal8Bit().constData());
    }

    Q_ASSERT(_mapParamName2Value[componentId].contains(paramId));
    Q_ASSERT(_mapParamName2MavParamType[componentId].contains(paramId));

    if (_failureMode == MockConfiguration::FailMissingParamOnAllRequests && strcmp(paramId, _failParam) == 0) {
        qCDebug(MockLinkLog) << "Ignoring request read for " << _failParam;
        // Fail to send this param no matter what
        return;
    }

    mavlink_msg_param_value_pack_chan(_vehicleId,
                                      componentId,                                               // component id
                                      _mockLink->mavlinkChannel(),
                                      &responseMsg,                                              // Outgoing message
                                      paramId,                                                   // Parameter name
                                      _floatUnionForParam(componentId, paramId),                 // Parameter value
                                      _mapParamName2MavParamType[componentId][paramId],          // Parameter type
                                      _mapParamName2Value[componentId].count(),                  // Total number of parameters
                                      _mapParamName2Value[componentId].keys().indexOf(paramId)); // Index of this parameter
    respondWithMavlinkMessage(responseMsg);
}

void MockVehicle::emitRemoteControlChannelRawChanged(int channel, uint16_t raw)
{
    uint16_t chanRaw[18];

    for (int i=0; i<18; i++) {
        chanRaw[i] = UINT16_MAX;
    }
    chanRaw[channel] = raw;

    mavlink_message_t responseMsg;
    mavlink_msg_rc_channels_pack_chan(_vehicleId,
                                      MAV_COMP_ID_AUTOPILOT1,
                                      _mockLink->mavlinkChannel(),
                                      &responseMsg,          // Outgoing message
                                      0,                     // time since boot, ignored
                                      18,                    // channel count
                                      chanRaw[0],            // channel raw value
            chanRaw[1],            // channel raw value
            chanRaw[2],            // channel raw value
            chanRaw[3],            // channel raw value
            chanRaw[4],            // channel raw value
            chanRaw[5],            // channel raw value
            chanRaw[6],            // channel raw value
            chanRaw[7],            // channel raw value
            chanRaw[8],            // channel raw value
            chanRaw[9],            // channel raw value
            chanRaw[10],           // channel raw value
            chanRaw[11],           // channel raw value
            chanRaw[12],           // channel raw value
            chanRaw[13],           // channel raw value
            chanRaw[14],           // channel raw value
            chanRaw[15],           // channel raw value
            chanRaw[16],           // channel raw value
            chanRaw[17],           // channel raw value
            0);                    // rss
    respondWithMavlinkMessage(responseMsg);
}

void MockVehicle::_handleFTP(const mavlink_message_t& msg)
{
    _mockLinkFTP.mavlinkMessageReceived(msg);
}

void MockVehicle::_handleCommandLong(const mavlink_message_t& msg)
{
    static bool firstCmdUser3 = true;
    static bool firstCmdUser4 = true;

    bool noAck = false;
    mavlink_command_long_t request;
    uint8_t commandResult = MAV_RESULT_UNSUPPORTED;

    mavlink_msg_command_long_decode(&msg, &request);

    switch (request.command) {
    case MAV_CMD_COMPONENT_ARM_DISARM:
        if (request.param1 == 0.0f) {
            _mavBaseMode &= ~MAV_MODE_FLAG_SAFETY_ARMED;
        } else {
            _mavBaseMode |= MAV_MODE_FLAG_SAFETY_ARMED;
        }
        commandResult = MAV_RESULT_ACCEPTED;
        break;
    case MAV_CMD_PREFLIGHT_CALIBRATION:
        _handlePreFlightCalibration(request);
        commandResult = MAV_RESULT_ACCEPTED;
        break;
    case MAV_CMD_CONTROL_HIGH_LATENCY:
        if (_mockConfig->isHighLatency()) {
            _highLatencyTransmissionEnabled = static_cast<int>(request.param1) != 0;
            emit highLatencyTransmissionEnabledChanged(_highLatencyTransmissionEnabled);
            commandResult = MAV_RESULT_ACCEPTED;
        } else {
            commandResult = MAV_RESULT_FAILED;
        }
        break;
    case MAV_CMD_PREFLIGHT_STORAGE:
        commandResult = MAV_RESULT_ACCEPTED;
        break;
    case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
        commandResult = MAV_RESULT_ACCEPTED;
        _respondWithAutopilotVersion();
        break;
    case MAV_CMD_REQUEST_MESSAGE:
        if (_handleRequestMessage(request, noAck)) {
            if (noAck) {
                return;
            }
            commandResult = MAV_RESULT_ACCEPTED;
        }
        break;
    case MAV_CMD_MOCKLINK_ALWAYS_RESULT_ACCEPTED:
        // Test command which always returns MAV_RESULT_ACCEPTED
        commandResult = MAV_RESULT_ACCEPTED;
        break;
    case MAV_CMD_MOCKLINK_ALWAYS_RESULT_FAILED:
        // Test command which always returns MAV_RESULT_FAILED
        commandResult = MAV_RESULT_FAILED;
        break;
    case MAV_CMD_MOCKLINK_SECOND_ATTEMPT_RESULT_ACCEPTED:
        // Test command which returns MAV_RESULT_ACCEPTED on second attempt
        if (firstCmdUser3) {
            firstCmdUser3 = false;
            return;
        } else {
            firstCmdUser3 = true;
            commandResult = MAV_RESULT_ACCEPTED;
        }
        break;
    case MAV_CMD_MOCKLINK_SECOND_ATTEMPT_RESULT_FAILED:
        // Test command which returns MAV_RESULT_FAILED on second attempt
        if (firstCmdUser4) {
            firstCmdUser4 = false;
            return;
        } else {
            firstCmdUser4 = true;
            commandResult = MAV_RESULT_FAILED;
        }
        break;
    case MAV_CMD_MOCKLINK_NO_RESPONSE:
        // No response
        return;
        break;
    }

    mavlink_message_t commandAck;
    mavlink_msg_command_ack_pack_chan(_vehicleId,
                                      MAV_COMP_ID_AUTOPILOT1,
                                      _mockLink->mavlinkChannel(),
                                      &commandAck,
                                      request.command,
                                      commandResult,
                                      0,    // progress
                                      0,    // result_param2
                                      0,    // target_system
                                      0);   // target_component
    respondWithMavlinkMessage(commandAck);
}

void MockVehicle::sendUnexpectedCommandAck(MAV_CMD command, MAV_RESULT ackResult)
{
    mavlink_message_t commandAck;
    mavlink_msg_command_ack_pack_chan(_vehicleId,
                                      MAV_COMP_ID_AUTOPILOT1,
                                      _mockLink->mavlinkChannel(),
                                      &commandAck,
                                      command,
                                      ackResult,
                                      0,    // progress
                                      0,    // result_param2
                                      0,    // target_system
                                      0);   // target_component
    respondWithMavlinkMessage(commandAck);
}

void MockVehicle::_respondWithAutopilotVersion(void)
{
    mavlink_message_t msg;

    uint8_t customVersion[8] = { };
    uint32_t flightVersion = 0;
#if !defined(NO_ARDUPILOT_DIALECT)
    if (_firmwareType == MAV_AUTOPILOT_ARDUPILOTMEGA) {
        if (_vehicleType == MAV_TYPE_FIXED_WING) {
            flightVersion |= 9 << (8*2);
        } else if (_vehicleType == MAV_TYPE_SUBMARINE ) {
            flightVersion |= 5 << (8*2);
        } else if (_vehicleType == MAV_TYPE_GROUND_ROVER ) {
            flightVersion |= 5 << (8*2);
        } else {
            flightVersion |= 6 << (8*2);
        }
        flightVersion |= 3 << (8*3);    // Major
        flightVersion |= 0 << (8*1);    // Patch
        flightVersion |= FIRMWARE_VERSION_TYPE_DEV << (8*0);
    } else if (_firmwareType == MAV_AUTOPILOT_PX4) {
#endif
        flightVersion |= 1 << (8*3);
        flightVersion |= 4 << (8*2);
        flightVersion |= 1 << (8*1);
        flightVersion |= FIRMWARE_VERSION_TYPE_DEV << (8*0);
#if !defined(NO_ARDUPILOT_DIALECT)
    }
#endif
    mavlink_msg_autopilot_version_pack_chan(_vehicleId,
                                            MAV_COMP_ID_AUTOPILOT1,
                                            _mockLink->mavlinkChannel(),
                                            &msg,
                                            MAV_PROTOCOL_CAPABILITY_MAVLINK2 | MAV_PROTOCOL_CAPABILITY_MISSION_FENCE | MAV_PROTOCOL_CAPABILITY_MISSION_RALLY | (_firmwareType == MAV_AUTOPILOT_ARDUPILOTMEGA ? MAV_PROTOCOL_CAPABILITY_TERRAIN : 0),
                                            flightVersion,                   // flight_sw_version,
                                            0,                               // middleware_sw_version,
                                            0,                               // os_sw_version,
                                            0,                               // board_version,
                                            (uint8_t *)&customVersion,       // flight_custom_version,
                                            (uint8_t *)&customVersion,       // middleware_custom_version,
                                            (uint8_t *)&customVersion,       // os_custom_version,
                                            0,                               // vendor_id,
                                            0,                               // product_id,
                                            0,                               // uid
                                            0);                              // uid2
    respondWithMavlinkMessage(msg);
}

void MockVehicle::setMissionItemFailureMode(MockLinkMissionItemHandler::FailureMode_t failureMode, MAV_MISSION_RESULT failureAckResult)
{
    _missionItemHandler.setFailureMode(failureMode, failureAckResult);
}

void MockVehicle::_sendHomePosition(void)
{
    mavlink_message_t msg;

    float bogus[4];
    bogus[0] = 0.0f;
    bogus[1] = 0.0f;
    bogus[2] = 0.0f;
    bogus[3] = 0.0f;

    mavlink_msg_home_position_pack_chan(_vehicleId,
                                        MAV_COMP_ID_AUTOPILOT1,
                                        _mockLink->mavlinkChannel(),
                                        &msg,
                                        (int32_t)(_vehicleLatitude * 1E7),
                                        (int32_t)(_vehicleLongitude * 1E7),
                                        (int32_t)(_vehicleAltitude * 1000),
                                        0.0f, 0.0f, 0.0f,
                                        &bogus[0],
            0.0f, 0.0f, 0.0f,
            0);
    respondWithMavlinkMessage(msg);
}

void MockVehicle::_sendGpsRawInt(void)
{
    static uint64_t timeTick = 0;
    mavlink_message_t msg;

    mavlink_msg_gps_raw_int_pack_chan(_vehicleId,
                                      MAV_COMP_ID_AUTOPILOT1,
                                      _mockLink->mavlinkChannel(),
                                      &msg,
                                      timeTick++,                           // time since boot
                                      3,                                    // 3D fix
                                      (int32_t)(_vehicleLatitude  * 1E7),
                                      (int32_t)(_vehicleLongitude * 1E7),
                                      (int32_t)(_vehicleAltitude  * 1000),
                                      UINT16_MAX, UINT16_MAX,               // HDOP/VDOP not known
                                      UINT16_MAX,                           // velocity not known
                                      UINT16_MAX,                           // course over ground not known
                                      8,                                    // satellites visible
                                      //-- Extension
                                      0,                                    // Altitude (above WGS84, EGM96 ellipsoid), in meters * 1000 (positive for up).
                                      0,                                    // Position uncertainty in meters * 1000 (positive for up).
                                      0,                                    // Altitude uncertainty in meters * 1000 (positive for up).
                                      0,                                    // Speed uncertainty in meters * 1000 (positive for up).
                                      0,                                    // Heading / track uncertainty in degrees * 1e5.
                                      65535);                               // Yaw not provided
    respondWithMavlinkMessage(msg);
}

void MockVehicle::_sendChunkedStatusText(uint16_t chunkId, bool missingChunks)
{
    mavlink_message_t msg;

    int cChunks = 4;
    int num = 0;
    for (int i=0; i<cChunks; i++) {
        if (missingChunks && (i & 1)) {
            continue;
        }
        int cBuf = MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN;
        char msgBuf[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
        memset(msgBuf, 0, sizeof(msgBuf));
        if (i == cChunks - 1) {
            // Last chunk is partial
            cBuf /= 2;
        }
        for (int j=0; j<cBuf-1; j++) {
            msgBuf[j] = '0' + num++;
            if (num > 9) {
                num = 0;
            }
        }
        msgBuf[cBuf-1] = 'A' + i;

        mavlink_msg_statustext_pack_chan(_vehicleId,
                                         MAV_COMP_ID_AUTOPILOT1,
                                         _mockLink->mavlinkChannel(),
                                         &msg,
                                         MAV_SEVERITY_INFO,
                                         msgBuf,
                                         chunkId,
                                         i);                    // chunk sequence number
        respondWithMavlinkMessage(msg);
    }

}

void MockVehicle::_sendStatusTextMessages(void)
{
    struct StatusMessage {
        MAV_SEVERITY        severity;
        const char*         msg;
    };

    static const struct StatusMessage rgMessages[] = {
    { MAV_SEVERITY_INFO,        "#Testing audio output" },
    { MAV_SEVERITY_EMERGENCY,   "Status text emergency" },
    { MAV_SEVERITY_ALERT,       "Status text alert" },
    { MAV_SEVERITY_CRITICAL,    "Status text critical" },
    { MAV_SEVERITY_ERROR,       "Status text error" },
    { MAV_SEVERITY_WARNING,     "Status text warning" },
    { MAV_SEVERITY_NOTICE,      "Status text notice" },
    { MAV_SEVERITY_INFO,        "Status text info" },
    { MAV_SEVERITY_DEBUG,       "Status text debug" },
};

    mavlink_message_t msg;

    for (size_t i=0; i<sizeof(rgMessages)/sizeof(rgMessages[0]); i++) {
        const struct StatusMessage* status = &rgMessages[i];

        mavlink_msg_statustext_pack_chan(_vehicleId,
                                         MAV_COMP_ID_AUTOPILOT1,
                                         _mockLink->mavlinkChannel(),
                                         &msg,
                                         status->severity,
                                         status->msg,
                                         0,                     // Not a chunked sequence
                                         0);                    // Not a chunked sequence
        respondWithMavlinkMessage(msg);
    }

    _sendChunkedStatusText(1, false /* missingChunks */);
    _sendChunkedStatusText(2, true /* missingChunks */);
    _sendChunkedStatusText(3, false /* missingChunks */);   // This should cause the previous incomplete chunk to spit out
    _sendChunkedStatusText(4, true /* missingChunks */);    // This should cause the timeout to fire
}

void MockVehicle::_handlePreFlightCalibration(const mavlink_command_long_t& request)
{
    const char* pCalMessage;
    static const char* gyroCalResponse = "[cal] calibration started: 2 gyro";
    static const char* magCalResponse = "[cal] calibration started: 2 mag";
    static const char* accelCalResponse = "[cal] calibration started: 2 accel";

    if (request.param1 == 1) {
        // Gyro cal
        pCalMessage = gyroCalResponse;
    } else if (request.param2 == 1) {
        // Mag cal
        pCalMessage = magCalResponse;
    } else if (request.param5 == 1) {
        // Accel cal
        pCalMessage = accelCalResponse;
    } else {
        return;
    }

    mavlink_message_t msg;
    mavlink_msg_statustext_pack_chan(_vehicleId,
                                     MAV_COMP_ID_AUTOPILOT1,
                                     _mockLink->mavlinkChannel(),
                                     &msg,
                                     MAV_SEVERITY_INFO,
                                     pCalMessage,
                                     0, 0);                 // Not chunked
    respondWithMavlinkMessage(msg);
}

void MockVehicle::_handleLogRequestList(const mavlink_message_t& msg)
{
    mavlink_log_request_list_t request;

    mavlink_msg_log_request_list_decode(&msg, &request);

    if (request.start != 0 && request.end != 0xffff) {
        qWarning() << "MockVehicle::_handleLogRequestList cannot handle partial requests";
        return;
    }

    mavlink_message_t responseMsg;
    mavlink_msg_log_entry_pack_chan(_vehicleId,
                                    MAV_COMP_ID_AUTOPILOT1,
                                    _mockLink->mavlinkChannel(),
                                    &responseMsg,
                                    _logDownloadLogId,       // log id
                                    1,                       // num_logs
                                    1,                       // last_log_num
                                    0,                       // time_utc
                                    _logDownloadFileSize);   // size
    respondWithMavlinkMessage(responseMsg);
}

void MockVehicle::_handleLogRequestData(const mavlink_message_t& msg)
{
    mavlink_log_request_data_t request;

    mavlink_msg_log_request_data_decode(&msg, &request);

    if (_logDownloadFilename.isEmpty()) {
#ifdef UNITTEST_BUILD
        _logDownloadFilename = UnitTest::createRandomFile(_logDownloadFileSize);
#endif
    }

    if (request.id != 0) {
        qWarning() << "MockVehicle::_handleLogRequestData id must be 0";
        return;
    }

    if (request.ofs > _logDownloadFileSize - 1) {
        qWarning() << "MockVehicle::_handleLogRequestData offset past end of file request.ofs:size" << request.ofs << _logDownloadFileSize;
        return;
    }

    // This will trigger _logDownloadWorker to send data
    _logDownloadCurrentOffset = request.ofs;
    if (request.ofs + request.count > _logDownloadFileSize) {
        request.count = _logDownloadFileSize - request.ofs;
    }
    _logDownloadBytesRemaining = request.count;
}

void MockVehicle::_logDownloadWorker(void)
{
    if (_logDownloadBytesRemaining != 0) {
        QFile file(_logDownloadFilename);
        if (file.open(QIODevice::ReadOnly)) {
            uint8_t buffer[MAVLINK_MSG_LOG_DATA_FIELD_DATA_LEN];

            qint64 bytesToRead = qMin(_logDownloadBytesRemaining, (uint32_t)MAVLINK_MSG_LOG_DATA_FIELD_DATA_LEN);
            Q_ASSERT(file.seek(_logDownloadCurrentOffset));
            Q_ASSERT(file.read((char *)buffer, bytesToRead) == bytesToRead);

            qDebug() << "MockVehicle::_logDownloadWorker" << _logDownloadCurrentOffset << _logDownloadBytesRemaining;

            mavlink_message_t responseMsg;
            mavlink_msg_log_data_pack_chan(_vehicleId,
                                           MAV_COMP_ID_AUTOPILOT1,
                                           _mockLink->mavlinkChannel(),
                                           &responseMsg,
                                           _logDownloadLogId,
                                           _logDownloadCurrentOffset,
                                           bytesToRead,
                                           &buffer[0]);
            respondWithMavlinkMessage(responseMsg);

            _logDownloadCurrentOffset += bytesToRead;
            _logDownloadBytesRemaining -= bytesToRead;

            file.close();
        } else {
            qWarning() << "MockVehicle::_logDownloadWorker open failed" << file.errorString();
        }
    }
}

void MockVehicle::_sendADSBVehicles(void)
{
    _adsbAngle += 2;
    _adsbVehicleCoordinate = QGeoCoordinate(_vehicleLatitude, _vehicleLongitude).atDistanceAndAzimuth(500, _adsbAngle);
    _adsbVehicleCoordinate.setAltitude(100);

    mavlink_message_t responseMsg;
    mavlink_msg_adsb_vehicle_pack_chan(_vehicleId,
                                       MAV_COMP_ID_AUTOPILOT1,
                                       _mockLink->mavlinkChannel(),
                                       &responseMsg,
                                       12345,                                       // ICAO address
                                       _adsbVehicleCoordinate.latitude() * 1e7,
                                       _adsbVehicleCoordinate.longitude() * 1e7,
                                       ADSB_ALTITUDE_TYPE_GEOMETRIC,
                                       _adsbVehicleCoordinate.altitude() * 1000,    // Altitude in millimeters
                                       10 * 100,                                    // Heading in centidegress
                                       0, 0,                                        // Horizontal/Vertical velocity
                                       "N1234500",                                  // Callsign
                                       ADSB_EMITTER_TYPE_ROTOCRAFT,
                                       1,                                           // Seconds since last communication
                                       ADSB_FLAGS_VALID_COORDS | ADSB_FLAGS_VALID_ALTITUDE | ADSB_FLAGS_VALID_HEADING | ADSB_FLAGS_VALID_CALLSIGN | ADSB_FLAGS_SIMULATED,
                                       0);                                          // Squawk code

    respondWithMavlinkMessage(responseMsg);
}

bool MockVehicle::_handleRequestMessage(const mavlink_command_long_t& request, bool& noAck)
{
    noAck = false;

    switch ((int)request.param1) {
    case MAVLINK_MSG_ID_COMPONENT_INFORMATION:
        switch (static_cast<int>(request.param2)) {
        case COMP_METADATA_TYPE_VERSION:
            _sendVersionMetaData();
            return true;
        case COMP_METADATA_TYPE_PARAMETER:
            _sendParameterMetaData();
            return true;
        }
        break;
    case MAVLINK_MSG_ID_DEBUG:
        switch (_requestMessageFailureMode) {
        case FailRequestMessageNone:
            break;
        case FailRequestMessageCommandAcceptedMsgNotSent:
            return true;
        case FailRequestMessageCommandUnsupported:
            return false;
        case FailRequestMessageCommandNoResponse:
            noAck = true;
            return true;
        case FailRequestMessageCommandAcceptedSecondAttempMsgSent:
            return true;
        }
    {
        mavlink_message_t   responseMsg;
        mavlink_msg_debug_pack_chan(_vehicleId,
                                    MAV_COMP_ID_AUTOPILOT1,
                                    _mockLink->mavlinkChannel(),
                                    &responseMsg,
                                    0, 0, 0);
        respondWithMavlinkMessage(responseMsg);
    }
        return true;
    }

    return false;
}

void MockVehicle::_sendVersionMetaData(void)
{
    mavlink_message_t   responseMsg;
#if 1
    char                metaDataURI[MAVLINK_MSG_COMPONENT_INFORMATION_FIELD_METADATA_URI_LEN]       = "mavlinkftp://version.json.gz";
#else
    char                metaDataURI[MAVLINK_MSG_COMPONENT_INFORMATION_FIELD_METADATA_URI_LEN]       = "https://bit.ly/31nm0fs";
#endif
    char                translationURI[MAVLINK_MSG_COMPONENT_INFORMATION_FIELD_TRANSLATION_URI_LEN] = "";

    mavlink_msg_component_information_pack_chan(_vehicleId,
                                                MAV_COMP_ID_AUTOPILOT1,
                                                _mockLink->mavlinkChannel(),
                                                &responseMsg,
                                                0,                          // time_boot_ms
                                                COMP_METADATA_TYPE_VERSION,
                                                1,                          // comp_metadata_uid
                                                metaDataURI,
                                                0,                          // comp_translation_uid
                                                translationURI);
    respondWithMavlinkMessage(responseMsg);
}

void MockVehicle::_sendParameterMetaData(void)
{
    mavlink_message_t   responseMsg;
#if 1
    char                metaDataURI[MAVLINK_MSG_COMPONENT_INFORMATION_FIELD_METADATA_URI_LEN]       = "mavlinkftp://parameter.json";
#else
    char                metaDataURI[MAVLINK_MSG_COMPONENT_INFORMATION_FIELD_METADATA_URI_LEN]       = "https://bit.ly/2ZKRIRE";
#endif
    char                translationURI[MAVLINK_MSG_COMPONENT_INFORMATION_FIELD_TRANSLATION_URI_LEN] = "";

    mavlink_msg_component_information_pack_chan(_vehicleId,
                                                MAV_COMP_ID_AUTOPILOT1,
                                                _mockLink->mavlinkChannel(),
                                                &responseMsg,
                                                0,                              // time_boot_ms
                                                COMP_METADATA_TYPE_PARAMETER,
                                                1,                              // comp_metadata_uid
                                                metaDataURI,
                                                0,                              // comp_translation_uid
                                                translationURI);
    respondWithMavlinkMessage(responseMsg);
}
