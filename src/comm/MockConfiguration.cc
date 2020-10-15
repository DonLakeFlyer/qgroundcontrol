/****************************************************************************
 *
 * (c) 2009-2020 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "MockConfiguration.h"

#include <QDebug>

const char* MockConfiguration::_firmwareTypeKey         = "FirmwareType";
const char* MockConfiguration::_vehicleTypeKey          = "VehicleType";
const char* MockConfiguration::_sendStatusTextKey       = "SendStatusText";
const char* MockConfiguration::_incrementVehicleIdKey   = "IncrementVehicleId";
const char* MockConfiguration::_failureModeKey          = "FailureMode";
const char* MockConfiguration::_twoVehiclesKey          = "TwoVehicles";

MockConfiguration::MockConfiguration(const QString& name)
    : LinkConfiguration(name)
{

}

MockConfiguration::MockConfiguration(MockConfiguration* source)
    : LinkConfiguration(source)
{
    _firmwareType       = source->_firmwareType;
    _vehicleType        = source->_vehicleType;
    _sendStatusText     = source->_sendStatusText;
    _incrementVehicleId = source->_incrementVehicleId;
    _failureMode        = source->_failureMode;
    _twoVehicles        = source->_twoVehicles;
}

void MockConfiguration::copyFrom(LinkConfiguration *source)
{
    LinkConfiguration::copyFrom(source);
    auto* usource = qobject_cast<MockConfiguration*>(source);

    if (!usource) {
        qWarning() << "dynamic_cast failed" << source << usource;
        return;
    }

    _firmwareType       = usource->_firmwareType;
    _vehicleType        = usource->_vehicleType;
    _sendStatusText     = usource->_sendStatusText;
    _incrementVehicleId = usource->_incrementVehicleId;
    _failureMode        = usource->_failureMode;
    _twoVehicles        = usource->_twoVehicles;
}

void MockConfiguration::saveSettings(QSettings& settings, const QString& root)
{
    settings.beginGroup(root);
    settings.setValue(_firmwareTypeKey,         (int)_firmwareType);
    settings.setValue(_vehicleTypeKey,          (int)_vehicleType);
    settings.setValue(_sendStatusTextKey,       _sendStatusText);
    settings.setValue(_incrementVehicleIdKey,   _incrementVehicleId);
    settings.setValue(_twoVehiclesKey,          _twoVehicles);
    settings.setValue(_failureModeKey,          (int)_failureMode);
    settings.sync();
    settings.endGroup();
}

void MockConfiguration::loadSettings(QSettings& settings, const QString& root)
{
    settings.beginGroup(root);
    _firmwareType       = (MAV_AUTOPILOT)settings.value(_firmwareTypeKey, (int)MAV_AUTOPILOT_PX4).toInt();
    _vehicleType        = (MAV_TYPE)settings.value(_vehicleTypeKey, (int)MAV_TYPE_QUADROTOR).toInt();
    _sendStatusText     = settings.value(_sendStatusTextKey, false).toBool();
    _incrementVehicleId = settings.value(_incrementVehicleIdKey, true).toBool();
    _twoVehicles        = settings.value(_twoVehiclesKey, false).toBool();
    _failureMode        = (FailureMode_t)settings.value(_failureModeKey, (int)FailNone).toInt();
    settings.endGroup();
}
