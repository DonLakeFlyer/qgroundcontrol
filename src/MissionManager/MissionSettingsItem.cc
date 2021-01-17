/****************************************************************************
 *
 * (c) 2009-2020 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "MissionSettingsItem.h"
#include "JsonHelper.h"
#include "MissionController.h"
#include "QGCGeo.h"
#include "SimpleMissionItem.h"
#include "SettingsManager.h"
#include "AppSettings.h"
#include "MissionCommandUIInfo.h"

#include <QPolygonF>

QGC_LOGGING_CATEGORY(MissionSettingsItemLog, "MissionSettingsItemLog")

const char* MissionSettingsItem::_plannedHomePositionAltitudeName = "PlannedHomePositionAltitude";

QMap<QString, FactMetaData*> MissionSettingsItem::_metaDataMap;

MissionSettingsItem::MissionSettingsItem(Vehicle* vehicle, QObject* parent)
    : ComplexMissionItem                (vehicle, parent)
    , _vehicle                          (vehicle)
    , _cameraSection                    (vehicle)
    , _speedSection                     (vehicle)
{
    _isIncomplete = false;
    _editorQml = "qrc:/qml/MissionSettingsEditor.qml";

    if (_metaDataMap.isEmpty()) {
        _metaDataMap = FactMetaData::createMapFromJsonFile(QStringLiteral(":/json/MissionSettings.FactMetaData.json"), nullptr /* metaDataParent */);
    }

    _cameraSection.setAvailable(true);
    _speedSection.setAvailable(true);

    connect(this,               &MissionSettingsItem::specifyMissionFlightSpeedChanged, this, &MissionSettingsItem::_setDirtyAndUpdateLastSequenceNumber);
    connect(&_cameraSection,    &CameraSection::itemCountChanged,                       this, &MissionSettingsItem::_setDirtyAndUpdateLastSequenceNumber);
    connect(&_speedSection,     &CameraSection::itemCountChanged,                       this, &MissionSettingsItem::_setDirtyAndUpdateLastSequenceNumber);
    connect(&_cameraSection,    &CameraSection::dirtyChanged,                           this, &MissionSettingsItem::_sectionDirtyChanged);
    connect(&_speedSection,     &SpeedSection::dirtyChanged,                            this, &MissionSettingsItem::_sectionDirtyChanged);
    connect(&_cameraSection,    &CameraSection::specifiedGimbalYawChanged,              this, &MissionSettingsItem::specifiedGimbalYawChanged);
    connect(&_cameraSection,    &CameraSection::specifiedGimbalPitchChanged,            this, &MissionSettingsItem::specifiedGimbalPitchChanged);
    connect(&_speedSection,     &SpeedSection::specifiedFlightSpeedChanged,             this, &MissionSettingsItem::specifiedFlightSpeedChanged);
    connect(this,               &MissionSettingsItem::coordinateChanged,                this, &MissionSettingsItem::_amslEntryAltChanged);
    connect(this,               &MissionSettingsItem::coordinateChanged,                this, &MissionSettingsItem::exitCoordinateChanged);
    connect(this,               &MissionSettingsItem::amslEntryAltChanged,              this, &MissionSettingsItem::amslExitAltChanged);
    connect(this,               &MissionSettingsItem::amslEntryAltChanged,              this, &MissionSettingsItem::minAMSLAltitudeChanged);
    connect(this,               &MissionSettingsItem::amslEntryAltChanged,              this, &MissionSettingsItem::maxAMSLAltitudeChanged);
    connect(_vehicle,           &Vehicle::homePositionChanged,                          this, &MissionSettingsItem::coordinateChanged);
}

int MissionSettingsItem::lastSequenceNumber(void) const
{
    int lastSequenceNumber = _sequenceNumber;

    lastSequenceNumber += _cameraSection.itemCount();
    lastSequenceNumber += _speedSection.itemCount();

    return lastSequenceNumber;
}

void MissionSettingsItem::setDirty(bool dirty)
{
    if (_dirty != dirty) {
        _dirty = dirty;
        if (!dirty) {
            _cameraSection.setDirty(false);
            _speedSection.setDirty(false);
        }
        emit dirtyChanged(_dirty);
    }
}

void MissionSettingsItem::save(QJsonArray&  missionItems)
{
    QList<MissionItem*> items;

    appendMissionItems(items, this);

    // First item should be planned home position, we are not responsible for save/load
    // Remaining items we just output as is
    for (int i=1; i<items.count(); i++) {
        MissionItem* item = items[i];
        QJsonObject saveObject;
        item->save(saveObject);
        missionItems.append(saveObject);
        item->deleteLater();
    }
}

void MissionSettingsItem::setSequenceNumber(int sequenceNumber)
{
    if (_sequenceNumber != sequenceNumber) {
        _sequenceNumber = sequenceNumber;
        emit sequenceNumberChanged(sequenceNumber);
        emit lastSequenceNumberChanged(lastSequenceNumber());
    }
}

bool MissionSettingsItem::load(const QJsonObject& /*complexObject*/, int /*sequenceNumber*/, QString& /*errorString*/)
{
    return true;
}

double MissionSettingsItem::greatestDistanceTo(const QGeoCoordinate &other) const
{
    Q_UNUSED(other);
    return 0;
}

bool MissionSettingsItem::specifiesCoordinate(void) const
{
    return true;
}

void MissionSettingsItem::appendMissionItems(QList<MissionItem*>& items, QObject* missionItemParent)
{
    int seqNum = _sequenceNumber;

    // IMPORTANT NOTE: If anything changes here you must also change MissionSettingsItem::scanForMissionSettings

    // Planned home position
    MissionItem* item = new MissionItem(seqNum++,
                                        MAV_CMD_NAV_WAYPOINT,
                                        MAV_FRAME_GLOBAL,
                                        0,                      // Hold time
                                        0,                      // Acceptance radius
                                        0,                      // Not sure?
                                        0,                      // Yaw
                                        coordinate().latitude(),
                                        coordinate().longitude(),
                                        _vehicle->homePosition().altitude(),
                                        true,                   // autoContinue
                                        false,                  // isCurrentItem
                                        missionItemParent);
    items.append(item);

    _cameraSection.appendSectionItems(items, missionItemParent, seqNum);
    _speedSection.appendSectionItems(items, missionItemParent, seqNum);
}

bool MissionSettingsItem::addMissionEndAction(QList<MissionItem*>& /*items*/, int /*seqNum*/, QObject* /*missionItemParent*/)
{
    return false;
}

bool MissionSettingsItem::scanForMissionSettings(QmlObjectListModel* visualItems, int scanIndex)
{
    bool foundSpeedSection = false;
    bool foundCameraSection = false;

    qCDebug(MissionSettingsItemLog) << "MissionSettingsItem::scanForMissionSettings count:scanIndex" << visualItems->count() << scanIndex;

    // Scan through the initial mission items for possible mission settings
    foundCameraSection = _cameraSection.scanForSection(visualItems, scanIndex);
    foundSpeedSection = _speedSection.scanForSection(visualItems, scanIndex);

    return foundSpeedSection || foundCameraSection;
}

double MissionSettingsItem::complexDistance(void) const
{
    return 0;
}

void MissionSettingsItem::_setDirty(void)
{
    setDirty(true);
}

void MissionSettingsItem::_setDirtyAndUpdateLastSequenceNumber(void)
{
    emit lastSequenceNumberChanged(lastSequenceNumber());
    setDirty(true);
}

void MissionSettingsItem::_sectionDirtyChanged(bool dirty)
{
    if (dirty) {
        setDirty(true);
    }
}

double MissionSettingsItem::specifiedGimbalYaw(void)
{
    return _cameraSection.specifyGimbal() ? _cameraSection.gimbalYaw()->rawValue().toDouble() : std::numeric_limits<double>::quiet_NaN();
}

double MissionSettingsItem::specifiedGimbalPitch(void)
{
    return _cameraSection.specifyGimbal() ? _cameraSection.gimbalPitch()->rawValue().toDouble() : std::numeric_limits<double>::quiet_NaN();
}

double MissionSettingsItem::specifiedFlightSpeed(void)
{
    if (_speedSection.specifyFlightSpeed()) {
        return _speedSection.flightSpeed()->rawValue().toDouble();
    } else {
        return std::numeric_limits<double>::quiet_NaN();
    }
}

QString MissionSettingsItem::abbreviation(void) const
{
    return tr("Launch");
}
