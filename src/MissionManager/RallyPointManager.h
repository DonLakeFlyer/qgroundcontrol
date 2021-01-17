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
#include <QGeoCoordinate>

#include "QGCLoggingCategory.h"
#include "PlanManager.h"

class Vehicle;
class PlanManager;

Q_DECLARE_LOGGING_CATEGORY(RallyPointManagerLog)

/// This is the base class for firmware specific rally point managers. A rally point manager is responsible
/// for communicating with the vehicle to set/get rally points.
class RallyPointManager : public PlanManager
{
    Q_OBJECT
    
public:
    RallyPointManager(Vehicle* vehicle, QObject* parent);
    ~RallyPointManager();
    
    bool                    supported       (void) const;
    void                    sendToVehicle   (const QList<QGeoCoordinate>& rgPoints);
    QString                 editorQml       (void) const                            { return QStringLiteral("qrc:/FirmwarePlugin/RallyPointEditor.qml"); }
    QList<QGeoCoordinate>   points          (void) const                            { return _rgPoints; }
    
private slots:
    void _managerSendComplete       (bool error);
    void _managerLoadComplete       (bool error);
    void _managerRemoveAllComplete  (bool error);

protected:
    QList<QGeoCoordinate> _rgPoints;
    QList<QGeoCoordinate> _rgSendPoints;
};
