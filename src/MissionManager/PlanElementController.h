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

#include "Vehicle.h"
#include "MultiVehicleManager.h"

class PlanMasterController;

/// This is the abstract base clas for Plan Element controllers.
/// Examples of plan elements are: missions (MissionController), geofence (GeoFenceController)
class PlanElementController : public QObject
{
    Q_OBJECT
    
public:
    PlanElementController(Vehicle* vehicle, QObject* parent);
    ~PlanElementController();
    
    Q_PROPERTY(bool supported       READ supported                      NOTIFY supportedChanged)        ///< true: Element is supported by Vehicle
    Q_PROPERTY(bool containsItems   READ containsItems                  NOTIFY containsItemsChanged)    ///< true: Elemement is non-empty
    Q_PROPERTY(bool syncInProgress  READ syncInProgress                 NOTIFY syncInProgressChanged)   ///< true: information is currently being saved/sent, false: no active save/send in progress
    Q_PROPERTY(bool dirty           READ dirty          WRITE setDirty  NOTIFY dirtyChanged)            ///< true: unsaved/sent changes are present, false: no changes since last save/send

    virtual void saveToJson         (QJsonObject& json) = 0;
    virtual bool loadFromJson       (const QJsonObject& json, QString& errorString) = 0;
    virtual void removeAll          (void) = 0;

    virtual bool supported              (void) const = 0;
    virtual bool containsItems          (void) const = 0;
    virtual bool syncInProgress         (void) const = 0;
    virtual bool dirty                  (void) const = 0;
    virtual void setDirty               (bool dirty) = 0;
    virtual void sendToVehicle          (void) = 0;         ///< Signals sendComplete when done
    virtual void loadFromVehicle        (void) = 0;
    virtual void removeAllFromVehicle   (void) = 0;         ///< Signals removeAllComplete when done

signals:
    void supportedChanged       (bool supported);
    void containsItemsChanged   (bool containsItems);
    void syncInProgressChanged  (bool syncInProgress);
    void dirtyChanged           (bool dirty);
    void sendComplete           (bool error);
    void loadComplete           (bool error);
    void removeAllComplete      (bool error);

protected:
    Vehicle* _vehicle = nullptr;
};
