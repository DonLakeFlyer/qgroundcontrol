/****************************************************************************
 *
 * (c) 2009-2020 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#pragma once

#include "PlanElementController.h"
#include "RallyPointManager.h"
#include "Vehicle.h"
#include "MultiVehicleManager.h"
#include "QGCLoggingCategory.h"
#include "QmlObjectListModel.h"

Q_DECLARE_LOGGING_CATEGORY(RallyPointControllerLog)

class RallyPointManager;

class RallyPointController : public PlanElementController
{
    Q_OBJECT
    
public:
    RallyPointController(RallyPointManager* rallyPointManager, QObject* parent);
    ~RallyPointController();
    
    Q_PROPERTY(QmlObjectListModel*  points                  READ points                                             CONSTANT)
    Q_PROPERTY(QString              editorQml               READ editorQml                                          CONSTANT)
    Q_PROPERTY(QObject*             currentRallyPoint       READ currentRallyPoint      WRITE setCurrentRallyPoint  NOTIFY currentRallyPointChanged)

    Q_INVOKABLE void addPoint       (QGeoCoordinate point);
    Q_INVOKABLE void removePoint    (QObject* rallyPoint);

    bool supported                  (void) const final;
    void saveToJson                 (QJsonObject& json) final;
    bool loadFromJson               (const QJsonObject& json, QString& errorString) final;
    void loadFromVehicle            (void) final;
    void sendToVehicle              (void) final;
    void removeAll                  (void) final;
    void removeAllFromVehicle       (void) final;
    bool syncInProgress             (void) const final;
    bool dirty                      (void) const final { return _dirty; }
    void setDirty                   (bool dirty) final;
    bool containsItems              (void) const final;

    QList<QGeoCoordinate> getRallyPoints(void);

    QmlObjectListModel* points                  (void) { return &_points; }
    QString             editorQml               (void) const;
    QObject*            currentRallyPoint       (void) const { return _currentRallyPoint; }

    void setCurrentRallyPoint   (QObject* rallyPoint);
    bool isEmpty                (void) const;

signals:
    void currentRallyPointChanged(QObject* rallyPoint);

private slots:
    void _managerLoadComplete   (bool error);
    void _setFirstPointCurrent  (void);
    void _updateContainsItems   (void);

private:
    RallyPointManager*  _rallyPointManager  = nullptr;
    bool                _dirty              = false;
    QmlObjectListModel  _points;
    QObject*            _currentRallyPoint  = nullptr;

    static const int    _jsonCurrentVersion = 2;
    static const char*  _jsonFileTypeValue;
    static const char*  _jsonPointsKey;
};
