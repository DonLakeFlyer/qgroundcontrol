/****************************************************************************
 *
 * (c) 2009-2020 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "PlanElementController.h"
#include "PlanMasterController.h"
#include "QGCApplication.h"
#include "MultiVehicleManager.h"
#include "SettingsManager.h"
#include "AppSettings.h"

PlanElementController::PlanElementController(Vehicle* vehicle, QObject* parent)
    : QObject   (parent)
    , _vehicle  (vehicle)
{

}

PlanElementController::~PlanElementController()
{

}
