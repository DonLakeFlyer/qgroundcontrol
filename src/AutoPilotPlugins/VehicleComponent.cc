#include "VehicleComponent.h"
#include "ParameterManager.h"
#include "QGCLoggingCategory.h"
#include "Vehicle.h"

#include <QtCore/QFile>
#include <QtCore/QJsonArray>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtQml/QQmlContext>
#include <QtQuick/QQuickItem>

QGC_LOGGING_CATEGORY(VehicleComponentLog, "AutoPilotPlugins.VehicleComponent");

VehicleComponent::VehicleComponent(Vehicle *vehicle, AutoPilotPlugin *autopilot, AutoPilotPlugin::KnownVehicleComponent KnownVehicleComponent, QObject *parent)
    : QObject(parent)
    , _vehicle(vehicle)
    , _autopilot(autopilot)
    , _KnownVehicleComponent(KnownVehicleComponent)
{
    // qCDebug(VehicleComponentLog) << Q_FUNC_INFO << this;

    if (!vehicle || !autopilot) {
        qCWarning(VehicleComponentLog) << "Internal error";
    }
}

VehicleComponent::~VehicleComponent()
{
    // qCDebug(VehicleComponentLog) << Q_FUNC_INFO << this;
}

QStringList VehicleComponent::sections() const
{
    if (_sectionsParsed) {
        return _cachedSections;
    }
    _sectionsParsed = true;

    const QString path = pageDefinition();
    if (path.isEmpty()) {
        return _cachedSections;
    }

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        qCWarning(VehicleComponentLog) << "Failed to open page definition:" << path;
        return _cachedSections;
    }

    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
    const QJsonArray sectionsArray = doc.object().value("sections").toArray();

    for (const QJsonValue &val : sectionsArray) {
        const QJsonObject secObj = val.toObject();
        const QString name = secObj.value("name").toString();
        if (name.isEmpty()) {
            continue;
        }

        const QJsonObject repeatObj = secObj.value("repeat").toObject();
        if (!repeatObj.isEmpty() && _vehicle && _vehicle->parameterManager()) {
            const QString paramPrefix = repeatObj.value("paramPrefix").toString();
            const QString probePostfix = repeatObj.value("probePostfix").toString();
            const int startIndex = repeatObj.value("startIndex").toInt(1);
            const bool firstOmits = repeatObj.value("firstIndexOmitsNumber").toBool(false);

            int count = 0;
            for (int i = startIndex; ; i++) {
                const QString idx = (firstOmits && i == startIndex) ? QString() : QString::number(i);
                const QString probeParam = paramPrefix + idx + probePostfix;
                if (!_vehicle->parameterManager()->parameterExists(ParameterManager::defaultComponentId, probeParam))
                    break;
                count++;
            }

            if (count <= 1) {
                _cachedSections.append(name);
            } else {
                for (int i = 0; i < count; i++) {
                    _cachedSections.append(name + QStringLiteral(" ") + QString::number(startIndex + i));
                }
            }
        } else {
            _cachedSections.append(name);
        }
    }

    return _cachedSections;
}

void VehicleComponent::addSummaryQmlComponent(QQmlContext *context, QQuickItem *parent)
{
    if (!context) {
        qCWarning(VehicleComponentLog) << "Internal error";
        return;
    }

    QQmlComponent component = new QQmlComponent(context->engine(), QUrl::fromUserInput("qrc:/qml/VehicleComponentSummaryButton.qml"), this);
    if (component.status() == QQmlComponent::Error) {
        qCWarning(VehicleComponentLog) << component.errors();
        return;
    }

    QQuickItem *const item = qobject_cast<QQuickItem*>(component.create(context));
    if (!item) {
        qCWarning(VehicleComponentLog) << "Internal error";
        return;
    }

    item->setParentItem(parent);
    item->setProperty("vehicleComponent", QVariant::fromValue(this));
}

void VehicleComponent::setupTriggerSignals()
{
    // Watch for changed on trigger list params
    for (const QString &paramName: setupCompleteChangedTriggerList()) {
        if (_vehicle->parameterManager()->parameterExists(ParameterManager::defaultComponentId, paramName)) {
            Fact *const fact = _vehicle->parameterManager()->getParameter(ParameterManager::defaultComponentId, paramName);
            (void) connect(fact, &Fact::valueChanged, this, &VehicleComponent::_triggerUpdated);
        }
    }
}
