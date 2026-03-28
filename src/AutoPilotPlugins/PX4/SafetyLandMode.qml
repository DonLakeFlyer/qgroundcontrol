// Land Mode escape-hatch component for SafetyComponent
import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import QGroundControl
import QGroundControl.FactControls
import QGroundControl.Controls

Rectangle {
    id: root
    required property var controller

    property real _margins: ScreenTools.defaultFontPixelHeight
    property real _labelWidth: ScreenTools.defaultFontPixelWidth * 30
    property real _editFieldWidth: ScreenTools.defaultFontPixelWidth * 20
    property real _imageHeight: ScreenTools.defaultFontPixelHeight * 3
    property real _imageWidth: _imageHeight * 2

    property Fact _disarmLandDelay: controller.getParameterFact(-1, "COM_DISARM_LAND")
    property Fact _landSpeedMC: controller.getParameterFact(-1, "MPC_LAND_SPEED", false)

    width: landModeGrid.width + (_margins * 2)
    height: landModeGrid.height + (_margins * 2)
    color: qgcPal.windowShade

    Row {
        id: landModeGrid
        spacing: _margins
        anchors.centerIn: parent

        Item {
            width: _imageWidth
            height: _imageHeight
            anchors.verticalCenter: parent.verticalCenter
            QGCColoredImage {
                color: qgcPal.text
                source: controller.vehicle.fixedWing ? "/qmlimages/LandMode.svg" : "/qmlimages/LandModeCopter.svg"
                height: _imageHeight
                width: _imageHeight
                anchors.centerIn: parent
            }
        }

        GridLayout {
            columns: 2
            anchors.verticalCenter: parent.verticalCenter

            QGCLabel {
                id: landVelocityLabel
                text: qsTr("Landing Descent Rate:")
                visible: controller.vehicle && !controller.vehicle.fixedWing
                Layout.minimumWidth: _labelWidth
                Layout.fillWidth: true
            }
            FactTextField {
                fact: _landSpeedMC
                visible: controller.vehicle && !controller.vehicle.fixedWing
                Layout.minimumWidth: _editFieldWidth
                Layout.fillWidth: true
            }

            QGCCheckBox {
                id: disarmDelayCheckBox
                text: qsTr("Disarm After:")
                checked: _disarmLandDelay.value > 0
                onClicked: _disarmLandDelay.value = checked ? 2 : 0
                Layout.fillWidth: true
            }
            FactTextField {
                fact: _disarmLandDelay
                enabled: disarmDelayCheckBox.checked
                Layout.fillWidth: true
            }
        }
    }
}
