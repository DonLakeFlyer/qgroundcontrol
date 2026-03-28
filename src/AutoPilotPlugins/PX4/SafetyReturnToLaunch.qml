// Return To Launch escape-hatch component for SafetyComponent
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

    property Fact _rtlLandDelay: controller.getParameterFact(-1, "RTL_LAND_DELAY")

    width: returnHomeGrid.width + (_margins * 2)
    height: returnHomeGrid.height + (_margins * 2)
    color: qgcPal.windowShade

    Row {
        id: returnHomeGrid
        spacing: _margins
        anchors.centerIn: parent

        Item {
            width: _imageWidth
            height: _imageHeight
            anchors.verticalCenter: parent.verticalCenter
            QGCColoredImage {
                color: qgcPal.text
                source: controller.vehicle.fixedWing ? "/qmlimages/ReturnToHomeAltitude.svg" : "/qmlimages/ReturnToHomeAltitudeCopter.svg"
                height: _imageHeight
                width: _imageHeight * 2
                anchors.centerIn: parent
            }
        }

        GridLayout {
            columns: 2
            anchors.verticalCenter: parent.verticalCenter

            QGCLabel {
                text: qsTr("Climb to altitude of:")
                Layout.minimumWidth: _labelWidth
                Layout.fillWidth: true
            }
            FactTextField {
                fact: controller.getParameterFact(-1, "RTL_RETURN_ALT")
                Layout.minimumWidth: _editFieldWidth
                Layout.fillWidth: true
            }

            QGCLabel {
                text: qsTr("Return to launch, then:")
                Layout.columnSpan: 2
            }
            Row {
                Layout.columnSpan: 2
                Item { width: ScreenTools.defaultFontPixelWidth; height: 1 }
                QGCRadioButton {
                    id: homeLandRadio
                    checked: _rtlLandDelay ? _rtlLandDelay.value === 0 : false
                    text: qsTr("Land immediately")
                    onClicked: _rtlLandDelay.value = 0
                }
            }
            Row {
                Layout.columnSpan: 2
                Item { width: ScreenTools.defaultFontPixelWidth; height: 1 }
                QGCRadioButton {
                    id: homeLoiterNoLandRadio
                    checked: _rtlLandDelay ? _rtlLandDelay.value < 0 : false
                    text: qsTr("Loiter and do not land")
                    onClicked: _rtlLandDelay.value = -1
                }
            }
            Row {
                Layout.columnSpan: 2
                Item { width: ScreenTools.defaultFontPixelWidth; height: 1 }
                QGCRadioButton {
                    id: homeLoiterLandRadio
                    checked: _rtlLandDelay ? _rtlLandDelay.value > 0 : false
                    text: qsTr("Loiter and land after specified time")
                    onClicked: _rtlLandDelay.value = 60
                }
            }

            QGCLabel {
                text: qsTr("Loiter Time")
                Layout.fillWidth: true
            }
            FactTextField {
                fact: controller.getParameterFact(-1, "RTL_LAND_DELAY")
                enabled: homeLoiterLandRadio.checked === true
                Layout.fillWidth: true
            }

            QGCLabel {
                text: qsTr("Loiter Altitude")
                Layout.fillWidth: true
            }
            FactTextField {
                fact: controller.getParameterFact(-1, "RTL_DESCEND_ALT")
                enabled: homeLoiterLandRadio.checked === true || homeLoiterNoLandRadio.checked === true
                Layout.fillWidth: true
            }
        }
    }
}
