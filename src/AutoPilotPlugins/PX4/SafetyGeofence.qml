// Geofence Failsafe escape-hatch component for SafetyComponent
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

    property Fact _fenceAction: controller.getParameterFact(-1, "GF_ACTION")
    property Fact _fenceRadius: controller.getParameterFact(-1, "GF_MAX_HOR_DIST")
    property Fact _fenceAlt:    controller.getParameterFact(-1, "GF_MAX_VER_DIST")

    width: geoFenceGrid.width + (_margins * 2)
    height: geoFenceGrid.height + (_margins * 2)
    color: qgcPal.windowShade

    Row {
        id: geoFenceGrid
        spacing: _margins
        anchors.centerIn: parent

        Item {
            width: _imageWidth
            height: _imageHeight
            anchors.verticalCenter: parent.verticalCenter
            Image {
                mipmap: true
                fillMode: Image.PreserveAspectFit
                source: qgcPal.globalTheme === QGCPalette.Light ? "/qmlimages/GeoFenceLight.svg" : "/qmlimages/GeoFence.svg"
                height: _imageHeight
                anchors.centerIn: parent
            }
        }

        GridLayout {
            columns: 2
            anchors.verticalCenter: parent.verticalCenter

            QGCLabel {
                text: qsTr("Action on breach:")
                Layout.minimumWidth: _labelWidth
                Layout.fillWidth: true
            }
            FactComboBox {
                fact: _fenceAction
                indexModel: false
                Layout.minimumWidth: _editFieldWidth
                Layout.fillWidth: true
            }

            QGCCheckBox {
                id: fenceRadiusCheckBox
                text: qsTr("Max Radius:")
                checked: _fenceRadius.value > 0
                onClicked: _fenceRadius.value = checked ? 100 : 0
                Layout.fillWidth: true
            }
            FactTextField {
                fact: _fenceRadius
                enabled: fenceRadiusCheckBox.checked
                Layout.fillWidth: true
            }

            QGCCheckBox {
                id: fenceAltMaxCheckBox
                text: qsTr("Max Altitude:")
                checked: _fenceAlt ? _fenceAlt.value > 0 : false
                onClicked: _fenceAlt.value = checked ? 100 : 0
                Layout.fillWidth: true
            }
            FactTextField {
                fact: _fenceAlt
                enabled: fenceAltMaxCheckBox.checked
                Layout.fillWidth: true
            }
        }
    }
}
