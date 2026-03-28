// Vehicle Telemetry Logging escape-hatch component for SafetyComponent
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

    property Fact _enableLogging: controller.getParameterFact(-1, "SDLOG_MODE")

    width: loggingGrid.width + (_margins * 2)
    height: loggingGrid.height + (_margins * 2)
    color: qgcPal.windowShade

    Row {
        id: loggingGrid
        spacing: _margins
        anchors.centerIn: parent

        Item {
            width: _imageWidth
            height: _imageHeight
            anchors.verticalCenter: parent.verticalCenter
            Image {
                mipmap: true
                fillMode: Image.PreserveAspectFit
                source: qgcPal.globalTheme === QGCPalette.Light ? "/qmlimages/no-logging-light.svg" : "/qmlimages/no-logging.svg"
                height: _imageHeight
                anchors.centerIn: parent
            }
        }

        GridLayout {
            columns: 2
            anchors.verticalCenter: parent.verticalCenter

            QGCLabel {
                text: qsTr("Telemetry logging to vehicle storage:")
                Layout.minimumWidth: _labelWidth
                Layout.fillWidth: true
            }
            QGCComboBox {
                model: [qsTr("Disabled"), qsTr("Enabled")]
                enabled: _enableLogging
                Layout.minimumWidth: _editFieldWidth
                Layout.fillWidth: true
                Component.onCompleted: {
                    currentIndex = _enableLogging ? (_enableLogging.value >= 0 ? 1 : 0) : 0
                }
                onActivated: (index) => {
                    if (_enableLogging) {
                        _enableLogging.value = index > 0 ? 0 : -1
                    }
                }
            }
        }
    }
}
