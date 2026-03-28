// HITL Simulation escape-hatch component for SafetyComponent
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

    readonly property string _hitlParam: "SYS_HITL"
    property bool _hitlAvailable: controller.parameterExists(-1, _hitlParam)
    property Fact _hitlEnabled: controller.getParameterFact(-1, _hitlParam, false)

    visible: _hitlAvailable
    width: hitlGrid.width + (_margins * 2)
    height: hitlGrid.height + (_margins * 2)
    color: qgcPal.windowShade

    Row {
        id: hitlGrid
        spacing: _margins
        anchors.centerIn: parent

        Item {
            width: _imageWidth
            height: _imageHeight
            anchors.verticalCenter: parent.verticalCenter
            QGCColoredImage {
                color: qgcPal.text
                source: "/qmlimages/HITL.svg"
                height: _imageHeight
                width: _imageHeight
                anchors.centerIn: parent
            }
        }

        GridLayout {
            columns: 2
            anchors.verticalCenter: parent.verticalCenter

            QGCLabel {
                text: qsTr("HITL Enabled:")
                Layout.minimumWidth: _labelWidth
                Layout.fillWidth: true
            }
            FactComboBox {
                fact: _hitlEnabled
                indexModel: false
                Layout.minimumWidth: _editFieldWidth
                Layout.fillWidth: true
            }
        }
    }
}
