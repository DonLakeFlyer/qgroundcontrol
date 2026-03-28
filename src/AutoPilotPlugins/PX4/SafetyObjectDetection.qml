// Object Detection escape-hatch component for SafetyComponent
import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import QGroundControl
import QGroundControl.FactControls
import QGroundControl.Controls

ColumnLayout {
    id: root
    required property var controller

    property Fact _collisionPrevention: controller.getParameterFact(-1, "CP_DIST")
    property bool _cpEnabled: _collisionPrevention && _collisionPrevention.rawValue > 0

    Binding {
        target:   cpSlider
        property: "enableCheckBoxChecked"
        value:    root._cpEnabled
    }

    FactTextFieldSlider {
        id:                     cpSlider
        Layout.fillWidth:       true
        label:                  qsTr("Collision Prevention Minimum Distance")
        fact:                   root._collisionPrevention
        showEnableCheckbox:     true

        onEnableCheckboxClicked: {
            if (root._collisionPrevention) {
                root._collisionPrevention.rawValue = cpSlider.enableCheckBoxChecked ? 5 : -1
            }
        }
    }

    FactCheckBox {
        id:      showObstacleDistanceOverlayCheckBox
        text:    qsTr("Show obstacle distance overlay")
        visible: _showObstacleDistanceOverlay ? _showObstacleDistanceOverlay.visible : false
        fact:    _showObstacleDistanceOverlay

        property Fact _showObstacleDistanceOverlay: QGroundControl.settingsManager.flyViewSettings.showObstacleDistanceOverlay
    }
}
