/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

import QGroundControl.GeoMap
import QGroundControl.PlanView

/// Point-marker anchor for complex mission item visuals: wraps
/// MissionItemIndexLabel.qml (confirmed QtLocation-independent) at a
/// coordinate, the same way GeoMapWaypointItem anchors its own indicator
/// content.
GeoMapItem {
    id: root

    property alias index: label.index
    property alias label: label.label
    property alias checked: label.checked

    altitudeMode: GeoMapItem.ClampToGround
    anchorPoint: Qt.point(label.anchorPointX, label.anchorPointY)
    width: label.width
    height: label.height

    MissionItemIndexLabel {
        id: label

        highlightSelected: true
    }
}
