/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

import QtQuick
import QtQuick.Shapes

import QGroundControl
import QGroundControl.GeoMap

/// Circle outline: the GeoMap counterpart of the QtLocation
/// QGCMapCircleVisuals ring (outline only — no drag handles or rotation
/// arrows). Built from great-circle points so it stays correct under 3D
/// camera tilt. Ground-projected by default; set altitudeMode to
/// GeoMapItem.Absolute to render at the center coordinate's altitude.
Item {
    id: root

    property var scene
    property var surfaceModel
    property var center                 ///< QGeoCoordinate
    property real radiusMeters: 0
    property color strokeColor: QGroundControl.globalPalette.mapMissionTrajectory
    property real strokeWidth: 3
    property alias altitudeMode: pathProjector.altitudeMode

    GeoMapProjectedPath {
        id: pathProjector
        scene: root.scene
        surfaceModel: root.surfaceModel
        // Skip the projection work entirely while hidden
        coordinates: (root.visible && root.center && root.center.isValid && root.radiusMeters > 0)
                     ? pathProjector.circleCoordinates(root.center, root.radiusMeters)
                     : []
    }

    Shape {
        visible: pathProjector.projected

        ShapePath {
            strokeColor: root.strokeColor
            strokeWidth: root.strokeWidth
            fillColor: "transparent"
            PathPolyline { path: pathProjector.screenPoints }
        }
    }
}
