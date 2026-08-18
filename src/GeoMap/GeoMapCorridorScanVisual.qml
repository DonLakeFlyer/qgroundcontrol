/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

import QtQuick.Shapes

import QGroundControl.GeoMap

/// Corridor Scan complex mission item visual for GeoMap: the shared transect
/// base plus the corridor centerline (src/PlanView/CorridorScanMapVisual.qml)
GeoMapTransectStyleVisuals {
    id: root

    GeoMapProjectedPath {
        id: corridorPath
        scene: root.scene
        surfaceModel: root.surfaceModel
        coordinates: root.item.corridorPolyline.path
    }

    Shape {
        visible: root.item.isCurrentItem && corridorPath.projected
        ShapePath {
            strokeColor: "#be781c"
            strokeWidth: 3
            fillColor: "transparent"
            PathPolyline { path: corridorPath.screenPoints }
        }
    }
}
