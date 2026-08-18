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

/// Base visual shared by Fixed Wing and VTOL landing patterns on GeoMap:
/// read-only GeoMap-native port of the common elements in
/// src/PlanView/FWLandingPatternMapVisual.qml / VTOLLandingPatternMapVisual.qml
/// (flight path, loiter ring, final-approach/landing markers). No
/// mouse-area/drag-to-place code ports over — FlyView missions are already
/// loaded and immutable.
Item {
    id: root

    property var item          ///< VisualMissionItem (LandingComplexItem)
    property var scene
    property var surfaceModel
    property string landingLabel: ""   ///< Label for the landing-point marker (VTOL overrides to "Land")

    readonly property bool _useLoiterToAlt: item.useLoiterToAlt.rawValue
    readonly property var _flightPath: _useLoiterToAlt ? [item.slopeStartCoordinate, item.landingCoordinate]
                                                        : [item.finalApproachCoordinate, item.landingCoordinate]

    GeoMapProjectedPath {
        id: flightPathProjector
        scene: root.scene
        surfaceModel: root.surfaceModel
        coordinates: root._flightPath
    }

    // Flight path (final approach/loiter exit -> landing point)
    Shape {
        visible: root.item.landingCoordSet && flightPathProjector.projected
        ShapePath {
            strokeColor: "#be781c"
            strokeWidth: 2
            fillColor: "transparent"
            PathPolyline { path: flightPathProjector.screenPoints }
        }
    }

    GeoMapProjectedPath {
        id: loiterPath
        scene: root.scene
        surfaceModel: root.surfaceModel
        coordinates: root._useLoiterToAlt
                     ? GeoMapProjectedPath.circleCoordinates(root.item.finalApproachCoordinate, root.item.loiterRadius.rawValue)
                     : []
    }

    // Loiter ring
    Shape {
        visible: root._useLoiterToAlt && loiterPath.projected
        ShapePath {
            strokeColor: "green"
            strokeWidth: 2
            fillColor: "transparent"
            PathPolyline { path: loiterPath.screenPoints }
        }
    }

    // Final approach / loiter point
    GeoMapMissionLabel {
        scene: root.scene
        surfaceModel: root.surfaceModel
        coordinate: root.item.finalApproachCoordinate
        index: root.item.sequenceNumber
        label: root._useLoiterToAlt ? qsTr("Loiter") : qsTr("Approach")
        checked: root.item.isCurrentItem
        visible: root.item.landingCoordSet
    }

    // Landing point
    GeoMapMissionLabel {
        scene: root.scene
        surfaceModel: root.surfaceModel
        coordinate: root.item.landingCoordinate
        index: root.item.lastSequenceNumber
        label: root.landingLabel
        checked: root.item.isCurrentItem
        visible: root.item.landingCoordSet
    }
}
