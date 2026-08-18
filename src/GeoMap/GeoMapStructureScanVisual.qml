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

/// Structure Scan complex mission item visual for GeoMap: read-only
/// GeoMap-native port of src/PlanView/StructureScanMapVisual.qml
Item {
    id: root

    property var item          ///< VisualMissionItem (StructureScanComplexItem)
    property var scene
    property var surfaceModel

    GeoMapProjectedPath {
        id: structurePath
        scene: root.scene
        surfaceModel: root.surfaceModel
        coordinates: root.item.structurePolygon.path
    }

    // Structure footprint
    Shape {
        visible: structurePath.projected
        ShapePath {
            strokeColor: "black"
            strokeWidth: 1
            fillColor: root.item.structurePolygon.showAltColor ? "red" : "green"
            fillOpacity: 0.5
            PathPolyline { path: structurePath.screenPoints }
        }
    }

    GeoMapProjectedPath {
        id: flightPath
        scene: root.scene
        surfaceModel: root.surfaceModel
        coordinates: root.item.flightPolygon.path
    }

    // Offset flight polygon
    Shape {
        visible: flightPath.projected
        ShapePath {
            strokeColor: "white"
            strokeWidth: 2
            fillColor: "transparent"
            PathPolyline { path: flightPath.screenPoints }
        }
    }

    // Entry point
    GeoMapMissionLabel {
        scene: root.scene
        surfaceModel: root.surfaceModel
        coordinate: root.item.coordinate
        index: root.item.sequenceNumber
        label: qsTr("Entry")
        checked: root.item.isCurrentItem
        visible: root.item.exitCoordinate.isValid
    }

    // Exit point
    GeoMapMissionLabel {
        scene: root.scene
        surfaceModel: root.surfaceModel
        coordinate: root.item.exitCoordinate
        index: root.item.lastSequenceNumber
        label: qsTr("Exit")
        checked: root.item.isCurrentItem
        visible: root.item.exitCoordinate.isValid
    }
}
