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

/// Base visual for Survey and Corridor Scan on GeoMap: read-only GeoMap-native
/// port of src/PlanView/TransectStyleMapVisuals.qml. No dragging/editing (the
/// mission is already loaded and immutable in FlyView), so the polygon/vertex
/// interactivity and opacity-dimming machinery from the source file are
/// dropped; the visual output (polygon fill, transect lines, entry/exit
/// markers and direction arrows) is otherwise a direct port.
Item {
    id: root

    property var item          ///< VisualMissionItem (TransectStyleComplexItem)
    property var scene
    property var surfaceModel

    readonly property var _mapPolygon: item.surveyAreaPolygon
    readonly property bool _currentItem: item.isCurrentItem
    readonly property var _transectPoints: item.visualTransectPoints
    readonly property bool _hasTurnaround: item.turnAroundDistance.rawValue !== 0
    readonly property int _firstTrueTransectIndex: _hasTurnaround ? 1 : 0
    readonly property int _lastTrueTransectIndex: _transectPoints.length - (_hasTurnaround ? 2 : 1)
    readonly property int _lastPointIndex: _transectPoints.length - 1
    readonly property int _transectCount: _transectPoints.length / (_hasTurnaround ? 4 : 2)
    readonly property bool _showPartialEntryExit: !_currentItem && _hasTurnaround && _transectPoints.length >= 2

    // Screen-space interpolation along transectScreenPoints[fromIndex]->[toIndex]
    // at fraction (arrowPosition/4, matching MapLineArrow's quarter-based
    // positioning); null when the referenced indices aren't available yet
    function _arrowPoint(points, fromIndex, toIndex, fraction) {
        if (fromIndex < 0 || toIndex >= points.length) {
            return null
        }
        const from = points[fromIndex]
        const to = points[toIndex]
        return Qt.point(from.x + (to.x - from.x) * fraction, from.y + (to.y - from.y) * fraction)
    }

    function _arrowHeading(points, fromIndex, toIndex) {
        if (fromIndex < 0 || toIndex >= points.length) {
            return 0
        }
        const from = points[fromIndex]
        const to = points[toIndex]
        return Math.atan2(to.x - from.x, -(to.y - from.y)) * 180 / Math.PI
    }

    GeoMapProjectedPath {
        id: polygonPath
        scene: root.scene
        surfaceModel: root.surfaceModel
        coordinates: root._mapPolygon.path
    }

    // Area polygon
    Shape {
        visible: polygonPath.projected
        ShapePath {
            strokeColor: "black"
            strokeWidth: 1
            fillColor: root._mapPolygon.showAltColor ? QGroundControl.globalPalette.surveyPolygonTerrainCollision
                                                      : QGroundControl.globalPalette.surveyPolygonInterior
            fillOpacity: 0.5
            PathPolyline { path: polygonPath.screenPoints }
        }
    }

    GeoMapProjectedPath {
        id: transectPath
        scene: root.scene
        surfaceModel: root.surfaceModel
        coordinates: root._transectPoints
    }

    // Full set of transect lines. Shown when item is selected.
    Shape {
        visible: root._currentItem && transectPath.projected
        ShapePath {
            strokeColor: "white"
            strokeWidth: 2
            fillColor: "transparent"
            PathPolyline { path: transectPath.screenPoints }
        }
    }

    // Entry and exit transect lines only. Used when item is not selected.
    Shape {
        visible: root._showPartialEntryExit && transectPath.projected
        ShapePath {
            strokeColor: "white"
            strokeWidth: 2
            fillColor: "transparent"
            PathPolyline {
                path: root._showPartialEntryExit && transectPath.screenPoints.length > 1
                      ? [transectPath.screenPoints[0], transectPath.screenPoints[1]] : []
            }
        }
    }
    Shape {
        visible: root._showPartialEntryExit && transectPath.projected
        ShapePath {
            strokeColor: "white"
            strokeWidth: 2
            fillColor: "transparent"
            PathPolyline {
                path: root._showPartialEntryExit && transectPath.screenPoints.length > 1
                      ? [transectPath.screenPoints[root._lastPointIndex - 1], transectPath.screenPoints[root._lastPointIndex]] : []
            }
        }
    }

    // Entry point
    GeoMapMissionLabel {
        scene: root.scene
        surfaceModel: root.surfaceModel
        coordinate: root.item.coordinate
        index: root.item.sequenceNumber
        checked: root.item.isCurrentItem
        visible: root.item.exitCoordinate.isValid
    }

    // Exit point
    GeoMapMissionLabel {
        scene: root.scene
        surfaceModel: root.surfaceModel
        coordinate: root.item.exitCoordinate
        index: root.item.lastSequenceNumber
        checked: root.item.isCurrentItem
        visible: root.item.exitCoordinate.isValid
    }

    readonly property var _entryArrow1Point: root._arrowPoint(transectPath.screenPoints, root._firstTrueTransectIndex, root._firstTrueTransectIndex + 1, 0.25)
    readonly property real _entryArrow1Heading: root._arrowHeading(transectPath.screenPoints, root._firstTrueTransectIndex, root._firstTrueTransectIndex + 1)
    GeoMapMissionArrow {
        visible: root._currentItem && transectPath.projected && !!root._entryArrow1Point
        screenPoint: root._entryArrow1Point || Qt.point(0, 0)
        heading: root._entryArrow1Heading
    }

    readonly property int _entryArrow2FromIndex: root._firstTrueTransectIndex + (root._hasTurnaround ? 4 : 2)
    readonly property var _entryArrow2Point: root._arrowPoint(transectPath.screenPoints, root._entryArrow2FromIndex, root._entryArrow2FromIndex + 1, 0.25)
    readonly property real _entryArrow2Heading: root._arrowHeading(transectPath.screenPoints, root._entryArrow2FromIndex, root._entryArrow2FromIndex + 1)
    GeoMapMissionArrow {
        visible: root._currentItem && root._transectCount > 3 && transectPath.projected && !!root._entryArrow2Point
        screenPoint: root._entryArrow2Point || Qt.point(0, 0)
        heading: root._entryArrow2Heading
    }

    readonly property var _exitArrow1Point: root._arrowPoint(transectPath.screenPoints, root._lastTrueTransectIndex - 1, root._lastTrueTransectIndex, 0.75)
    readonly property real _exitArrow1Heading: root._arrowHeading(transectPath.screenPoints, root._lastTrueTransectIndex - 1, root._lastTrueTransectIndex)
    GeoMapMissionArrow {
        visible: root._currentItem && transectPath.projected && !!root._exitArrow1Point
        screenPoint: root._exitArrow1Point || Qt.point(0, 0)
        heading: root._exitArrow1Heading
    }

    readonly property int _exitArrow2ToIndex: root._lastTrueTransectIndex - (root._hasTurnaround ? 4 : 2)
    readonly property var _exitArrow2Point: root._arrowPoint(transectPath.screenPoints, root._exitArrow2ToIndex - 1, root._exitArrow2ToIndex, 3.25)
    readonly property real _exitArrow2Heading: root._arrowHeading(transectPath.screenPoints, root._exitArrow2ToIndex - 1, root._exitArrow2ToIndex)
    GeoMapMissionArrow {
        visible: root._currentItem && root._transectCount > 3 && transectPath.projected && !!root._exitArrow2Point
        screenPoint: root._exitArrow2Point || Qt.point(0, 0)
        heading: root._exitArrow2Heading
    }
}
