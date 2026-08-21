/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

import QtQuick
import QtPositioning

import QGroundControl
import QGroundControl.Controls
import QGroundControl.GeoMap

/// GeoMap-engine drop-in for FlyViewMap: hosts a FlyViewGeoMap and implements
/// the mapControl contract the Fly View overlays consume (pipState,
/// isSatelliteMap, toCoordinate/fromCoordinate, zoomLevel, ...). Interactive
/// map editing is 2D-only; 3D is view-only (see issue #14901).
Item {
    id: root

    // mapControl contract (parity with FlyViewMap)
    property Item pipView
    property Item pipState: _pipState
    property var rightPanelWidth
    property var planMasterController
    property bool pipMode: false // true: map is shown in a small pip mode
    property var toolInsets // Insets for the center viewport area
    property string mapName

    // Writable center (contract parity with FlyViewMap): assignments recentre
    // the camera; the Binding below keeps it tracking the camera afterwards
    property var center
    readonly property real zoomLevel: {
        // Viewport/FOV referenced so the invokable re-evaluates on resize
        geoMapControl.camera.viewportSize
        geoMapControl.camera.verticalFieldOfView
        return geoMapControl.camera.zoomLevelForDistance(geoMapControl.camera.distance)
    }
    readonly property bool isSatelliteMap: _mapTypeSetting.indexOf("Satellite") > -1 || _mapTypeSetting.indexOf("Hybrid") > -1
    property alias pinchZoomDisabledByVirtualJoysticks: geoMapControl.pinchZoomDisabledByVirtualJoysticks

    // Escape hatch for GeoMap-specific chrome/controls
    readonly property var geoMap: geoMapControl

    // Host-provided offset from the map top to where the toolInsets frame
    // starts (the widget layer sits below the toolbar)
    property real toolInsetsTopOffset: 0

    readonly property string _mapTypeSetting: QGroundControl.settingsManager.flightMapSettings.mapType.rawValue

    onCenterChanged: {
        if (center && center.isValid) {
            geoMapControl.camera.center = center
        }
    }

    // Screen point -> ground coordinate through the camera pick ray
    // (clipToViewPort accepted for signature parity, screen points are
    // always inside the viewport for the fly view use cases)
    function toCoordinate(point, clipToViewPort) {
        return geoMapControl.camera.coordinateAtScreenPoint(point)
    }

    // Geographic coordinate -> screen point; invalid/behind-camera projects
    // to an off-screen point so distance math degrades instead of throwing
    function fromCoordinate(coordinate, clipToViewPort) {
        const screenPos = geoMapControl.scene.screenPositionFor(coordinate)
        return (screenPos === undefined) ? Qt.point(-1, -1) : screenPos
    }

    FlyViewGeoMap {
        id: geoMapControl
        anchors.fill: parent
    }

    FlyViewGeoMapChrome {
        anchors.fill: parent
        geoMap: geoMapControl
        objectNamePrefix: "flyViewGeoMap"
        visible: !root.pipMode
        // Right edge below the instrument/photo-video panel
        buttonsTopMargin: root.toolInsetsTopOffset
                          + (root.toolInsets ? root.toolInsets.topEdgeRightInset : 0)
        attributionBottomMargin: (root.toolInsets ? root.toolInsets.bottomEdgeLeftInset : 0) + ScreenTools.defaultFontPixelWidth / 2
    }

    PipState {
        id: _pipState
        pipView: root.pipView
        isDark: _isFullWindowItemDark
    }

    Binding {
        target: root
        property: "center"
        value: geoMapControl.camera.center
        restoreMode: Binding.RestoreNone
    }
}
