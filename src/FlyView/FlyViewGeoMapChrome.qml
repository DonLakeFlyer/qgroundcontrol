/****************************************************************************
 *
 * (c) 2009-2024 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

import QtQuick

import QGroundControl
import QGroundControl.Controls
import QGroundControl.GeoMap

/// View chrome for a FlyViewGeoMap: 2D/3D mode switch, compass and diagnostic
/// buttons plus the terrain attribution/stats overlay. Shared by the
/// standalone GeoView (FlyViewGeo) and the Fly View GeoMap engine overlay.
Item {
    id: root

    required property var geoMap

    // Distinguishes objectNames when both the GeoView and the Fly View engine
    // overlay instantiate this chrome (UI tests look items up by objectName)
    property string objectNamePrefix: "flyViewGeo"

    // Margins let the Fly View host push the chrome clear of its own widgets
    property real buttonsTopMargin: ScreenTools.defaultFontPixelWidth
    property real buttonsRightMargin: ScreenTools.defaultFontPixelWidth
    property real attributionBottomMargin: ScreenTools.defaultFontPixelWidth / 2

    // Bottom-left overlay: dataset attribution (required by the terrain
    // tiles' terms) plus live SurfaceModel stats for manual verification;
    // last line adds the perf counters while Stats is on
    Rectangle {
        anchors.left: parent.left
        anchors.bottom: parent.bottom
        anchors.leftMargin: ScreenTools.defaultFontPixelWidth / 2
        anchors.bottomMargin: root.attributionBottomMargin
        width: debugOverlayLabel.implicitWidth + ScreenTools.defaultFontPixelWidth
        height: debugOverlayLabel.implicitHeight + ScreenTools.defaultFontPixelWidth
        radius: ScreenTools.defaultFontPixelWidth / 2
        color: Qt.rgba(0, 0, 0, 0.5)

        QGCLabel {
            id: debugOverlayLabel
            objectName: root.objectNamePrefix + "DebugOverlay"
            anchors.centerIn: parent
            font.family: ScreenTools.fixedFontFamily
            color: "white"
            text: qsTr("Terrain: Mapzen/Tilezen Terrain Tiles — SRTM (NASA), 3DEP (USGS), GMTED2010, ETOPO1 (NOAA)")
                  + "\n" + qsTr("patches: %1  pending: %2  max zoom: %3")
                      .arg(root.geoMap.patchCount)
                      .arg(root.geoMap.pendingCount)
                      .arg(root.geoMap.maxZoomLevel)
                  + (root.geoMap.modelStats !== "" ? "\n" + root.geoMap.modelStats : "")
        }
    }

    Column {
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.topMargin: root.buttonsTopMargin
        anchors.rightMargin: root.buttonsRightMargin
        spacing: ScreenTools.defaultFontPixelHeight / 2

        // 2D<->3D mode switch: the mode flips immediately (2D locks tilt
        // gestures); GeoMap's mode-change handler animates tilt and terrain
        QGCButton {
            objectName: root.objectNamePrefix + "ModeButton"
            text: root.geoMap.camera.mode === GeoMapCamera.Mode2D ? qsTr("3D") : qsTr("2D")
            onClicked: root.geoMap.camera.mode = (root.geoMap.camera.mode === GeoMapCamera.Mode2D)
                           ? GeoMapCamera.Mode3D : GeoMapCamera.Mode2D
        }

        // Compass: needle tracks heading, click animates back to north-up
        QGCButton {
            objectName: root.objectNamePrefix + "CompassButton"
            text: qsTr("N")
            rotation: -root.geoMap.camera.heading
            onClicked: root.geoMap.animateHeadingToNorth()
        }

        // Diagnostic: report holes, terrain cliffs, and bad height
        // data to the debug output
        QGCButton {
            objectName: root.objectNamePrefix + "AnalyzeButton"
            text: qsTr("Analyze")
            onClicked: root.geoMap.analyzeSurface()
        }

        // Render-statistics overlay toggle (perf diagnostics)
        QGCButton {
            objectName: root.objectNamePrefix + "StatsButton"
            text: qsTr("Stats")
            onClicked: root.geoMap.renderStats = !root.geoMap.renderStats
        }

        // Perf capture: records per-second counters; the CSV path
        // shows in the debug overlay when stopped
        QGCButton {
            objectName: root.objectNamePrefix + "RecordButton"
            text: root.geoMap.perfCapturing ? qsTr("Stop") : qsTr("Record")
            onClicked: root.geoMap.perfCapturing ? root.geoMap.stopPerfCapture() : root.geoMap.startPerfCapture()
        }
    }
}
