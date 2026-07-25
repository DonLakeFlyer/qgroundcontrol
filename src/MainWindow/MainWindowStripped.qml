// HACK: AX12 debugging - minimal window mirroring window-test/main.qml
import QtQuick
import QtQuick.Controls
import QtQuick.Window

ApplicationWindow {
    id: root
    visible: true
    visibility: Window.FullScreen

    Rectangle {
        anchors.fill: parent
        color: "steelblue"

        Text {
            anchors.centerIn: parent
            font.pixelSize: 30
            color: "white"
            text: "win: " + root.width + "x" + root.height
                  + "\nScreen: " + Screen.width + "x" + Screen.height
                  + "\navail: " + Screen.desktopAvailableWidth + "x" + Screen.desktopAvailableHeight
        }
    }
}
