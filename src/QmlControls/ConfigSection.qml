import QtQuick
import QtQuick.Layouts

import QGroundControl
import QGroundControl.Controls

RowLayout {
    id:      control
    spacing: ScreenTools.defaultFontPixelWidth * 2

    default property alias contentItem: _controlsColumn.data

    property string heading
    property string iconSource

    QGCPalette { id: qgcPal; colorGroupEnabled: true }

    Rectangle {
        width:             ScreenTools.defaultFontPixelWidth / 3
        Layout.fillHeight: true
        color:             qgcPal.windowShadeLight
    }

    ColumnLayout {
        Layout.fillWidth: true
        spacing:          ScreenTools.defaultFontPixelHeight / 2

        QGCLabel {
            text:           control.heading
            font.pointSize: ScreenTools.largeFontPointSize
            visible:        control.heading !== ""
        }

        RowLayout {
            Layout.fillWidth: true
            spacing:          ScreenTools.defaultFontPixelWidth * 2

            QGCColoredImage {
                Layout.preferredWidth:  ScreenTools.defaultFontPixelHeight * 6
                Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 3
                source:                 control.iconSource
                color:                  qgcPal.text
                fillMode:               Image.PreserveAspectFit
                visible:                control.iconSource !== ""
            }

            ColumnLayout {
                id:               _controlsColumn
                Layout.fillWidth: true
                spacing:          ScreenTools.defaultFontPixelHeight / 2
            }
        }
    }
}
