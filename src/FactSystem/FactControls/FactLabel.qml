/****************************************************************************
 *
 * (c) 2009-2020 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

import QGroundControl.Controls      1.0
import QGroundControl.FactSystem    1.0

QGCLabel {
    property Fact fact
    property bool showUnits:    true
    property bool unitsSpacer:  true

    font.strikeout: fact ? fact.telemetryLost : false
    text:           valueText()

    function valueText() {
        if (fact) {
            var str
            if (fact.enumStringValue.length !== 0) {
                str = fact.enumStringValue
            } else {
                str = fact.valueString
            }
            if (showUnits && fact.units.length !==0) {
                str += (unitsSpacer ? " " : "") + fact.units
            }
            return str
        } else {
            return qsTr("--.--")
        }
    }
}
