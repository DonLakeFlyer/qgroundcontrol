"""Generate QML vehicle config pages from JSON definitions.

Each page definition describes sections with controls that bind to vehicle
parameters via FactPanelController.  Sections can either list individual
controls (which are code-generated) or reference a hand-written QML
``component`` (escape-hatch for complex UIs).
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

from ..common.controls import (
    EnableCheckboxDef,
    ButtonDef,
    DialogButtonDef,
    ActionButtonDef,
    RadioOptionDef,
    ToggleCheckboxDef,
    parse_enable_checkbox,
    parse_button,
    parse_dialog_button,
    parse_action_button,
    parse_radio_options,
    parse_toggle_checkbox,
    render_label,
    render_slider,
    render_checkbox,
    render_combobox,
    render_textfield,
    render_radiogroup,
    render_dialog_button,
    render_action_button,
    render_bitmask_checkbox,
    render_bitmask,
    render_toggle_checkbox,
)


# --------------------------------------------------------------------------- #
# Data model
# --------------------------------------------------------------------------- #

@dataclass
class ControlDef:
    """A single control inside a config section."""
    param: str = ""            # vehicle parameter name via FactPanelController
    setting: str = ""          # settings path, e.g. "flyViewSettings.showObstacleDistanceOverlay"
    label: str = ""
    control: str = ""          # combobox | textfield | checkbox | slider | dialogButton (auto-detected if empty)
    showWhen: str = ""
    enableWhen: str = ""
    optional: bool = False     # true: param may not exist, pass false to getParameterFact
    sliderMin: str = ""        # explicit slider min override
    sliderMax: str = ""        # explicit slider max override
    enableCheckbox: EnableCheckboxDef | None = None  # slider: checked/onClicked
    button: ButtonDef | None = None                  # adjacent button
    options: list[RadioOptionDef] = field(default_factory=list)  # radiogroup options
    dialogButton: DialogButtonDef | None = None      # button that opens a popup dialog
    actionButton: ActionButtonDef | None = None       # standalone button calling a method
    warning: bool = False                            # label: use warning color
    raw: bool = False                                # radiogroup: use rawValue instead of value
    bitMask: int = 0                                 # bitmaskCheckbox: the bitmask value
    firstEntryIsAll: bool = False                    # bitmask: first entry is "all" toggle
    toggleCheckbox: ToggleCheckboxDef | None = None  # toggleCheckbox: custom checked/onClicked


@dataclass
class RepeatDef:
    """Repeat a section for each indexed parameter instance."""
    paramPrefix: str = ""            # e.g. "BAT"
    probePostfix: str = ""           # e.g. "_SOURCE" — used to discover count
    startIndex: int = 1
    firstIndexOmitsNumber: bool = False  # when True, index 1 -> "BAT" not "BAT1"
    indexing: str = ""               # custom indexing mode, e.g. "apm_battery"


@dataclass
class SectionDef:
    """A section within a config page — either generated or a component escape."""
    name: str = ""
    image: str = ""            # qrc path for section icon
    controls: list[ControlDef] = field(default_factory=list)
    component: str = ""        # escape hatch: hand-written QML component name
    showWhen: str = ""
    repeat: RepeatDef | None = None  # repeat for indexed params


@dataclass
class PageDef:
    """Top-level page definition loaded from JSON."""
    bindings: dict[str, str] = field(default_factory=dict)  # name -> QML expression
    sections: list[SectionDef] = field(default_factory=list)
    imports: list[str] = field(default_factory=list)  # extra QML import lines
    controllerType: str = "FactPanelController"  # QML type for the controller


# --------------------------------------------------------------------------- #
# JSON loading
# --------------------------------------------------------------------------- #

def load_page_def(json_path: Path) -> PageDef:
    """Load a config page definition from a JSON file."""
    with open(json_path, encoding="utf-8") as f:
        data = json.load(f)

    bindings = data.get("bindings", {})
    extra_imports = data.get("imports", [])
    sections: list[SectionDef] = []
    for sec_data in data.get("sections", []):
        controls: list[ControlDef] = []
        for ctrl_data in sec_data.get("controls", []):
            controls.append(ControlDef(
                param=ctrl_data.get("param", ""),
                setting=ctrl_data.get("setting", ""),
                label=ctrl_data.get("label", ""),
                control=ctrl_data.get("control", ""),
                showWhen=ctrl_data.get("showWhen", ""),
                enableWhen=ctrl_data.get("enableWhen", ""),
                optional=ctrl_data.get("optional", False),
                sliderMin=str(ctrl_data.get("sliderMin", "")),
                sliderMax=str(ctrl_data.get("sliderMax", "")),
                enableCheckbox=parse_enable_checkbox(ctrl_data.get("enableCheckbox")),
                button=parse_button(ctrl_data.get("button")),
                options=parse_radio_options(ctrl_data.get("options")),
                dialogButton=parse_dialog_button(ctrl_data.get("dialogButton")),
                actionButton=parse_action_button(ctrl_data.get("actionButton")),
                warning=ctrl_data.get("warning", False),
                raw=ctrl_data.get("raw", False),
                bitMask=ctrl_data.get("bitMask", 0),
                firstEntryIsAll=ctrl_data.get("firstEntryIsAll", False),
                toggleCheckbox=parse_toggle_checkbox(ctrl_data.get("toggleCheckbox")),
            ))
        repeat_data = sec_data.get("repeat")
        repeat_def = None
        if repeat_data:
            repeat_def = RepeatDef(
                paramPrefix=repeat_data.get("paramPrefix", ""),
                probePostfix=repeat_data.get("probePostfix", ""),
                startIndex=repeat_data.get("startIndex", 1),
                firstIndexOmitsNumber=repeat_data.get("firstIndexOmitsNumber", False),
                indexing=repeat_data.get("indexing", ""),
            )
        sections.append(SectionDef(
            name=sec_data.get("name", ""),
            image=sec_data.get("image", ""),
            controls=controls,
            component=sec_data.get("component", ""),
            showWhen=sec_data.get("showWhen", ""),
            repeat=repeat_def,
        ))
    return PageDef(bindings=bindings, sections=sections, imports=extra_imports,
                   controllerType=data.get("controllerType", "FactPanelController"))


# --------------------------------------------------------------------------- #
# QML generation helpers
# --------------------------------------------------------------------------- #

_HEADER = """\
// This file is auto-generated. Do not edit.
import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import QGroundControl
import QGroundControl.FactControls
import QGroundControl.Controls
"""


def _detect_control_type(ctrl: ControlDef) -> str:
    """Detect the best control type for a parameter when not explicitly set."""
    if ctrl.control:
        return ctrl.control
    # Default to textfield; combobox/checkbox can be specified explicitly
    return "textfield"


def _fact_ref(ctrl: ControlDef, indexed: bool = False) -> str:
    """Return the QML expression for the control's fact binding.

    When *indexed* is True the control lives inside a Repeater delegate and
    ``ctrl.param`` is a postfix (e.g. ``_SOURCE``).  The full param name is
    built at runtime from ``_paramName`` which is a JS function defined by
    the repeat section.
    """
    if ctrl.setting:
        return f"QGroundControl.settingsManager.{ctrl.setting}"
    if indexed:
        if ctrl.optional:
            return f'controller.getParameterFact(-1, _paramName("{ctrl.param}"), false)'
        return f'controller.getParameterFact(-1, _paramName("{ctrl.param}"))'
    if ctrl.optional:
        return f'controller.getParameterFact(-1, "{ctrl.param}", false)'
    return f'controller.getParameterFact(-1, "{ctrl.param}")'


def _qml_control(ctrl: ControlDef, indent: str, *, indexed: bool = False, dialog_counter: list[int] | None = None) -> str:
    """Generate QML for a single control inside a ConfigSection ColumnLayout."""
    fact_ref = _fact_ref(ctrl, indexed=indexed)
    control_type = _detect_control_type(ctrl)

    # Static label — no fact binding
    if control_type == "label":
        qml = render_label(
            indent,
            text=ctrl.label,
            warning=ctrl.warning,
        )
        if ctrl.showWhen:
            qml = _inject_prop(qml, f"{indent}    visible: {ctrl.showWhen}")
        return qml

    # Dialog button — standalone button that opens a popup dialog
    if control_type == "dialogButton" and ctrl.dialogButton:
        if dialog_counter is None:
            dialog_counter = [0]
        factory_id = f"_dlgFactory{dialog_counter[0]}"
        dialog_counter[0] += 1

        parts: list[str] = []

        # If the control also has a param, render textfield + button in a RowLayout
        if ctrl.param:
            # Factory and component go first (siblings, not inside the row)
            comp_id = f"{factory_id}Component"
            parts.append(f"{indent}QGCPopupDialogFactory {{")
            parts.append(f"{indent}    id: {factory_id}")
            parts.append(f"{indent}    dialogComponent: {comp_id}")
            parts.append(f"{indent}}}")
            parts.append(f"{indent}Component {{")
            parts.append(f"{indent}    id: {comp_id}")
            parts.append(f"{indent}    {ctrl.dialogButton.dialogComponent} {{ }}")
            parts.append(f"{indent}}}")

            # RowLayout with textfield + button
            params_js = ", ".join(
                f'"{k}": {v}' for k, v in ctrl.dialogButton.dialogParams.items()
            )
            open_arg = f"{{ {params_js} }}" if params_js else ""

            ri = indent + "    "  # row-inner indent

            parts.append(f"{indent}RowLayout {{")
            parts.append(f"{indent}    Layout.fillWidth: true")
            parts.append(f"{indent}    spacing: ScreenTools.defaultFontPixelWidth")

            if not ctrl.dialogButton.buttonAfter:
                # Label (fill width), Button, plain FactTextField
                label_text = ctrl.label if ctrl.label else f"{fact_ref}.shortDescription"
                label_qtr = f'qsTr("{label_text}")' if ctrl.label else label_text
                parts.append(f'{ri}QGCLabel {{')
                parts.append(f'{ri}    text: {label_qtr}')
                parts.append(f'{ri}    Layout.fillWidth: true')
                parts.append(f'{ri}}}')
                parts.append(f'{ri}QGCButton {{')
                parts.append(f'{ri}    text: qsTr("{ctrl.dialogButton.text}")')
                parts.append(f'{ri}    onClicked: {factory_id}.open({open_arg})')
                if ctrl.enableWhen:
                    parts.append(f'{ri}    enabled: {ctrl.enableWhen}')
                parts.append(f'{ri}}}')
                parts.append(f'{ri}FactTextField {{')
                parts.append(f'{ri}    fact: {fact_ref}')
                if ctrl.enableWhen:
                    parts.append(f'{ri}    enabled: {ctrl.enableWhen}')
                parts.append(f'{ri}}}')
            else:
                # LabelledFactTextField, then Button
                tf = render_textfield(
                    fact_ref, ri,
                    label=ctrl.label,
                    enable_when=ctrl.enableWhen,
                    label_source=f"{fact_ref}.shortDescription",
                    qml_type="LabelledFactTextField",
                )
                parts.append(tf)
                parts.append(f'{ri}QGCButton {{')
                parts.append(f'{ri}    text: qsTr("{ctrl.dialogButton.text}")')
                parts.append(f'{ri}    onClicked: {factory_id}.open({open_arg})')
                if ctrl.enableWhen:
                    parts.append(f'{ri}    enabled: {ctrl.enableWhen}')
                parts.append(f'{ri}}}')

            parts.append(f"{indent}}}")

            qml = "\n".join(parts)
        else:
            qml = render_dialog_button(
                indent,
                dialog_button=ctrl.dialogButton,
                factory_id=factory_id,
                enable_when=ctrl.enableWhen,
            )

        if ctrl.showWhen:
            # Wrap in a ColumnLayout with visibility
            qml = f"{indent}ColumnLayout {{\n{indent}    visible: {ctrl.showWhen}\n" + \
                  "\n".join(f"    {l}" for l in qml.splitlines()) + \
                  f"\n{indent}}}"
        return qml

    # Action button — standalone button calling a controller method
    if control_type == "actionButton" and ctrl.actionButton:
        qml = render_action_button(
            indent,
            action_button=ctrl.actionButton,
            enable_when=ctrl.enableWhen,
        )
        if ctrl.showWhen:
            qml = _inject_prop(qml, f"{indent}    visible: {ctrl.showWhen}")
        return qml

    # Slider — has its own label
    if control_type == "slider":
        qml = render_slider(
            fact_ref, indent,
            label=ctrl.label,
            enable_checkbox=ctrl.enableCheckbox,
            button=ctrl.button,
            enable_when=ctrl.enableWhen,
            allow_using_min_max=True,
            slider_min=ctrl.sliderMin,
            slider_max=ctrl.sliderMax,
        )
        if ctrl.showWhen:
            qml = _inject_prop(qml, f"{indent}    visible: {ctrl.showWhen}")
        return qml

    # Radio group — label + radio buttons
    if control_type == "radiogroup":
        return render_radiogroup(
            fact_ref, indent,
            label=ctrl.label,
            options=ctrl.options,
            enable_when=ctrl.enableWhen,
            raw=ctrl.raw,
        )

    # Bitmask checkbox — FactBitMaskCheckBoxSlider
    if control_type == "bitmaskCheckbox":
        qml = render_bitmask_checkbox(
            fact_ref, indent,
            label=ctrl.label,
            bit_mask=ctrl.bitMask,
            enable_when=ctrl.enableWhen,
        )
        if ctrl.showWhen:
            qml = _inject_prop(qml, f"{indent}    visible: {ctrl.showWhen}")
        return qml

    # Bitmask — full FactBitmask widget
    if control_type == "bitmask":
        qml = render_bitmask(
            fact_ref, indent,
            first_entry_is_all=ctrl.firstEntryIsAll,
            enable_when=ctrl.enableWhen,
        )
        if ctrl.showWhen:
            qml = _inject_prop(qml, f"{indent}    visible: {ctrl.showWhen}")
        return qml

    # Toggle checkbox — QGCCheckBoxSlider with custom logic
    if control_type == "toggleCheckbox" and ctrl.toggleCheckbox:
        qml = render_toggle_checkbox(
            indent,
            label=ctrl.label,
            toggle=ctrl.toggleCheckbox,
            enable_when=ctrl.enableWhen,
        )
        if ctrl.showWhen:
            qml = _inject_prop(qml, f"{indent}    visible: {ctrl.showWhen}")
        return qml

    # Checkbox — FactCheckBoxSlider includes label
    if control_type == "checkbox":
        qml = render_checkbox(
            fact_ref, indent,
            label=ctrl.label,
            enable_when=ctrl.enableWhen,
            label_source=f"{fact_ref}.shortDescription",
            qml_type="FactCheckBoxSlider",
        )
        if ctrl.showWhen:
            qml = _inject_prop(qml, f"{indent}    visible: {ctrl.showWhen}")
        return qml

    # Combobox — LabelledFactComboBox includes label
    if control_type == "combobox":
        qml = render_combobox(
            fact_ref, indent,
            label=ctrl.label,
            enable_when=ctrl.enableWhen,
            label_source=f"{fact_ref}.shortDescription",
            qml_type="LabelledFactComboBox",
            combo_preferred_width="ScreenTools.defaultFontPixelWidth * 30",
        )
        if ctrl.showWhen:
            qml = _inject_prop(qml, f"{indent}    visible: {ctrl.showWhen}")
        return qml

    # Textfield — LabelledFactTextField includes label
    qml = render_textfield(
        fact_ref, indent,
        label=ctrl.label,
        enable_when=ctrl.enableWhen,
        label_source=f"{fact_ref}.shortDescription",
        qml_type="LabelledFactTextField",
    )
    if ctrl.showWhen:
        qml = _inject_prop(qml, f"{indent}    visible: {ctrl.showWhen}")
    return qml


def _inject_prop(qml: str, prop_line: str) -> str:
    """Insert a property line after the opening brace of a QML block."""
    qml_lines = qml.split("\n")
    qml_lines.insert(1, prop_line)
    return "\n".join(qml_lines)


def _wrap_visible(qml: str, expr: str, indent: str) -> str:
    """Wrap a QML block in an Item with a visible binding."""
    lines = [f"{indent}Item {{"]
    lines.append(f"{indent}    visible: {expr}")
    lines.append(f"{indent}    Layout.fillWidth: true")
    for line in qml.splitlines():
        lines.append(f"    {line}" if line.strip() else line)
    lines.append(f"{indent}}}")
    return "\n".join(lines)


def _qml_generated_section(sec: SectionDef, sec_idx: int) -> str:
    """Generate QML for a section with auto-generated controls."""
    ind = "                "  # base indent inside outerColumn
    lines: list[str] = []

    name_vis = f'(sectionNameFilter === "" || sectionNameFilter === qsTr("{sec.name}"))'
    show_vis = f"{name_vis} && {sec.showWhen}" if sec.showWhen else name_vis

    lines.append(f'{ind}ConfigSection {{')
    lines.append(f'{ind}    Layout.fillWidth: true')
    lines.append(f'{ind}    visible: {show_vis}')
    lines.append(f'{ind}    heading: qsTr("{sec.name}")')
    if sec.image:
        lines.append(f'{ind}    iconSource: "{sec.image}"')

    ctrl_indent = ind + "    "
    for ctrl in sec.controls:
        lines.append("")
        lines.append(_qml_control(ctrl, ctrl_indent))

    lines.append(f'{ind}}}')

    return "\n".join(lines)


def _qml_component_section(sec: SectionDef, sec_idx: int) -> str:
    """Generate QML for a component escape-hatch section."""
    ind = "                "
    lines: list[str] = []

    name_vis = f'(sectionNameFilter === "" || sectionNameFilter === qsTr("{sec.name}"))'
    show_vis = f"{name_vis} && {sec.showWhen}" if sec.showWhen else name_vis

    lines.append(f'{ind}QGCLabel {{')
    lines.append(f'{ind}    text: qsTr("{sec.name}") + " ⚙"')
    lines.append(f'{ind}    visible: {show_vis}')
    lines.append(f'{ind}}}')

    # Wrap escape hatch in a Row with a colored left bar
    lines.append(f'{ind}Row {{')
    lines.append(f'{ind}    visible: {show_vis}')
    lines.append(f'{ind}    spacing: 0')
    lines.append(f'{ind}    Rectangle {{')
    lines.append(f'{ind}        width: 3')
    lines.append(f'{ind}        height: parent.height')
    lines.append(f'{ind}        color: "orange"')
    lines.append(f'{ind}    }}')
    lines.append(f'{ind}    {sec.component} {{')
    lines.append(f'{ind}        controller: controller')
    lines.append(f'{ind}    }}')
    lines.append(f'{ind}}}')

    return "\n".join(lines)


def _safe_id(name: str) -> str:
    """Convert a section name to a safe QML identifier."""
    return "".join(c if c.isalnum() else "_" for c in name).lower()


def _qml_repeat_count_property(sec: SectionDef) -> str:
    """Generate the count property for a repeat section (emitted at Item level)."""
    rep = sec.repeat
    assert rep is not None
    ind = "            "  # Item-level indent
    safe = _safe_id(sec.name)

    if rep.indexing == "apm_battery":
        # APM battery indexing: 0->"BATT_", 1->"BATT2_", ..., 8->"BATT9_", 9->"BATTA_", ...
        lines: list[str] = []
        lines.append(f"{ind}function _battPrefixForIndex(_i) {{")
        lines.append(f'{ind}    if (_i === 0) return "{rep.paramPrefix}_"')
        lines.append(f'{ind}    if (_i <= 8) return "{rep.paramPrefix}" + (_i + 1) + "_"')
        lines.append(f'{ind}    return "{rep.paramPrefix}" + String.fromCharCode(65 + _i - 9) + "_"')
        lines.append(f"{ind}}}")
        lines.append(f"{ind}function _battLabelForIndex(_i) {{")
        lines.append(f"{ind}    if (_i <= 8) return String(_i + 1)")
        lines.append(f"{ind}    return String.fromCharCode(65 + _i - 9)")
        lines.append(f"{ind}}}")
        lines.append(f"{ind}property int _{safe}Count: {{")
        lines.append(f"{ind}    var _i = 0")
        lines.append(f"{ind}    while (_i < 16) {{")
        lines.append(f'{ind}        if (!controller.parameterExists(-1, _battPrefixForIndex(_i) + "{rep.probePostfix}"))')
        lines.append(f"{ind}            return _i")
        lines.append(f"{ind}        _i++")
        lines.append(f"{ind}    }}")
        lines.append(f"{ind}    return 16")
        lines.append(f"{ind}}}")
        return "\n".join(lines)

    probe_param = f'"{rep.paramPrefix}" + _idx + "{rep.probePostfix}"'
    if rep.firstIndexOmitsNumber:
        idx_expr = f'(_i === {rep.startIndex}) ? "" : _i'
    else:
        idx_expr = "_i"

    lines: list[str] = []
    lines.append(f"{ind}property int _{safe}Count: {{")
    lines.append(f"{ind}    var _i = {rep.startIndex}")
    lines.append(f"{ind}    while (true) {{")
    lines.append(f"{ind}        var _idx = {idx_expr}")
    lines.append(f"{ind}        if (!controller.parameterExists(-1, {probe_param}))")
    lines.append(f"{ind}            return _i - {rep.startIndex}")
    lines.append(f"{ind}        _i++")
    lines.append(f"{ind}    }}")
    lines.append(f"{ind}}}")
    return "\n".join(lines)


def _qml_repeat_section(sec: SectionDef, sec_idx: int) -> str:
    """Generate QML for a repeated (indexed) section using a Repeater."""
    rep = sec.repeat
    assert rep is not None
    ind = "                "  # base indent inside outerColumn

    name_vis = f'(sectionNameFilter === "" || sectionNameFilter === heading)'
    show_vis = f"{name_vis} && {sec.showWhen}" if sec.showWhen else name_vis
    safe = _safe_id(sec.name)

    lines: list[str] = []

    # Repeater
    lines.append(f"{ind}Repeater {{")
    lines.append(f"{ind}    model: _{safe}Count")
    lines.append("")
    lines.append(f"{ind}    ConfigSection {{")
    lines.append(f"{ind}        Layout.fillWidth: true")
    lines.append(f"{ind}        visible: {show_vis}")

    # heading — include index when count > 1
    heading_base = sec.name.replace("{index}", '" + _displayIndex + "')
    if "{index}" in sec.name:
        lines.append(f'{ind}        heading: qsTr("{heading_base}")')
    else:
        lines.append(f'{ind}        heading: _{safe}Count > 1 ? qsTr("{sec.name}") + " " + _displayIndex : qsTr("{sec.name}")')

    if sec.image:
        lines.append(f'{ind}        iconSource: "{sec.image}"')

    # Internal properties for index math
    lines.append("")
    if rep.indexing == "apm_battery":
        lines.append(f"{ind}        property int _rawIndex: index")
        lines.append(f"{ind}        property string _prefix: _battPrefixForIndex(_rawIndex)")
        lines.append(f"{ind}        property string _displayIndex: _battLabelForIndex(_rawIndex)")
        lines.append(f'{ind}        function _paramName(postfix) {{ return _prefix + postfix }}')
    else:
        lines.append(f"{ind}        property int _rawIndex: index + {rep.startIndex}")
        if rep.firstIndexOmitsNumber:
            lines.append(f'{ind}        property string _indexStr: (_rawIndex === {rep.startIndex}) ? "" : String(_rawIndex)')
        else:
            lines.append(f"{ind}        property string _indexStr: String(_rawIndex)")
        lines.append(f'{ind}        property string _displayIndex: String(_rawIndex)')
        lines.append(f'{ind}        function _paramName(postfix) {{ return "{rep.paramPrefix}" + _indexStr + postfix }}')

    ctrl_indent = ind + "        "
    dialog_counter = [0]
    for ctrl in sec.controls:
        lines.append("")
        lines.append(_qml_control(ctrl, ctrl_indent, indexed=True, dialog_counter=dialog_counter))

    lines.append(f"{ind}    }}")
    lines.append(f"{ind}}}")

    return "\n".join(lines)


# --------------------------------------------------------------------------- #
# Public API
# --------------------------------------------------------------------------- #

def generate_config_page_qml(page: PageDef) -> str:
    """Generate a complete QML file for a vehicle config page."""
    lines: list[str] = [_HEADER]
    for imp in page.imports:
        lines.append(f"import {imp}")
    if page.imports:
        lines.append("")
    lines.append("SetupPage {")
    lines.append("    id: configPage")
    lines.append("    pageComponent: pageComponent")
    lines.append("")
    lines.append("    Component {")
    lines.append("        id: pageComponent")
    lines.append("")
    lines.append("        Item {")
    lines.append("            width: Math.max(availableWidth, outerColumn.width)")
    lines.append("            height: outerColumn.height")
    lines.append("")
    lines.append(f"            {page.controllerType} {{")
    lines.append("                id: controller")
    lines.append("            }")
    lines.append("")
    lines.append("            property real _margins: ScreenTools.defaultFontPixelHeight")
    lines.append("")

    # Emit page-level bindings as QML properties
    for name, expr in page.bindings.items():
        lines.append(f"            property var {name}: {expr}")
    if page.bindings:
        lines.append("")

    # Emit repeat count properties at Item level (visible to Repeater delegates)
    for sec in page.sections:
        if sec.repeat:
            lines.append(_qml_repeat_count_property(sec))
            lines.append("")

    lines.append("            property string sectionNameFilter: \"\"")
    lines.append("")
    lines.append("            function sectionVisible(name) {")
    # Build a switch that returns visibility per section.
    # Merge showWhen conditions for sections that share the same name
    # so that the section is visible when ANY variant matches.
    visible_conditions: dict[str, list[str]] = {}
    for sec in page.sections:
        if sec.showWhen:
            visible_conditions.setdefault(sec.name, []).append(sec.showWhen)
    for name, conditions in visible_conditions.items():
        merged = " || ".join(dict.fromkeys(conditions))  # deduplicate, preserve order
        lines.append(f'                if (name === qsTr("{name}")) return {merged}')
    lines.append("                return true")
    lines.append("            }")
    lines.append("")
    lines.append("            ColumnLayout {")
    lines.append("                id: outerColumn")
    lines.append("                spacing: _margins * 1.25")
    lines.append("                anchors.horizontalCenter: parent.horizontalCenter")

    for sec_idx, sec in enumerate(page.sections):
        lines.append("")
        if sec.repeat:
            lines.append(_qml_repeat_section(sec, sec_idx))
        elif sec.component:
            lines.append(_qml_component_section(sec, sec_idx))
        else:
            lines.append(_qml_generated_section(sec, sec_idx))

    lines.append("            }")
    lines.append("        }")
    lines.append("    }")
    lines.append("}")
    lines.append("")

    return "\n".join(lines)


def get_section_names(page: PageDef) -> list[str]:
    """Return the display names of all sections in a page."""
    return [sec.name for sec in page.sections if sec.name]
