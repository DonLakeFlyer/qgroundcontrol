#pragma once

#include "VehicleConfigUITestBase.h"

/// UI test that connects an ArduPilot MockLink started with the
/// OptionAPMStartFreshParams option (freshly flashed firmware state: no
/// airframe selected, compass/accel/radio uncalibrated) and verifies that
/// Frame, Sensors and Radio all report setup required, that Frame is the
/// required prerequisite for the Sensors and Radio pages, and that selecting
/// a frame class unblocks them.
class APMFreshFlashUITest : public VehicleConfigUITestBase
{
    Q_OBJECT

public:
    APMFreshFlashUITest() = default;

private slots:
    void _testFreshFlashSetupState();

private:
    /// Click the given component button in the config sidebar and verify
    /// whether the prerequisite-setup message panel is shown.
    void _verifyFramePrereq(const QString &compObjectName, bool expectPrereqShown);

    /// Verify the sidebar button for the given component shows the red
    /// setup-required icon (ConfigButton turns icon.color red when
    /// setupComplete is false).
    void _verifySetupIndicator(const QString &compObjectName, bool expectSetupComplete);
};
