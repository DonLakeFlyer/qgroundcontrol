#include "MavlinkSettingsTest.h"

#include <QtCore/QSettings>

#include "MavlinkSettings.h"

namespace {
constexpr const char* kDeprecatedNoInitialDownloadKey = "noInitialDownloadWhenFlying";
}

void MavlinkSettingsTest::init()
{
    UnitTest::init();

    QSettings settings;
    settings.beginGroup(MavlinkSettings::settingsGroup);
    _hadDeprecatedNoInitialDownload = settings.contains(kDeprecatedNoInitialDownloadKey);
    _savedDeprecatedNoInitialDownload = settings.value(kDeprecatedNoInitialDownloadKey);
    _hadNoInitialDownloadWhenArmed = settings.contains(MavlinkSettings::noInitialDownloadWhenArmedName);
    _savedNoInitialDownloadWhenArmed = settings.value(MavlinkSettings::noInitialDownloadWhenArmedName);
    settings.endGroup();
}

void MavlinkSettingsTest::cleanup()
{
    QSettings settings;
    settings.beginGroup(MavlinkSettings::settingsGroup);
    if (_hadDeprecatedNoInitialDownload) {
        settings.setValue(kDeprecatedNoInitialDownloadKey, _savedDeprecatedNoInitialDownload);
    } else {
        settings.remove(kDeprecatedNoInitialDownloadKey);
    }
    if (_hadNoInitialDownloadWhenArmed) {
        settings.setValue(MavlinkSettings::noInitialDownloadWhenArmedName, _savedNoInitialDownloadWhenArmed);
    } else {
        settings.remove(MavlinkSettings::noInitialDownloadWhenArmedName);
    }
    settings.endGroup();

    UnitTest::cleanup();
}

void MavlinkSettingsTest::_noInitialDownloadWhenFlyingMigration()
{
    QSettings settings;
    settings.beginGroup(MavlinkSettings::settingsGroup);
    settings.remove(MavlinkSettings::noInitialDownloadWhenArmedName);
    settings.setValue(kDeprecatedNoInitialDownloadKey, true);
    settings.endGroup();

    // Migration runs in the constructor. SettingsFacts ignore QSettings under unit tests,
    // so assert against the raw stored values rather than the facts.
    const MavlinkSettings mavlinkSettings;
    settings.beginGroup(MavlinkSettings::settingsGroup);
    QVERIFY(settings.contains(MavlinkSettings::noInitialDownloadWhenArmedName));
    QCOMPARE(settings.value(MavlinkSettings::noInitialDownloadWhenArmedName).toBool(), true);
    QVERIFY(!settings.contains(kDeprecatedNoInitialDownloadKey));
    settings.endGroup();
}

UT_REGISTER_TEST(MavlinkSettingsTest, TestLabel::Unit)
