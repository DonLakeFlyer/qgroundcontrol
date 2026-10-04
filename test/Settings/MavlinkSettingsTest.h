#pragma once

#include <QtCore/QVariant>

#include "UnitTest.h"

class MavlinkSettingsTest : public UnitTest
{
    Q_OBJECT

private slots:
    void init() override;
    void cleanup() override;

    void _noInitialDownloadWhenFlyingMigration();

private:
    QVariant _savedDeprecatedNoInitialDownload;
    QVariant _savedNoInitialDownloadWhenArmed;
    bool _hadDeprecatedNoInitialDownload = false;
    bool _hadNoInitialDownloadWhenArmed = false;
};
