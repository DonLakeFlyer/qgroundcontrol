// Copyright (C) 2011-2012 Denis Shienkov <denis.shienkov@gmail.com>
// Copyright (C) 2011 Sergey Belyashov <Sergey.Belyashov@gmail.com>
// Copyright (C) 2012 Laszlo Papp <lpapp@kde.org>
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR LGPL-3.0-only OR GPL-2.0-only OR GPL-3.0-only

#include <AndroidSerial.h>
#include <QGCLoggingCategory.h>
#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QStringList>
#include <unistd.h>

#include "qserialport_p.h"
#include "qserialportinfo.h"
#include "qserialportinfo_p.h"

QGC_LOGGING_CATEGORY(QSerialPortInfo_AndroidLog, "Android.AndroidSerialPortInfo")

QT_BEGIN_NAMESPACE

QList<QSerialPortInfo> availablePortsByFiltersOfDevices(bool& ok)
{
    const QList<QSerialPortInfo> serialPortInfoList = AndroidSerial::availableDevices();
    ok = !serialPortInfoList.isEmpty();
    return serialPortInfoList;
}

// Enumerates internal (non-USB) hardware UARTs such as /dev/ttyS1. On stock Android these
// device nodes are not accessible to apps, so the access() filter makes this self-hiding.
// Devices like the RadioMaster AX12 grant access in their system image.
QList<QSerialPortInfo> availablePortsBySysfs(bool& ok)
{
    QList<QSerialPortInfo> serialPortInfoList;

    QStringList candidates;

    const QDir sysfsTtyDir(QStringLiteral("/sys/class/tty"));
    const QStringList sysfsEntries = sysfsTtyDir.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
    for (const QString& name : sysfsEntries) {
        // Only real hardware UARTs have a "device" symlink
        if (QFileInfo::exists(sysfsTtyDir.filePath(name + QStringLiteral("/device")))) {
            candidates.append(name);
        }
    }

    if (candidates.isEmpty()) {
        // sysfs may be restricted by SELinux; fall back to scanning /dev for common UART names
        static const QStringList nameFilters = {
            QStringLiteral("ttyS*"),   QStringLiteral("ttyHS*"),  QStringLiteral("ttyMSM*"), QStringLiteral("ttyHSL*"),
            QStringLiteral("ttymxc*"), QStringLiteral("ttyAMA*"), QStringLiteral("ttySAC*"),
        };
        candidates = QDir(QStringLiteral("/dev")).entryList(nameFilters, QDir::System | QDir::Files);
    }

    for (const QString& name : std::as_const(candidates)) {
        const QString devPath = QLatin1String("/dev/") + name;
        if (::access(devPath.toLocal8Bit().constData(), R_OK | W_OK) != 0) {
            continue;
        }

        qCDebug(QSerialPortInfo_AndroidLog) << "Found accessible internal serial port" << devPath;

        QSerialPortInfoPrivate priv;
        priv.portName = name;
        priv.device = devPath;
        priv.description = QStringLiteral("Internal UART");
        serialPortInfoList.append(priv);
    }

    ok = true;
    return serialPortInfoList;
}

QList<QSerialPortInfo> availablePortsByUdev(bool& ok)
{
    ok = false;
    return QList<QSerialPortInfo>();
}

QList<QSerialPortInfo> QSerialPortInfo::availablePorts()
{
    bool ok = false;
    QList<QSerialPortInfo> serialPortInfoList = availablePortsByFiltersOfDevices(ok);
    if (!ok) {
        serialPortInfoList.clear();
    }

    serialPortInfoList.append(availablePortsBySysfs(ok));

    return serialPortInfoList;
}

QString QSerialPortInfoPrivate::portNameToSystemLocation(const QString& source)
{
    return (source.startsWith(QLatin1Char('/')) || source.startsWith(QLatin1String("./")) ||
            source.startsWith(QLatin1String("../")))
               ? source
               : (QLatin1String("/dev/") + source);
}

QString QSerialPortInfoPrivate::portNameFromSystemLocation(const QString& source)
{
    return source.startsWith(QLatin1String("/dev/")) ? source.mid(5) : source;
}

QT_END_NAMESPACE
