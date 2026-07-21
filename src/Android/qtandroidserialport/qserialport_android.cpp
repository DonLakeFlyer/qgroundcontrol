#include <QtCore/QDeadlineTimer>
#include <QtCore/QMetaObject>
#include <QtCore/QPointer>
#include <QtCore/QScopeGuard>
#include <cerrno>
#include <climits>
#include <cstring>
#include <fcntl.h>
#include <iterator>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include "QGCLoggingCategory.h"
#include "qserialport_p.h"

QGC_LOGGING_CATEGORY(AndroidSerialPortLog, "Android.AndroidSerialPort")

QT_BEGIN_NAMESPACE

bool QSerialPortPrivate::open(QIODevice::OpenMode mode)
{
    qCDebug(AndroidSerialPortLog) << "Opening" << systemLocation;

    // USB devices enumerated through the Java layer have /dev/bus/usb/... locations.
    // Anything else (e.g. /dev/ttyS1 internal UART) is opened directly as a device node.
    _isDeviceNode = !systemLocation.startsWith(QLatin1String("/dev/bus/usb/"));
    if (_isDeviceNode) {
        return _posixOpen(mode);
    }

    AndroidSerial::registerPointer(this);
    auto tokenGuard = qScopeGuard([this]() { AndroidSerial::unregisterPointer(this); });

    _deviceId = AndroidSerial::open(systemLocation, this);
    if (_deviceId == INVALID_DEVICE_ID) {
        qCWarning(AndroidSerialPortLog) << "Error opening" << systemLocation;
        setError(QSerialPortErrorInfo(QSerialPort::DeviceNotFoundError));
        return false;
    }

    descriptor = AndroidSerial::getDeviceHandle(_deviceId);
    if (descriptor == -1) {
        qCWarning(AndroidSerialPortLog) << "Failed to get device handle for" << systemLocation;
        setError(QSerialPortErrorInfo(QSerialPort::OpenError));
        close();
        return false;
    }

    if (!_setParameters(inputBaudRate, dataBits, stopBits, parity)) {
        qCWarning(AndroidSerialPortLog) << "Failed to set serial port parameters for" << systemLocation;
        close();
        return false;
    }

    if (!setFlowControl(flowControl)) {
        qCWarning(AndroidSerialPortLog) << "Failed to set serial port flow control for" << systemLocation;
        close();
        return false;
    }

    if (mode & QIODevice::ReadOnly) {
        if (!startAsyncRead()) {
            qCWarning(AndroidSerialPortLog) << "Failed to start async read for" << systemLocation;
            close();
            return false;
        }
    } else if (mode & QIODevice::WriteOnly) {
        if (!_stopAsyncRead()) {
            qCWarning(AndroidSerialPortLog) << "Failed to stop async read for" << systemLocation;
        }
    }

    (void) clear(QSerialPort::AllDirections);
    tokenGuard.dismiss();

    return true;
}

void QSerialPortPrivate::close()
{
    qCDebug(AndroidSerialPortLog) << "Closing" << systemLocation;

    _stopAsyncRead();

    if (_isDeviceNode) {
        if (descriptor != -1) {
            (void) ::close(descriptor);
            descriptor = -1;
        }
        _isDeviceNode = false;
        return;
    }

    if (_deviceId != INVALID_DEVICE_ID) {
        if (!AndroidSerial::close(_deviceId)) {
            qCWarning(AndroidSerialPortLog) << "Failed to close device with ID" << _deviceId;
            setError(QSerialPortErrorInfo(QSerialPort::UnknownError, QSerialPort::tr("Closing device failed")));
        }
        _deviceId = INVALID_DEVICE_ID;
    }

    descriptor = -1;

    AndroidSerial::unregisterPointer(this);
}

void QSerialPortPrivate::exceptionArrived(const QString& ex)
{
    qCWarning(AndroidSerialPortLog) << "Exception arrived on device ID" << _deviceId << ":" << ex;
    setError(QSerialPortErrorInfo(QSerialPort::UnknownError, ex));
}

bool QSerialPortPrivate::startAsyncRead()
{
    if (_isDeviceNode) {
        if (!_posixStartReadThread()) {
            setError(QSerialPortErrorInfo(QSerialPort::UnknownError, QSerialPort::tr("Failed to start async read")));
            return false;
        }
        _scheduleReadyRead();
        return true;
    }

    if (!AndroidSerial::readThreadRunning(_deviceId)) {
        const bool result = AndroidSerial::startReadThread(_deviceId);
        if (!result) {
            qCWarning(AndroidSerialPortLog) << "Failed to start async read thread for device ID" << _deviceId;
            setError(QSerialPortErrorInfo(QSerialPort::UnknownError, QSerialPort::tr("Failed to start async read")));
            return false;
        }
    }

    // If pending bytes were left behind due to read buffer backpressure,
    // schedule another drain as soon as reads are active again.
    _scheduleReadyRead();

    return true;
}

bool QSerialPortPrivate::_stopAsyncRead()
{
    bool result = true;

    if (_isDeviceNode) {
        _posixStopReadThread();
        return true;
    }

    if (AndroidSerial::readThreadRunning(_deviceId)) {
        result = AndroidSerial::stopReadThread(_deviceId);
        if (!result) {
            qCWarning(AndroidSerialPortLog) << "Failed to stop async read thread for device ID" << _deviceId;
            setError(QSerialPortErrorInfo(QSerialPort::UnknownError, QSerialPort::tr("Failed to stop async read")));
        }
    }

    return result;
}

qint64 QSerialPortPrivate::_drainPendingDataLocked(qint64 maxBytes)
{
    const qsizetype pendingSize = _pendingSizeLocked();
    if (pendingSize <= 0) {
        _pendingData.clear();
        _pendingDataOffset = 0;
        return 0;
    }

    qint64 toDrain = pendingSize;
    if (maxBytes >= 0) {
        toDrain = qMin(toDrain, maxBytes);
    }

    if (toDrain <= 0) {
        return 0;
    }

    buffer.append(_pendingData.constData() + _pendingDataOffset, toDrain);
    _pendingDataOffset += static_cast<qsizetype>(toDrain);

    if (_pendingDataOffset >= _pendingData.size()) {
        _pendingData.clear();
        _pendingDataOffset = 0;
    } else {
        // Compact occasionally to keep append operations efficient without
        // paying the cost on every drain.
        constexpr qsizetype kCompactThreshold = 4096;
        if (_pendingDataOffset >= kCompactThreshold && (_pendingDataOffset * 2) >= _pendingData.size()) {
            _compactPendingDataLocked();
        }
    }

    _bufferBytesEstimate.store(buffer.size(), std::memory_order_relaxed);
    return toDrain;
}

qsizetype QSerialPortPrivate::_pendingSizeLocked() const
{
    return qMax<qsizetype>(0, _pendingData.size() - _pendingDataOffset);
}

void QSerialPortPrivate::_compactPendingDataLocked()
{
    if (_pendingDataOffset <= 0) {
        return;
    }

    if (_pendingDataOffset >= _pendingData.size()) {
        _pendingData.clear();
        _pendingDataOffset = 0;
        return;
    }

    _pendingData.remove(0, _pendingDataOffset);
    _pendingDataOffset = 0;
}

void QSerialPortPrivate::newDataArrived(const char* bytes, int length)
{
    // qCDebug(AndroidSerialPortLog) << "newDataArrived" << length;

    qint64 droppedBytes = 0;

    QMutexLocker locker(&_readMutex);
    int bytesToRead = length;
    if (readBufferMaxSize) {
        const qint64 totalBuffered = _pendingSizeLocked() + _bufferBytesEstimate.load(std::memory_order_relaxed);
        const qint64 headroom = readBufferMaxSize - totalBuffered;
        if (bytesToRead > headroom) {
            bytesToRead = static_cast<int>(qMax(qint64(0), headroom));
            droppedBytes = static_cast<qint64>(length - bytesToRead);
        }
    }

    if (bytesToRead > 0) {
        constexpr qsizetype kCompactBeforeAppendThreshold = 8192;
        if (_pendingDataOffset >= kCompactBeforeAppendThreshold) {
            _compactPendingDataLocked();
        }
        _pendingData.append(bytes, bytesToRead);
        _readWaitCondition.wakeAll();
    }
    locker.unlock();

    if (droppedBytes > 0) {
        qCWarning(AndroidSerialPortLog) << "Read buffer full, dropping" << droppedBytes << "bytes";
    }

    if (bytesToRead <= 0) {
        return;
    }

    _scheduleReadyRead();
}

void QSerialPortPrivate::_scheduleReadyRead()
{
    Q_Q(QSerialPort);

    if (!_readyReadPending.exchange(true)) {
        QPointer<QSerialPort> guard(q);
        QMetaObject::invokeMethod(
            q,
            [this, guard]() {
                if (!guard) {
                    return;
                }

                QMutexLocker locker(&_readMutex);
                if (_pendingSizeLocked() <= 0) {
                    _readyReadPending.store(false);
                    return;
                }

                if (readBufferMaxSize > 0) {
                    const qint64 canAccept = readBufferMaxSize - buffer.size();
                    if (canAccept > 0) {
                        (void) _drainPendingDataLocked(canAccept);
                    }
                } else {
                    (void) _drainPendingDataLocked();
                }

                // Reset flag after drain so data arriving during the drain
                // does not enqueue redundant lambdas. If pending data remains,
                // reschedule so nothing is left undelivered.
                const bool more = (_pendingSizeLocked() > 0);
                _readyReadPending.store(false);

                _readWaitCondition.wakeAll();
                locker.unlock();

                emit guard->readyRead();

                if (more) {
                    _scheduleReadyRead();
                }
            },
            Qt::QueuedConnection);
    }
}

bool QSerialPortPrivate::waitForReadyRead(int msecs)
{
    QMutexLocker locker(&_readMutex);
    if (!buffer.isEmpty()) {
        return true;
    }

    if (_pendingSizeLocked() > 0) {
        (void) _drainPendingDataLocked();
        return true;
    }

    QDeadlineTimer deadline(msecs);
    while (buffer.isEmpty() && (_pendingSizeLocked() <= 0)) {
        if (!_readWaitCondition.wait(&_readMutex, deadline)) {
            break;
        }

        if (!buffer.isEmpty()) {
            return true;
        }

        if (_pendingSizeLocked() > 0) {
            (void) _drainPendingDataLocked();
            return true;
        }
    }
    locker.unlock();

    qCWarning(AndroidSerialPortLog) << "Timeout while waiting for ready read on device ID" << _deviceId;
    setError(QSerialPortErrorInfo(QSerialPort::TimeoutError, QSerialPort::tr("Timeout while waiting for ready read")));

    return false;
}

bool QSerialPortPrivate::waitForBytesWritten(int msecs)
{
    const bool result = _writeDataOneShot(msecs);
    if (!result) {
        qCWarning(AndroidSerialPortLog) << "Timeout while waiting for bytes written on device ID" << _deviceId;
        setError(QSerialPortErrorInfo(QSerialPort::TimeoutError,
                                      QSerialPort::tr("Timeout while waiting for bytes written")));
    }

    return result;
}

bool QSerialPortPrivate::_writeDataOneShot(int msecs)
{
    if (writeBuffer.isEmpty()) {
        return true;
    }

    qint64 pendingBytesWritten = 0;

    while (!writeBuffer.isEmpty()) {
        const char* dataPtr = writeBuffer.readPointer();
        const qint64 dataSize = writeBuffer.nextDataBlockSize();

        const qint64 written = _writeToPort(dataPtr, dataSize, msecs);
        if (written < 0) {
            qCWarning(AndroidSerialPortLog) << "Failed to write data one shot on device ID" << _deviceId;
            setError(QSerialPortErrorInfo(QSerialPort::WriteError, QSerialPort::tr("Failed to write data one shot")));
            return false;
        }

        writeBuffer.free(written);
        pendingBytesWritten += written;
    }

    const bool result = (pendingBytesWritten > 0);
    if (result) {
        Q_Q(QSerialPort);
        emit q->bytesWritten(pendingBytesWritten);
    }

    return result;
}

qint64 QSerialPortPrivate::_writeToPort(const char* data, qint64 maxSize, int timeout, bool async)
{
    qint64 result = 0;
    if (_isDeviceNode) {
        Q_UNUSED(async);
        result = _posixWrite(data, maxSize, timeout);
    } else {
        result = AndroidSerial::write(_deviceId, data, maxSize, timeout, async);
    }

    if (result < 0) {
        qCWarning(AndroidSerialPortLog) << "Failed to write to port ID" << _deviceId;
        setError(QSerialPortErrorInfo(QSerialPort::WriteError, QSerialPort::tr("Failed to write to port")));
    }

    return result;
}

qint64 QSerialPortPrivate::writeData(const char* data, qint64 maxSize)
{
    if (!data || (maxSize <= 0)) {
        qCWarning(AndroidSerialPortLog) << "Invalid data or size in writeData for device ID" << _deviceId;
        setError(QSerialPortErrorInfo(QSerialPort::WriteError, QSerialPort::tr("Invalid data or size")));
        return -1;
    }

    return _writeToPort(data, maxSize);
}

bool QSerialPortPrivate::flush()
{
    const bool result = _writeDataOneShot();
    if (!result) {
        qCWarning(AndroidSerialPortLog) << "Flush operation failed for device ID" << _deviceId;
        setError(QSerialPortErrorInfo(QSerialPort::UnknownError, QSerialPort::tr("Failed to flush")));
    }

    return result;
}

bool QSerialPortPrivate::clear(QSerialPort::Directions directions)
{
    const bool input = directions & QSerialPort::Input;
    const bool output = directions & QSerialPort::Output;

    if (_isDeviceNode) {
        const int queue = (input && output) ? TCIOFLUSH : (input ? TCIFLUSH : TCOFLUSH);
        if (::tcflush(descriptor, queue) == -1) {
            qCWarning(AndroidSerialPortLog) << "tcflush failed for" << systemLocation << ":" << strerror(errno);
            setError(QSerialPortErrorInfo(QSerialPort::UnknownError, QSerialPort::tr("Failed to purge buffers")));
            return false;
        }
        return true;
    }

    const bool result = AndroidSerial::purgeBuffers(_deviceId, input, output);
    if (!result) {
        qCWarning(AndroidSerialPortLog) << "Failed to purge buffers for device ID" << _deviceId;
        setError(QSerialPortErrorInfo(QSerialPort::UnknownError, QSerialPort::tr("Failed to purge buffers")));
    }

    return result;
}

QSerialPort::PinoutSignals QSerialPortPrivate::pinoutSignals()
{
    if (_isDeviceNode) {
        int arg = 0;
        if (::ioctl(descriptor, TIOCMGET, &arg) == -1) {
            return QSerialPort::NoSignal;
        }

        QSerialPort::PinoutSignals ret = QSerialPort::NoSignal;
        if (arg & TIOCM_DTR)
            ret |= QSerialPort::DataTerminalReadySignal;
        if (arg & TIOCM_RTS)
            ret |= QSerialPort::RequestToSendSignal;
        if (arg & TIOCM_CTS)
            ret |= QSerialPort::ClearToSendSignal;
        if (arg & TIOCM_DSR)
            ret |= QSerialPort::DataSetReadySignal;
        if (arg & TIOCM_CAR)
            ret |= QSerialPort::DataCarrierDetectSignal;
        if (arg & TIOCM_RNG)
            ret |= QSerialPort::RingIndicatorSignal;
        return ret;
    }

    return AndroidSerial::getControlLines(_deviceId);
}

bool QSerialPortPrivate::setDataTerminalReady(bool set)
{
    if (_isDeviceNode) {
        int arg = TIOCM_DTR;
        if (::ioctl(descriptor, set ? TIOCMBIS : TIOCMBIC, &arg) == -1) {
            setError(QSerialPortErrorInfo(QSerialPort::UnknownError, QSerialPort::tr("Failed to set DTR")));
            return false;
        }
        return true;
    }

    const bool result = AndroidSerial::setDataTerminalReady(_deviceId, set);
    if (!result) {
        qCWarning(AndroidSerialPortLog) << "Failed to set DTR for device ID" << _deviceId;
        setError(QSerialPortErrorInfo(QSerialPort::UnknownError, QSerialPort::tr("Failed to set DTR")));
    }

    return result;
}

bool QSerialPortPrivate::setRequestToSend(bool set)
{
    if (_isDeviceNode) {
        int arg = TIOCM_RTS;
        if (::ioctl(descriptor, set ? TIOCMBIS : TIOCMBIC, &arg) == -1) {
            setError(QSerialPortErrorInfo(QSerialPort::UnknownError, QSerialPort::tr("Failed to set RTS")));
            return false;
        }
        return true;
    }

    const bool result = AndroidSerial::setRequestToSend(_deviceId, set);
    if (!result) {
        qCWarning(AndroidSerialPortLog) << "Failed to set RTS for device ID" << _deviceId;
        setError(QSerialPortErrorInfo(QSerialPort::UnknownError, QSerialPort::tr("Failed to set RTS")));
    }

    return result;
}

bool QSerialPortPrivate::_setParameters(qint32 baudRate, QSerialPort::DataBits dataBits_,
                                        QSerialPort::StopBits stopBits_, QSerialPort::Parity parity_)
{
    bool result = false;
    if (_isDeviceNode) {
        result = _posixSetParameters(baudRate, dataBits_, stopBits_, parity_);
    } else {
        result = AndroidSerial::setParameters(_deviceId, baudRate, _dataBitsToAndroidDataBits(dataBits_),
                                              _stopBitsToAndroidStopBits(stopBits_), _parityToAndroidParity(parity_));
    }

    if (!result) {
        qCWarning(AndroidSerialPortLog) << "Failed to set Parameters for device ID" << _deviceId;
        setError(QSerialPortErrorInfo(QSerialPort::UnknownError, QSerialPort::tr("Failed to set parameters")));
    }

    return result;
}

bool QSerialPortPrivate::setBaudRate()
{
    return setBaudRate(inputBaudRate, QSerialPort::AllDirections);
}

bool QSerialPortPrivate::setBaudRate(qint32 baudRate, QSerialPort::Directions directions)
{
    if (baudRate <= 0) {
        qCWarning(AndroidSerialPortLog) << "Invalid baud rate value:" << baudRate;
        setError(
            QSerialPortErrorInfo(QSerialPort::UnsupportedOperationError, QSerialPort::tr("Invalid baud rate value")));
        return false;
    }

    if (directions != QSerialPort::AllDirections) {
        qCWarning(AndroidSerialPortLog) << "Custom baud rate direction is unsupported:" << directions;
        setError(QSerialPortErrorInfo(QSerialPort::UnsupportedOperationError,
                                      QSerialPort::tr("Custom baud rate direction is unsupported")));
        return false;
    }

    const bool result = _setParameters(baudRate, dataBits, stopBits, parity);
    if (result) {
        inputBaudRate = outputBaudRate = baudRate;
    } else {
        qCWarning(AndroidSerialPortLog) << "Failed to set baud rate for device ID" << _deviceId;
        setError(QSerialPortErrorInfo(QSerialPort::UnknownError, QSerialPort::tr("Failed to set baud rate")));
    }

    return result;
}

int QSerialPortPrivate::_dataBitsToAndroidDataBits(QSerialPort::DataBits dataBits_)
{
    switch (dataBits_) {
        case QSerialPort::Data5:
            return AndroidSerial::Data5;
        case QSerialPort::Data6:
            return AndroidSerial::Data6;
        case QSerialPort::Data7:
            return AndroidSerial::Data7;
        case QSerialPort::Data8:
            return AndroidSerial::Data8;
        default:
            qCWarning(AndroidSerialPortLog) << "Invalid Data Bits" << dataBits_;
            return AndroidSerial::Data8;  // Default to Data8
    }
}

bool QSerialPortPrivate::setDataBits(QSerialPort::DataBits dataBits_)
{
    const bool result = _setParameters(inputBaudRate, dataBits_, stopBits, parity);
    if (!result) {
        qCWarning(AndroidSerialPortLog) << "Failed to set data bits for device ID" << _deviceId;
        setError(QSerialPortErrorInfo(QSerialPort::UnknownError, QSerialPort::tr("Failed to set data bits")));
    }

    return result;
}

int QSerialPortPrivate::_parityToAndroidParity(QSerialPort::Parity parity_)
{
    switch (parity_) {
        case QSerialPort::SpaceParity:
            return AndroidSerial::SpaceParity;
        case QSerialPort::MarkParity:
            return AndroidSerial::MarkParity;
        case QSerialPort::EvenParity:
            return AndroidSerial::EvenParity;
        case QSerialPort::OddParity:
            return AndroidSerial::OddParity;
        case QSerialPort::NoParity:
            return AndroidSerial::NoParity;
        default:
            qCWarning(AndroidSerialPortLog) << "Invalid parity type:" << parity_;
            return AndroidSerial::NoParity;  // Default to NoParity
    }
}

bool QSerialPortPrivate::setParity(QSerialPort::Parity parity_)
{
    const bool result = _setParameters(inputBaudRate, dataBits, stopBits, parity_);
    if (!result) {
        qCWarning(AndroidSerialPortLog) << "Failed to set parity for device ID" << _deviceId;
        setError(QSerialPortErrorInfo(QSerialPort::UnknownError, QSerialPort::tr("Failed to set parity")));
    }

    return result;
}

int QSerialPortPrivate::_stopBitsToAndroidStopBits(QSerialPort::StopBits stopBits_)
{
    switch (stopBits_) {
        case QSerialPort::TwoStop:
            return AndroidSerial::TwoStop;
        case QSerialPort::OneAndHalfStop:
            return AndroidSerial::OneAndHalfStop;
        case QSerialPort::OneStop:
            return AndroidSerial::OneStop;
        default:
            qCWarning(AndroidSerialPortLog) << "Invalid Stop Bits type:" << stopBits_;
            return AndroidSerial::OneStop;  // Default to OneStop
    }
}

bool QSerialPortPrivate::setStopBits(QSerialPort::StopBits stopBits_)
{
    const bool result = _setParameters(inputBaudRate, dataBits, stopBits_, parity);
    if (!result) {
        qCWarning(AndroidSerialPortLog) << "Failed to set StopBits for device ID" << _deviceId;
        setError(QSerialPortErrorInfo(QSerialPort::UnknownError, QSerialPort::tr("Failed to set StopBits")));
    }

    return result;
}

int QSerialPortPrivate::_flowControlToAndroidFlowControl(QSerialPort::FlowControl flowControl_)
{
    switch (flowControl_) {
        case QSerialPort::HardwareControl:
            return AndroidSerial::RtsCtsFlowControl;
        case QSerialPort::SoftwareControl:
            return AndroidSerial::XonXoffFlowControl;
        case QSerialPort::NoFlowControl:
            return AndroidSerial::NoFlowControl;
        default:
            qCWarning(AndroidSerialPortLog) << "Invalid Flow Control type:" << flowControl_;
            return AndroidSerial::NoFlowControl;  // Default to NoFlowControl
    }
}

bool QSerialPortPrivate::setFlowControl(QSerialPort::FlowControl flowControl_)
{
    bool result = false;
    if (_isDeviceNode) {
        result = _posixSetFlowControl(flowControl_);
    } else {
        result = AndroidSerial::setFlowControl(_deviceId, _flowControlToAndroidFlowControl(flowControl_));
    }

    if (!result) {
        qCWarning(AndroidSerialPortLog) << "Failed to set Flow Control for device ID" << _deviceId;
        setError(QSerialPortErrorInfo(QSerialPort::UnknownError, QSerialPort::tr("Failed to set Flow Control")));
    }

    return result;
}

bool QSerialPortPrivate::setBreakEnabled(bool set)
{
    bool result = false;
    if (_isDeviceNode) {
        result = (::ioctl(descriptor, set ? TIOCSBRK : TIOCCBRK) != -1);
    } else {
        result = AndroidSerial::setBreak(_deviceId, set);
    }

    if (!result) {
        setError(QSerialPortErrorInfo(QSerialPort::UnknownError, QSerialPort::tr("Failed to set Break Enabled")));
    }

    return result;
}

static constexpr qint32 kStandardBaudRates[] = {
    50,     75,     110,     134,     150,     200,     300,     600,     1200,    1800,
    2400,   4800,   9600,    19200,   38400,   57600,   115200,  230400,  460800,  500000,
    576000, 921600, 1000000, 1152000, 1500000, 2000000, 2500000, 3000000, 3500000, 4000000,
};

QList<qint32> QSerialPortPrivate::standardBaudRates()
{
    return QList<qint32>(std::begin(kStandardBaudRates), std::end(kStandardBaudRates));
}

// ----------------------------------------------------------------------------
// POSIX backend for internal (non-USB) device nodes such as /dev/ttyS1
// ----------------------------------------------------------------------------

static speed_t _baudRateToPosixSpeed(qint32 baudRate)
{
    switch (baudRate) {
        case 50:
            return B50;
        case 75:
            return B75;
        case 110:
            return B110;
        case 134:
            return B134;
        case 150:
            return B150;
        case 200:
            return B200;
        case 300:
            return B300;
        case 600:
            return B600;
        case 1200:
            return B1200;
        case 1800:
            return B1800;
        case 2400:
            return B2400;
        case 4800:
            return B4800;
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        case 460800:
            return B460800;
        case 500000:
            return B500000;
        case 576000:
            return B576000;
        case 921600:
            return B921600;
        case 1000000:
            return B1000000;
        case 1152000:
            return B1152000;
        case 1500000:
            return B1500000;
        case 2000000:
            return B2000000;
        case 2500000:
            return B2500000;
        case 3000000:
            return B3000000;
        case 3500000:
            return B3500000;
        case 4000000:
            return B4000000;
        default:
            return B0;
    }
}

bool QSerialPortPrivate::_posixOpen(QIODevice::OpenMode mode)
{
    descriptor = ::open(systemLocation.toLocal8Bit().constData(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (descriptor == -1) {
        qCWarning(AndroidSerialPortLog) << "Error opening" << systemLocation << ":" << strerror(errno);
        setError(
            QSerialPortErrorInfo((errno == EACCES) ? QSerialPort::PermissionError : QSerialPort::DeviceNotFoundError));
        return false;
    }

    struct termios tio;
    if (::tcgetattr(descriptor, &tio) == -1) {
        qCWarning(AndroidSerialPortLog) << "tcgetattr failed for" << systemLocation << ":" << strerror(errno);
        setError(QSerialPortErrorInfo(QSerialPort::OpenError));
        close();
        return false;
    }

    ::cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    if (::tcsetattr(descriptor, TCSANOW, &tio) == -1) {
        qCWarning(AndroidSerialPortLog) << "tcsetattr failed for" << systemLocation << ":" << strerror(errno);
        setError(QSerialPortErrorInfo(QSerialPort::OpenError));
        close();
        return false;
    }

    if (!_setParameters(inputBaudRate, dataBits, stopBits, parity)) {
        qCWarning(AndroidSerialPortLog) << "Failed to set serial port parameters for" << systemLocation;
        close();
        return false;
    }

    if (!setFlowControl(flowControl)) {
        qCWarning(AndroidSerialPortLog) << "Failed to set serial port flow control for" << systemLocation;
        close();
        return false;
    }

    if (mode & QIODevice::ReadOnly) {
        if (!startAsyncRead()) {
            qCWarning(AndroidSerialPortLog) << "Failed to start async read for" << systemLocation;
            close();
            return false;
        }
    }

    (void) clear(QSerialPort::AllDirections);

    return true;
}

bool QSerialPortPrivate::_posixStartReadThread()
{
    if (_posixReadThread.joinable()) {
        return true;
    }

    if (::pipe(_posixShutdownPipe) == -1) {
        qCWarning(AndroidSerialPortLog) << "Failed to create shutdown pipe:" << strerror(errno);
        return false;
    }

    _posixReadThread = std::thread([this]() { _posixReadLoop(); });

    return true;
}

void QSerialPortPrivate::_posixStopReadThread()
{
    if (!_posixReadThread.joinable()) {
        return;
    }

    const char stop = 1;
    (void) ::write(_posixShutdownPipe[1], &stop, 1);
    _posixReadThread.join();

    for (int& fd : _posixShutdownPipe) {
        if (fd != -1) {
            (void) ::close(fd);
            fd = -1;
        }
    }
}

void QSerialPortPrivate::_posixReadLoop()
{
    char readBuf[4096];

    for (;;) {
        struct pollfd fds[2] = {
            {descriptor, POLLIN, 0},
            {_posixShutdownPipe[0], POLLIN, 0},
        };

        const int ret = ::poll(fds, 2, -1);
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            exceptionArrived(QStringLiteral("poll failed: %1").arg(QLatin1String(strerror(errno))));
            break;
        }

        if (fds[1].revents != 0) {
            break;  // shutdown requested
        }

        if (fds[0].revents & (POLLERR | POLLHUP | POLLNVAL)) {
            exceptionArrived(QStringLiteral("Serial port error/hangup"));
            break;
        }

        if (fds[0].revents & POLLIN) {
            const ssize_t bytesRead = ::read(descriptor, readBuf, sizeof(readBuf));
            if (bytesRead > 0) {
                newDataArrived(readBuf, static_cast<int>(bytesRead));
            } else if ((bytesRead < 0) && (errno != EAGAIN) && (errno != EINTR)) {
                exceptionArrived(QStringLiteral("read failed: %1").arg(QLatin1String(strerror(errno))));
                break;
            }
        }
    }
}

qint64 QSerialPortPrivate::_posixWrite(const char* data, qint64 maxSize, int timeout)
{
    qint64 totalWritten = 0;
    QDeadlineTimer deadline(timeout);

    while (totalWritten < maxSize) {
        struct pollfd pfd = {descriptor, POLLOUT, 0};
        const int remaining = static_cast<int>(qMin<qint64>(deadline.remainingTime(), INT_MAX));
        const int ret = ::poll(&pfd, 1, remaining);
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            return -1;
        }
        if (ret == 0) {
            break;  // timeout, return partial write
        }

        const ssize_t written = ::write(descriptor, data + totalWritten, maxSize - totalWritten);
        if (written < 0) {
            if ((errno == EAGAIN) || (errno == EINTR)) {
                continue;
            }
            return -1;
        }

        totalWritten += written;
    }

    return totalWritten;
}

bool QSerialPortPrivate::_posixSetParameters(qint32 baudRate, QSerialPort::DataBits dataBits_,
                                             QSerialPort::StopBits stopBits_, QSerialPort::Parity parity_)
{
    const speed_t speed = _baudRateToPosixSpeed(baudRate);
    if (speed == B0) {
        qCWarning(AndroidSerialPortLog) << "Unsupported baud rate:" << baudRate;
        return false;
    }

    struct termios tio;
    if (::tcgetattr(descriptor, &tio) == -1) {
        return false;
    }

    (void) ::cfsetispeed(&tio, speed);
    (void) ::cfsetospeed(&tio, speed);

    tio.c_cflag &= ~CSIZE;
    switch (dataBits_) {
        case QSerialPort::Data5:
            tio.c_cflag |= CS5;
            break;
        case QSerialPort::Data6:
            tio.c_cflag |= CS6;
            break;
        case QSerialPort::Data7:
            tio.c_cflag |= CS7;
            break;
        case QSerialPort::Data8:
        default:
            tio.c_cflag |= CS8;
            break;
    }

    if (stopBits_ == QSerialPort::TwoStop) {
        tio.c_cflag |= CSTOPB;
    } else {
        tio.c_cflag &= ~CSTOPB;
    }

    tio.c_cflag &= ~(PARENB | PARODD);
    tio.c_iflag &= ~(INPCK | ISTRIP);
    switch (parity_) {
        case QSerialPort::EvenParity:
            tio.c_cflag |= PARENB;
            break;
        case QSerialPort::OddParity:
            tio.c_cflag |= (PARENB | PARODD);
            break;
        case QSerialPort::NoParity:
        default:
            break;
    }

    return (::tcsetattr(descriptor, TCSANOW, &tio) != -1);
}

bool QSerialPortPrivate::_posixSetFlowControl(QSerialPort::FlowControl flowControl_)
{
    struct termios tio;
    if (::tcgetattr(descriptor, &tio) == -1) {
        return false;
    }

    tio.c_cflag &= ~CRTSCTS;
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    switch (flowControl_) {
        case QSerialPort::HardwareControl:
            tio.c_cflag |= CRTSCTS;
            break;
        case QSerialPort::SoftwareControl:
            tio.c_iflag |= (IXON | IXOFF);
            break;
        case QSerialPort::NoFlowControl:
        default:
            break;
    }

    return (::tcsetattr(descriptor, TCSANOW, &tio) != -1);
}

QSerialPort::Handle QSerialPort::handle() const
{
    Q_D(const QSerialPort);
    return d->descriptor;
}

QT_END_NAMESPACE
