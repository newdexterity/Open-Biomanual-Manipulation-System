#include "deviceinfo.h"
#include "qbluetoothuuid.h"


DeviceInfo::DeviceInfo(const QBluetoothDeviceInfo &d)
{
    device = d;
}

QString DeviceInfo::getAddress() const
{
#ifdef Q_OS_MAC
    return device.deviceUuid().toString();
#else
    return device.address().toString();
#endif

}

QString DeviceInfo::getName() const
{
    return device.name();
}

QBluetoothDeviceInfo DeviceInfo::getDevice()
{
    return device;
}

void DeviceInfo::setDevice(const QBluetoothDeviceInfo &dev)
{
    qDebug() << "Setting device!";
    device = QBluetoothDeviceInfo(dev);
    Q_EMIT deviceChanged();
}
