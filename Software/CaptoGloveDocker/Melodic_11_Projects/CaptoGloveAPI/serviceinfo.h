#ifndef SERVICEINFO_H
#define SERVICEINFO_H
#include <QtBluetooth/QLowEnergyService>

class ServiceInfo: public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString serviceName READ getName NOTIFY serviceChanged)
    Q_PROPERTY(QString serviceUuid READ getUuid NOTIFY serviceChanged)
    Q_PROPERTY(QString serviceType READ getType NOTIFY serviceChanged)
public:
    ServiceInfo() = default;
    ServiceInfo(QLowEnergyService *service);
    QLowEnergyService *service() const;
    QString getUuid() const;
    QString getName() const;
    QString getType() const;

Q_SIGNALS:
    void serviceChanged();

private:
    QLowEnergyService *m_service = nullptr;
};

#endif // SERVICEINFO_H
