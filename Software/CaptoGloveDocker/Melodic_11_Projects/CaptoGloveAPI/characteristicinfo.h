#ifndef CHARACTERISTICINFO_H
#define CHARACTERISTICINFO_H
#include <QObject>
#include <QString>
#include <QtBluetooth/QLowEnergyCharacteristic>

class CharacteristicInfo: public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString characteristicName READ getName NOTIFY characteristicChanged)
    Q_PROPERTY(QString characteristicUuid READ getUuid NOTIFY characteristicChanged)
    Q_PROPERTY(QString characteristicValue READ getValue NOTIFY characteristicChanged)
    Q_PROPERTY(QString characteristicHandle READ getHandle NOTIFY characteristicChanged)
    Q_PROPERTY(QString characteristicPermission READ getPermission NOTIFY characteristicChanged)

public:
    CharacteristicInfo() = default;
    CharacteristicInfo(const QLowEnergyCharacteristic &characteristic);
    void setCharacteristic(const QLowEnergyCharacteristic &characteristic);
    QString getName() const;
    QString getUuid() const;
    QString getValue() const;
    QString getHandle() const;
    QString getPermission() const;
    QLowEnergyCharacteristic getCharacteristic() const;

Q_SIGNALS:
    void characteristicChanged();

private:
    QLowEnergyCharacteristic m_characteristic;
};

#endif // CHARACTERISTICINFO_H
