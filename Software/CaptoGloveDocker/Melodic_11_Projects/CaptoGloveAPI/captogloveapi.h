#ifndef CAPTOGLOVEAPI_H
#define CAPTOGLOVEAPI_H

// Bluetooth include
#include "qbluetooth.h"
#include "qbluetoothdevicediscoveryagent.h"
#include "qbluetoothlocaldevice.h"

// Q include
#include "qlowenergycontroller.h"
#include "qfile.h"
#include "qsettings.h"

#include "deviceinfo.h"
#include "serviceinfo.h"
#include "characteristicinfo.h"

// Specific datatypes include
#include <QDebug>
#include <QMetaEnum>
#include <QTimer>
#include <QtEndian>
#include <QThread>

// Include protobuffer msg?
#include <proto_impl/captoglove_v1.pb.h>

// Include Logger
#include "logger.h"

class CaptoGloveAPI : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QVariant devicesList READ getDevices NOTIFY devicesUpdated)
    Q_PROPERTY(QVariant servicesList READ getServices NOTIFY servicesUpdated)
    Q_PROPERTY(QVariant charactersiticsList READ getCharacteristics NOTIFY characteristicsUpdated)
    Q_PROPERTY(bool controllerError READ hasControllerError)
    Q_PROPERTY(bool alive READ alive NOTIFY aliveChanged)
    //Q_PROPERTY(DeviceState deviceState READ deviceState NOTIFY deviceStateChanged)

    typedef enum{
        DeviceFound=0,
        DeviceConnected,
        DevicePaired,
        DeviceError,
        DeviceNotFound,
    }DeviceState;


public:
    CaptoGloveAPI(QObject *parent, QString configPath);
    ~CaptoGloveAPI();

    QVariant getDevices();                                                                  // xx
    QVariant getServices();                                                                 // xx
    QVariant getCharacteristics();                                                          // xx

    // Service Getters
    int getBatteryLevel();                                                                  // xx
    QString getDeviceName();                                                                // xx
    DeviceState getDeviceState();
    QByteArray getCurrentFingerPosition();                                                  // xx

    bool alive() const;
    bool hasControllerError() const;                                                        // xx
    bool isRandomAddress() const;



    void initializeController (const QBluetoothDeviceInfo &info);                           // xx

                                                                                            // init, TEST method
    void run();
    void discoverServices();
    bool getFingers();
    bool getInit();

    bool m_initialized;



public slots:
    void startDeviceDiscovery();                                                            // xx
    void scanServices(DeviceInfo &device);                          // xx

    void startConnection();

    void connectToService(const QString &uuid);
    void disconnectFromDevice();

    void setDeviceStateSlot(CaptoGloveAPI::DeviceState state);

private slots:
    // QBluetoothDeviceDiscoveryAgent
    void addDevice(const QBluetoothDeviceInfo &device);                                     // xx
    void deviceScanError(QBluetoothDeviceDiscoveryAgent::Error error);                      // xx

    // QLowEnergyController related
    void addLowEnergyService (const QBluetoothUuid &uuid);                                  // xx
    void deviceConnected();                                                                 // xx
    void deviceDisconnected();                                                              // xx
    void errorReceived();                                                                   // xx
    void serviceScanDone();                                                                 // xx

    // QLowEnergyService related
    void serviceDetailsDiscovered(QLowEnergyService::ServiceState newState);
    void processLoop();

    captoglove_v1::FingerFeedbackMsg setFingerMsg(QVector<int> fingerVector);
    captoglove_v1::BatteryLevelMsg setBatteryMsg(int batteryVal);




Q_SIGNALS:
    void devicesUpdated();
    void servicesUpdated();
    void characteristicsUpdated();
    void updateChanged();
    void stateChanged();
    void disconnected();
    void aliveChanged();
    void servicesDiscovered();
    void initialized();
    void testSignal();
    void updateFingerState(captoglove_v1::FingerFeedbackMsg);
    void updateBatteryState(captoglove_v1::BatteryLevelMsg);
    void setDeviceStateSignal(CaptoGloveAPI::DeviceState state);

private:
    void saveSettings(QSettings* Settings);
    void loadSettings(QSettings* Settings);

    // QLowEnergyController
    void serviceDiscovered(const QBluetoothUuid &gatt);
    void checkServiceStatus(const QBluetoothUuid &uuid);

    void serviceStateChanged(QLowEnergyService::ServiceState s);

    // Generic Access
    void GAServiceStateChanged(QLowEnergyService::ServiceState s);

    // Battery service
    void batteryServiceStateChanged (QLowEnergyService::ServiceState s);
    void updateBatteryLevelValue(const QLowEnergyCharacteristic &c,
                                 const QByteArray &value);
    void confirmedBatteryDescWrite(const QLowEnergyDescriptor &d,
                                  const QByteArray &value);

    // Scan Parameters service
    void scanParamsServiceStateChanged(QLowEnergyService::ServiceState s);

    void HIDserviceStateChanged(QLowEnergyService::ServiceState s);

    void genericAccessServiceStateChanged(QLowEnergyService::ServiceState s);

    void fingerPoseServiceStateChanged(QLowEnergyService::ServiceState s);

    void fingerPoseCharacteristicChanged(const QLowEnergyCharacteristic &c,
                                         const QByteArray &value);
    void confirmedDescriptorWrite(const QLowEnergyDescriptor &d,
                                  const QByteArray &value);

    // General
    void readInitialValue(QLowEnergyService &service);

    void refreshStates();

    void setUpdate(const QString &message);
    int readBatteryLevel();
    void getScanParams();

    quint8 convertToPercentage(quint8 value);

    QString m_configPath;
    QSettings* m_config;

    QBluetoothDeviceDiscoveryAgent *m_discoveryAgent;
    QBluetoothLocalDevice *localDevice;

    QLowEnergyController* m_controller = nullptr;
    QList<DeviceInfo *> m_devices;
    QList<QBluetoothDeviceInfo> m_devicesBTInfo;
    QList<ServiceInfo *> m_services;
    QMap<QBluetoothUuid, CharacteristicInfo*> m_characteristics;


    // Stuff related to device
    DeviceState m_deviceState;
    QString m_deviceName="";
    QString m_deviceAddress="";
    int m_scanTimeoutMs;
    bool m_rightHand;
    int m_thumbIndex; int m_indexIndex; int m_middleIndex; int m_ringIndex; int m_pinkyIndex;

    // Service flags
    bool m_randomAdress;
    bool m_foundGAService=false;
    bool m_foundBatteryLevelService=false;
    bool m_foundHIDService=false;
    bool m_foundHIDControlPointService=false;
    bool m_foundScanParametersService=false;
    bool m_foundDeviceInfoService=false;
    bool m_foundFingerPositionService=false;

    // Control params
    bool m_reconnect = true;

    // Services -> it's better to have only one list which contains those services
    QLowEnergyService *m_batteryLevelService=nullptr;
    QLowEnergyService *m_GAService = nullptr;
    QLowEnergyService *m_HIDService = nullptr;
    QLowEnergyService *m_ScanParametersService = nullptr;
    QLowEnergyService *m_DeviceInfoService = nullptr;
    QLowEnergyService *m_FingerPositionsService = nullptr;

    // Global characteristics
    QLowEnergyCharacteristic m_fingerPositionsChar;

    // Characteristics
    QLowEnergyDescriptor m_batteryNotificationDesc;
    bool m_connected;
    bool m_discoveredServices;
    bool m_deviceScanState;
    QString m_message;
    DeviceInfo m_peripheralDevice;

    // Values of interest for getter
    int m_batteryLevelValue;
    QByteArray m_currentFingerPosition;


    captoglove_v1::BatteryLevelMsg m_batteryMsg;
    captoglove_v1::DeviceInformationMsg m_deviceInformationMsg;
    captoglove_v1::FingerFeedbackMsg m_fingerFeedbackMsg;

};

#endif // CAPTOGLOVEAPI_H

