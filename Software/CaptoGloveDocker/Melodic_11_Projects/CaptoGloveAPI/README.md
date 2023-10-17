# CaptoGloveAPI 


CaptoGloveAPI is Linux driver for ROS. Intened for use with: 
 - [captoglove_ros_wrapper](https://github.com/fzoric8/captoglove_ros_wrapper) 
 - [docker_files](https://github.com/fzoric8/docker_files)
 - [protobuffers](https://github.com/fzoric8/protobuffers)  

# In order to realize full functionality of CaptoGloves on Linux PCs this are very valuable resources: 

 - [Signals & Slots](https://doc.qt.io/qt-5/signalsandslots.html) 
 - [Event driven architecture](https://www.oreilly.com/library/view/software-architecture-patterns/9781491971437/ch02.html) 
 - [QtBluetooth](https://doc.qt.io/qt-5/qtbluetooth-index.html) 
 - [Qt BLE overview](https://doc.qt.io/qt-5/qtbluetooth-le-overview.html)
 - [Creating Qt project files](https://doc.qt.io/qt-5/qmake-project-files.html) 
 - [BLE services and characteristics](https://www.oreilly.com/library/view/getting-started-with/9781491900550/ch04.html) 

# Classes that are heavily used throught development: 
 - [QtLowEnergyController](https://doc.qt.io/qt-5/qlowenergycontroller.html) 
 - [QtLowEnergyService](https://doc.qt.io/qt-5/qlowenergyservice.html) 
 - [QtLowEnergyCharacteristic](https://doc.qt.io/qt-5/qlowenergycharacteristic.html) 
 - [QtLowEnergyDescriptor](https://doc.qt.io/qt-5/qlowenergydescriptor.html) 
 
# Qt uses BlueZ as underlying technology to support this, this resources were extremely helpful:
 - [Get started with BLE](https://www.jaredwolff.com/get-started-with-bluetooth-low-energy/)
 - [Control With BlueZ](https://learn.adafruit.com/reverse-engineering-a-bluetooth-low-energy-light-bulb/control-with-bluez) 

# [GAP and GATT profile](https://learn.adafruit.com/introduction-to-bluetooth-low-energy/gap)

# [Bluetoothctl]

Used for connecting to BLE device on linux. Sometimes has problems, check following [link](https://stackoverflow.com/questions/48279646/bluetoothctl-no-default-controller-available)

# Dependencies 

Depends on `pthread` library which must be used and gcc 7.5.0

### How to install gcc 7.5.0?

```
sudo apt-get install -y software-properties-common
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt update
sudo apt install g++-7 -y
```

Add symbolic pointers to new version: 

```
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 60 \
                         --slave /usr/bin/g++ g++ /usr/bin/g++-7 
sudo update-alternatives --config gcc
gcc --version
g++ --version
```

## Relevant code 

There is [LE scanner](https://code.qt.io/cgit/qt/qtconnectivity.git/tree/examples/bluetooth/lowenergyscanner?h=5.15) used as example. 
And there is application where we have continuous exchange of data between BLE server and client on 
following [link](https://code.qt.io/cgit/qt/qtconnectivity.git/tree/examples/bluetooth/heartrate-game) 

## TBD:

- [ ] Document existing code to comply to doxygen standard
- [ ] Explore different characteristics of CaptoGloves (gyroscope/accelerometer) 
- [ ] Implement methods to enable gyroscope/accelerometer control  


## How to build: 

You can build CaptoGloveAPI-buildGCC_x64 as follows: 
```
cd CaptoGloveAPI-buildGCC_x64
qmake ../CaptoGloveAPI/CaptoGloveAPI.pro 
make -j8 
```

## TODO: 
- [x] Initialize controller 
- [x] Establish connection with BLE device
- [x] Scan services 
- [x] Check how to discover characteristics
- [x] Read battery value
- [x] Scan characteristics 
- [x] Check for updates of certain characteristsics/services
- [x] Add Logger / Needed for ROS
- [x] Add ros wrapper 
- [x] Use CaptoGloveAPI with gloves to control drone 
- [ ] Check Logger functionality based on signals 
- [ ] Map all signals and slots 
- [ ] Refactor code 
- [ ] Error handling (Impossible to find device, adapter not found, can't connect, connection not working) 
- [ ] Add protobuffer messages to enable writing in them
