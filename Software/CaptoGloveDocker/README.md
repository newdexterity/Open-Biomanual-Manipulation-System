# Resources for enabling BLTE 
https://www.jaredwolff.com/get-started-with-bluetooth-low-energy/

build the docker using 
docker build -t <image_name>:<tag_name> <dockerfile_path> 
for concrete example use
docker build -t ros_capto_img:kinetic_11 .

use this command to start docker
./start kinetic_11

## Notes
* The hci and bluetooth client channels for publishing the finger sensor data seems to be linked to the settings uploaded to the captoglove in the captoglove suite.
* ToDo: upload a copy of glove settings for this code to work
* currently the thumb pressure data and index finger data overlaps in the data extracted from the channels

once in should see :
non-network local connections being added to access control list
Using 'aroa' as username, with UID=1000 and GUID=1000
Mouting host folder '/home/aroa/CaptoGloveDocker/Melodic_11_Projects/' to '/home/aroa/' in the container
Using 'docker' to run container
 * Starting system message bus dbus                                      [ OK ] 
 * Stopping bluetooth                                                    [ OK ] 
 * Starting bluetooth                                                    [ OK ] 

use bluetoothctl to connect to captogloves
bluetoothctl
scan on
***hold down button on captoglove for 7 s, until it flashes off, then back on into white/orange. want to go into white light mode
scan off
connect D4:B9:A8:85:E2:22
connect C2:4F:2D:05:78:D9
quit

sameple:
root@ARoA-NUC:/home/developer/catkin_ws# bluetoothctl
Agent registered
[bluetooth]# scan on
Discovery started
[CHG] Controller 94:B8:6D:84:32:F7 Discovering: yes
[NEW] Device 51:39:62:52:21:79 51-39-62-52-21-79
[NEW] Device 64:96:0C:52:3A:C7 64-96-0C-52-3A-C7
[NEW] Device C2:4F:2D:05:78:D9 CaptoGlove
[bluetooth]# scan off 
Discovery stopped
[CHG] Controller 94:B8:6D:84:32:F7 Discovering: no
[CHG] Device D4:B9:A8:85:E2:22 RSSI is nil
[CHG] Device 64:96:0C:52:3A:C7 TxPower is nil
[bluetooth]# connect C2:4F:2D:05:78:D9
Attempting to connect to C2:4F:2D:05:78:D9
[CHG] Device C2:4F:2D:05:78:D9 Connected: yes
Connection successful

[CHG] Device D4:B9:A8:85:E2:22 Connected: no
[NEW] Primary Service
	/org/bluez/hci0/dev_C2_4F_2D_05_78_D9/service0008
	00001801-0000-1000-8000-00805f9b34fb
	Generic Attribute Profile
[NEW] Primary Service
	/org/bluez/hci0/dev_C2_4F_2D_05_78_D9/service0034
	0000180a-0000-1000-8000-00805f9b34fb
	Device Information
[NEW] Characteristic
	/org/bluez/hci0/dev_C2_4F_2D_05_78_D9/service0034/char0035
	00002a50-0000-1000-8000-00805f9b34fb
	PnP ID
[NEW] Primary Service
	/org/bluez/hci0/dev_C2_4F_2D_05_78_D9/service0037
	0000ff05-3333-acda-0000-ff522ee73921
	Vendor specific
[NEW] Characteristic
	/org/bluez/hci0/dev_C2_4F_2D_05_78_D9/service0037/char0038
	0000f001-3333-acda-0000-ff522ee73921
	Vendor specific
[NEW] Characteristic
	/org/bluez/hci0/dev_C2_4F_2D_05_78_D9/service0037/char003a
	0000f002-3333-acda-0000-ff522ee73921
	Vendor specific
[NEW] Descriptor
	/org/bluez/hci0/dev_C2_4F_2D_05_78_D9/service0037/char003a/desc003c
	00002902-0000-1000-8000-00805f9b34fb
	Client Characteristic Configuration
[NEW] Characteristic
	/org/bluez/hci0/dev_C2_4F_2D_05_78_D9/service0037/char003d
	0000f003-3333-acda-0000-ff522ee73921
	Vendor specific
[NEW] Descriptor
	/org/bluez/hci0/dev_C2_4F_2D_05_78_D9/service0037/char003d/desc003f
	00002902-0000-1000-8000-00805f9b34fb
	Client Characteristic Configuration
[NEW] Characteristic
	/org/bluez/hci0/dev_C2_4F_2D_05_78_D9/service0037/char0040
	0000f004-3333-acda-0000-ff522ee73921
	Vendor specific
[NEW] Descriptor
	/org/bluez/hci0/dev_C2_4F_2D_05_78_D9/service0037/char0040/desc0042
	00002902-0000-1000-8000-00805f9b34fb
	Client Characteristic Configuration

roslaunch captoglove_ros_wrapper captoglove_ros_wrapper.launch 

