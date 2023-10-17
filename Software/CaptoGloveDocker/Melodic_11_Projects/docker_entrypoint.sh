#!/bin/bash

service dbus start
bluetoothd --debug &

rfkill block bluetooth
service bluetooth stop
rfkill unblock bluetooth
service bluetooth start 

/bin/bash
cd /home/developer
