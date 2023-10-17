#!/bin/bash

service dbus start
bluetoothd &

rfkill block bluetooth
service bluetooth stop
rfkill unblock bluetooth
service bluetooth start 

/bin/bash
cd /home/developer
