# CreasolDomBus
Domoticz plugin for the DomBus I/O modules (attached to Domoticz by RS485 bus)
https://www.creasol.it/CreasolDomBus1

DomBus* are electronic devices that can be used to expand inputs and outputs of home automation system controlled by Domoticz. One or more devices can be attached to a RS485 serial bus, so with a 4 wire cables it's possible to connect multiple devices interconnecting the whole building.

In this folder you can find the python plugin for Domoticz, or you can install it typing the following commands from the domoticz shell (linux, raspberry):

cd /home/pi/domoticz # or other directory where domoticz is installed
wget -O /tmp/dombus.tgz http://docs.creasol.it/CreasolDomBus/domoticz_CreasolDomBus_latest.tgz
tar xvzf /tmp/dombus.tgz
service domoticz restart

then you can find the Creasol DomBus protocol in the Setup -> Hardware panel.
