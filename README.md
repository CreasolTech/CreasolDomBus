# CreasolDomBus
Domoticz plugin for the DomBus I/O/Sensors modules that can be connected to Domoticz by RS485 serial bus.

Author: Creasol - linux@creasol.it 

Websites: https://www.creasol.it/domotics

For any support request, please write email to linux@creasol.it or click on http://support.creasol.it
Also, it's possible to get support by Telegram https://t.me/DomBus  (DomBus public channel)


Introduction

*DomBus* are electronic modules designed for Domoticz produced by Creasol, an italian company.

DomBus modules can be used to expand inputs, outputs and sensors for your home automation system. 

One or more devices can be attached to a RS485 serial bus, so with a 4 wire cables (a twisted pair for data and 2 wires for 5-24V power supply) it's possible to connect multiple devices interconnecting the whole building.
As the bus carry both data and power supply, using a 12Vdc power supply with backup battery it's possible to get home automation system working even in case of blackout / power outage.

In this folder you can find the python plugin for Domoticz: you can install it by using Python Plugin Manager, or typing the following commands from the linux shell:

#install git, if not already installed

if [ ! `which git` ]; then sudo apt install git; fi

#change to the domoticz directory / plugins

cd /home/pi/domoticz/plugins # or other directory where domoticz is installed

#fetch the Python Plugin Manager (that can be used to install other plugins, including Creasol DomBus)

git clone https://github.com/ycahome/pp-manager

#fetch Creasol Plugin

git clone https://github.com/CreasolTech/CreasolDomBus

#restart Domoticz daemon

service domoticz restart

---------

Then you can find the Creasol DomBus protocol in the Setup -> Hardware panel:

select a name (for example, dombus), select Type Creasol DomBus and enter the serial device used for RS485 bus.

Now you're ready to connect one DomBus device to the RS485 bus, and check the new I/Os in the Switches panel or Setup -> Devices menu.


