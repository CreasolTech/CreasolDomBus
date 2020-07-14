# CreasolDomBus
Domoticz plugin for the DomBus I/O modules that can be connected to Domoticz by RS485 serial bus.

Author: Creasol - linux@creasol.it 

Websites: https://www.creasol.it/CreasolDomBus1

          http://www.creasol.it/support/domotics-home-automation-and-diy

For any support request, please write email to linux@creasol.it or click on http://support.creasol.it


Introduction

*DomBus* are electronic devices produced by Creasol that can be used to expand inputs and outputs of home automation system controlled by Domoticz. 
One or more devices can be attached to a RS485 serial bus, so with a 4 wire cables (a twisted pair for data and 2 wires for 5-24V power supply) it's possible to connect multiple devices interconnecting the whole building.

In this folder you can find the python plugin for Domoticz, or you can install it typing the following commands from the domoticz shell (linux, raspberry):

#install wget, if not already installed

if [ ! `which wget` ]; then sudo apt install wget; fi

#change to the domoticz directory

cd /home/pi/domoticz # or other directory where domoticz is installed

#fetch the plugin

wget -O /tmp/CreasolDomBus.zip https://github.com/CreasolTech/CreasolDomBus/archive/master.zip

extract the plugin

unzip /tmp/CreasolDomBus.zip

#restart Domoticz daemon

service domoticz restart

---------

Then you can find the Creasol DomBus protocol in the Setup -> Hardware panel:

select a name (for example, dombus), select Type Creasol DomBus and enter the serial device used for RS485 bus.

Now you're ready to connect one DomBus device to the RS485 bus, and check the new I/Os in Switches panel or Setup -> Devices


