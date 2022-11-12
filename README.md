# CreasolDomBus
Domoticz plugin for the **DomBus I/O/Sensors modules** that can be connected to Domoticz by RS485 serial bus.

Author: Creasol - linux@creasol.it 

Websites: https://www.creasol.it/domotics

For any support request, please write email to linux@creasol.it or click on http://support.creasol.it
Also, it's possible to get support by Telegram https://t.me/DomBus  (DomBus public channel)

# Prerequisites

* Python version 3.6.1 or later

* plugin files should be contained in the directory *DOMOTICZDIR/plugins/CreasolDomBus* 

# Introduction

*DomBus* is a family of electronic **modules for Domoticz**, designed and produced by Creasol, an italian company.

*DomBus* modules are designed to be **user friendly and minimize power consumption**, and can be used to **expand inputs, outputs and sensors for your Smart Home / Building**. 

One or more devices can be attached to a RS485 serial bus, so with a 4 wire cables (a twisted pair for data and 2 wires for 5-24V power supply) it's possible to connect multiple devices interconnecting the whole building.
As the bus carry both data and power supply, **using a 12Vdc power supply with backup battery it's possible to get home automation system working even in case of blackout / power outage**.

In this folder you can find the python plugin for Domoticz: **you can install the hardware plugin by using Python Plugin Manager, or typing the following commands from the linux shell**:

```
#install git, if not already installed
which git
if [ $? -ne 0 ]; then sudo apt install git; fi

#change to the domoticz directory / plugins
cd /home/pi/domoticz/plugins 

#fetch the Python Plugin Manager (that can be used to install/upgrade other plugins, including Creasol DomBus)
git clone https://github.com/ycahome/pp-manager

#fetch Creasol Plugin
git clone https://github.com/CreasolTech/CreasolDomBus

#restart Domoticz daemon
service domoticz restart
```


In case of Windows opeating system, please follow the installation instructions at https://www.domoticz.com/wiki/Using_Python_plugins#Installing_Python_for_Windows and please note that Domoticz should be restarted to include new python plugins that have been installed.

Then you can find the Creasol DomBus protocol in the **Setup -> Hardware panel**:

**specify a name** (for example, dombus), select **Type Creasol DomBus** and **enter the serial device** used for RS485 bus.

Now you're ready to connect one *DomBus* device to the RS485 bus, and check the new I/Os in the Switches panel or Setup -> Devices menu.


