# CreasolDom Python Plugin
#
# Author: creasol https://creasol.it
#
"""
<plugin key="CreasolDomBus" name="Creasol DomBus RS485 modules (inputs, outputs, sensors)" author="Creasol" version="1.0.5" wikilink="http://www.domoticz.com/wiki/Creasol_Dombus" externallink="https://www.creasol.it/domotics">
    <description>
        <h2>Creasol DomBus plugin</h2><br/>
        RS485 bus protocol used to connect Domoticz controller (Raspberry PI, Linux, Windows, ...) to one or more Creasol DomBus* modules.<br/>
        Useful to expand Domoticz I/O/S: digital inputs, ultrasonic distance senrsors, 230Vac inputs, 12/24V inputs, digital output, power-optimized relays, buzzer, blinds/roller shutters, LEDs, temperature and relative humidity sensors, Led stripes, ...<br/>
        <h3>Supported modules:</h3>
        <ul style="list-style-type:square">
            <li>Creasol <b>DomBus1</b> :  <b>6*digital inputs, 1*230Vac input</b> (to detect 230V blackout, for example), <b>3*250V 5A relay outputs</b>.</li>
            <li>Creasol <b>DomBus12</b> : <b>7*I/O</b> (digital inputs/outpus, analog inputs, buzzer output, dimmer output), <b>2 open-drain outputs</b> (to drive external relays).</li>
            <li>Creasol <b>DomBus23</b> : <b>2* power-optimized relays, 1* 230Vac input, 2* analog/digital inputs, 2*I/O</b> (digital inputs/outputs, analog inputs, buzzer output, dimmer output), <b>2* 12/24V inputs with optocoupler, 1 power mosfet 30V 13A with dimmer</b> function.</li>
            <li>Creasol <b>DomBus31</b> : <b>6* SPST 250V 5A relays + 2* SPDT (NO+NC terminal blocks) 250V 10A relays</b>. All relays are power-optimized, and can be used to drive blind/roller shutter motors</li>
            <li>Creasol <b>DomBusTH</b> : <b>Temperature + Humidity sensor, 3* LEDs, 4* I/O</b> (digital/analog inputs, digital outputs, buzzer), <b>1* 0-30V analog input, 2 open-drain outputs</b> (to drive exteranl relays or LEDs, 40V 50mA max).</li>
        </ul>
        <h3>Configuration</h3>
        Please enter the right serial port, below.
    </description>
    <params>
        <param field="SerialPort" label="Serial Port device" width="150px" required="true" default="/dev/ttyUSB0"/>    
        <param field="Mode6" label="Log level" width="150px">
            <options>
                <option label="None" value="0"  default="true" />
                <option label="Errors" value="1"/>
                <option label="Warnings" value="2"/>
                <option label="Verbose" value="3"/>
                <option label="Debug" value="4"/>
                <option label="RS485 traffic, protocol 2" value="5"/>
                <option label="RS485 traffic, all protocols" value="6"/>
            </options>
        </param>
    </params>
</plugin>
"""

"""
Protocol definition (in bytes):
    preamble addr length cmd1 arg1_1 [arg1_2] [arg1_3] cmd2 arg2_1 [arg2_2] [arg2_3] ...... checksum

    preamble: 0x5a when transmitted by the master controller, 0xda when transmitted by the slave device

    address (16 bits): 
        0 => master (domoticz)
        0xffff => broadcast
        1 => default slave address (then it must be changed to permit other devices to be connected

    length: payload lenght (from cmd1 to checksum, excluding checksum)    

    cmd: syntax: CCCC ALLL  where CCCC is the command family, A=1 when this is an acknowledge, LLL is the number of arguments for that command

    args: at least 1 argument for each command

    checksum: simply a checksum from preamble to checksum, excluding checksum

"""

import Domoticz
import time
import re
import CreasolDomBusProtocol as dombus
serialConn=None

class BasePlugin:
    def __init__(self):
        return

    def onStart(self):
        global serialConn
        dombus.logLevel=int(Parameters["Mode6"])
        SerialConn = Domoticz.Connection(Name="dombus_serial", Transport="Serial", Protocol="None", Address=Parameters["SerialPort"], Baud=115200)
        SerialConn.Connect()
        Domoticz.Log("Serial connection activated")
        dombus.portsDisabledInit()
        DumpConfigToLog()
        return

    def onStop(self):
        Domoticz.Log("onStop called")

    def onConnect(self, Connection, Status, Description):
        global SerialConn
        if (Status == 0):
            Domoticz.Log("Connected successfully to: "+Parameters["SerialPort"])
            SerialConn = Connection
            #sendFrame(0xffff,0,CreasolDomProtocol.CMD_DEVICE_DISCOVERY,0)
            # send broadcasts to find attached devices
        else:
            Domoticz.Log("Failed to connect ("+str(Status)+") to: "+Parameters["SerialPort"])
            Domoticz.Debug("Failed to connect ("+str(Status)+") to: "+Parameters["SerialPort"]+" with error: "+Description)
        return True

    def onMessage(self, Connection, Data):
        dombus.rxbuffer+=Data
        #Domoticz.Log("Received bytes="+str(len(Data))+" Bytes in rxbuffer="+str(len(dombus.rxbuffer)))
        while (1):
            length=len(dombus.rxbuffer)
            if (length>=dombus.FRAME_LEN_MIN):
                #find out preamble and decode frame
                dombus.decode(Devices)
                if (length==len(dombus.rxbuffer)):
                    # decode has not removed any byte from buffer: exit waiting for new data
                    break
            else:
                #insufficient data in rxbuffer: exit
                break   
        dombus.send(Devices, SerialConn)
        return
    def onCommand(self, Unit, Command, Level, Hue):
        Domoticz.Log("onCommand called for Unit " + str(Unit) + ": Command:'" + str(Command) + "', Level:" + str(Level)+", Hue:"+str(Hue))
        newstate=0 if Command=="Off" else 1
        #send command to the module
        deviceID=Devices[Unit].DeviceID
        hwaddr="0x"+deviceID[1:5]
        frameAddr=int(hwaddr,16)
        port=int("0x"+deviceID[7:11],0)
        dombus.parseCommand(Devices,Unit,Command,Level,Hue,frameAddr,port)
        dombus.send(Devices, SerialConn)
        #Domoticz.Debug("SwitchType=="+str(Devices[Unit].SwitchType))
        if (Devices[Unit].SwitchType==7):   #Dimmer
            minLevel=1 if (re.search('OUT_ANALOG',Devices[Unit].Description)) else 5
            nv=1 if Level>=minLevel else 0
            if (Command=='Off'):
                Devices[Unit].Update(nValue=0, sValue=str(Level))
            else:
                Devices[Unit].Update(nValue=int(nv), sValue=str(Level))
        elif (Devices[Unit].SwitchType==18): #selector
            Devices[Unit].Update(nValue=Level, sValue=str(Level))
        else:
            Devices[Unit].Update(nValue=newstate, sValue=Command)
        return True
    def onNotification(self, Name, Subject, Text, Status, Priority, Sound, ImageFile):
        Domoticz.Log("Notification: " + Name + "," + Subject + "," + Text + "," + Status + "," + str(Priority) + "," + Sound + "," + ImageFile)
        dombus.send(Devices, SerialConn)
        return
    def onDisconnect(self, Connection):
        Domoticz.Log("onDisconnect called")
        return
    def onHeartbeat(self):
        #timestamp = str(int(time.time()))
        #Domoticz.Log("Heartbeat " + timestamp)
        dombus.send(Devices, SerialConn) #send frame, if any, or check if the status of a device should be transmitted (periodically transmit outputs status for every device)
        dombus.heartbeat(Devices)
        return
    def onDeviceModified(self, Unit): #called when device is modified by the domoticz frontend (e.g. when description or name was changed by the user)
        Domoticz.Debug("Device description="+Devices[Unit].Description)
        # parse configuration in the device Description...
        deviceID=Devices[Unit].DeviceID
        hwaddr="0x"+deviceID[1:5]
        frameAddr=int(hwaddr,16)
        port=int("0x"+deviceID[7:11],0)
        dombus.getDeviceID(frameAddr, port) #used to set devID variable
        Domoticz.Debug("DeviceID="+deviceID+" hwaddr="+str(hwaddr)+" frameAddr="+str(hex(frameAddr))+" port="+str(port))
        opts=Devices[Unit].Description.split(',')
        dombus.parseTypeOpt(Devices, Unit, opts, frameAddr, port)
        dombus.send(Devices, SerialConn)
        return True

global _plugin
_plugin = BasePlugin()

def onStart():
    global _plugin
    _plugin.onStart()

def onStop():
    global _plugin
    _plugin.onStop()

def onConnect(Connection, Status, Description):
    global _plugin
    _plugin.onConnect(Connection, Status, Description)
    DumpConfigToLog()

def onMessage(Connection, Data):
    global _plugin
    _plugin.onMessage(Connection, Data)

def onCommand(Unit, Command, Level, Hue):
    global _plugin
    _plugin.onCommand(Unit, Command, Level, Hue)

def onNotification(Name, Subject, Text, Status, Priority, Sound, ImageFile):
    global _plugin
    _plugin.onNotification(Name, Subject, Text, Status, Priority, Sound, ImageFile)

def onDisconnect(Connection):
    global _plugin
    _plugin.onDisconnect(Connection)

def onHeartbeat():
    global _plugin
    _plugin.onHeartbeat()

def onDeviceModified(Unit):
    global _plugin
    _plugin.onDeviceModified(Unit)

    # Generic helper functions
def DumpConfigToLog():
    for x in Parameters:
        if Parameters[x] != "":
            Domoticz.Debug( "'" + x + "':'" + str(Parameters[x]) + "'")
    Domoticz.Debug("Type of Domoticz.Device:"+str(type(Domoticz.Device)))
    Domoticz.Debug("Type of Devices:"+str(type(Devices)))
    Domoticz.Debug("Device count: " + str(len(Devices)))
    for x in Devices:
        Domoticz.Debug("Device:           " + str(x) + " - " + str(Devices[x]))
        Domoticz.Debug("Device ID:       '" + str(Devices[x].ID) + "'")
        Domoticz.Debug("DeviceID:        '" + str(Devices[x].DeviceID) + "'")
        Domoticz.Debug("Device Name:     '" + Devices[x].Name + "'")
        Domoticz.Debug("Device Unit:     '" + str(Devices[x].Unit) + "'")
        Domoticz.Debug("Device Type:     '" + str(Devices[x].Type) + "'")
        Domoticz.Debug("Device nValue:    " + str(Devices[x].nValue))
        Domoticz.Debug("Device sValue:   '" + Devices[x].sValue + "'")
        Domoticz.Debug("Device LastLevel: " + str(Devices[x].LastLevel))
    return

