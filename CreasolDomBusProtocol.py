# Define constants and functions needed by CreasolDom RS485 bus protocol 

"""
Protocol definition (in bytes):
    ========================= Protocol 1 ========================
    0xda addr[2] length cmd|ACK|LEN arg1_1 [arg1_2] [arg1_3] cmd2 arg2_1 [arg2_2] [arg2_3] ...... checksum
    
    preamble: 0x5a when transmitted by the master controller, 0xda when transmitted by the slave device
    
    address (2 bytes): 
        0 => master (domoticz)
        0xffff => broadcast
        1 => default slave address

    length: payload lenght (from cmd1 to checksum, excluding checksum)    

    cmd: syntax: CCCC ALLL  where CCCC is the command family, A=1 when this is an acknowledge, LLL is the number of arguments for that command
    
    args: at least 1 argument for each command

    checksum: simply a checksum from preamble to checksum, excluding checksum

    ======================== Protocol 2 ======================== @20201216
    0x3a dstAddr[2] srcAddr[2] length cmd|ACK|len/2 port parameters[1,3,5,7,9,11] cmd2|ACK|len2/2 port2 parameters2 cmd3|ACK|len3/2 port3 parameters3


    
"""
import Domoticz
import time
import struct
import re
import json
import math

#if 1, when a module does not transmit for more than 15 minutes (MODULE_ALIVE_TIME), it will appear in red (TimedOut)
PROTOCOL1_WITH_PERIODIC_TX=0    # set to 1 if all existing modules transmit their status periodically (oldest modules with protocol 1 did not)

#some constants
FRAME_LEN_MIN=6
FRAME_LEN_MIN2=9    #Min length of frame for protocol 2
FRAME_LEN_MAX=31    #max length for TX (devices cannot handle long frames)
FRAME_LEN=3
FRAME_LEN2=5
FRAME_HEADER=4
FRAME_HEADER2=6
PREAMBLE_MASTER=0x5a
PREAMBLE_DEVICE=0xda
PREAMBLE=0x3a           #Preamble for protocol 2
CMD_LEN_MASK=0x07
CMD_MASK=0xf0
CMD_ACK=0x08

TX_RETRY=10                     #max number of retries
TX_RETRY_TIME=80                # ms: retry every TX_RETRY_TIME * 2^retry
PERIODIC_STATUS_INTERVAL=300    #seconds: refresh output status to device every 5 minutes
MODULE_ALIVE_TIME=900           #if no frame is received in this time, module is considered dead (and periodic output status will not be transmitted)

CMD_CONFIG=0x00                 #Config port
CMD_GET=0x10                    #Get status
CMD_SET=0x20                    #Set outputs/values
CMD_DCMD_CONFIG=0xe0            #Send DCMD configuration
CMD_DCMD=0xf0                   #Receive DCMD command from Dombus

SUBCMD_CALIBRATE=0x00           #Send calibration value to a temperature/humidity sensor
SUBCMD_SET=0x01                 #Send a 16bit value (used to change modbus device address and evseMaxCurrent)
SUBCMD_SET2=0x02                #Send parameter 2 (16bit value)
SUBCMD_SET3=0x03                #Send parameter 3 (16bit value)
SUBCMD_SET4=0x04                #Send parameter 4 (16bit value)
SUBCMD_SET5=0x05                #Send parameter 5 (16bit value)
SUBCMD_SETMAX=0x10              #Send parameter 16

PORTTYPE_DISABLED=0x0000        #port not used
PORTTYPE_OUT_DIGITAL=0x0002     #digital, opto or relay output
PORTTYPE_OUT_RELAY_LP=0x0004    #relay output with lowpower PWM
PORTTYPE_OUT_LEDSTATUS=0x0008   #output used as led status
PORTTYPE_OUT_DIMMER=0x0010      #dimmer output, 0-100%
PORTTYPE_OUT_BUZZER=0x0020      #buzzer outputs (2 ports used as buzzer output, in push-pull)
PORTTYPE_IN_AC=0x0040           #input AC 50Hz (with optocoupler)
PORTTYPE_IN_DIGITAL=0x0080      #input digital
PORTTYPE_IN_ANALOG=0x0100       #input analog (ADC)
PORTTYPE_IN_TWINBUTTON=0x0200   #2 buttons connected to a single input through a resistor
PORTTYPE_IN_COUNTER=0x0400      #input pulses that increase a counter (incremental)
PORTTYPE_1WIRE=0x1000           #1 wire
PORTTYPE_SENSOR_DISTANCE=0x2000 #distance measurement (send a pulse and measure echo delay)
PORTTYPE_SENSOR_TEMP=0x4000     #Temperature
PORTTYPE_SENSOR_HUM=0x8000      #Relative Humidity
PORTTYPE_SENSOR_TEMP_HUM=0xc000 #Temp+Hum
PORTTYPE_OUT_BLIND=0x01000000   #Blind output, close command (next port of DomBus device will be automatically used as Blind output, open command)
PORTTYPE_OUT_ANALOG=0x02000000  #0-10V output, 1% step, 0-100
PORTTYPE_CUSTOM=0x80000000      #custom port with only 1 function

PORTOPT_NONE=0x0000             #No options
PORTOPT_INVERTED=0x0001         #Logical inverted: MUST BE 1
PORTOPT_SELECTOR=0x0002         #Custom port configured as a selection switch to show/set different values
PORTOPT_DIMMER=0x0004           #Dimmer slide
PORTOPT_ADDRESS=0x0100          #Modbus device address
PORTOPT_IMPORT_ENERGY=0x0102    #Total import energy in Wh*10 [32bit]
PORTOPT_EXPORT_ENERGY=0x0104    #Total export energy in Wh*10 [32bit]
PORTOPT_VOLTAGE=0x0106          #Voltage in Volt/10
PORTOPT_POWER_FACTOR=0x0108     #Power factore 1/1000
PORTOPT_FREQUENCY=0x010a        #Frequency Hz/100

PORTTYPE={PORTTYPE_OUT_DIGITAL:244, PORTTYPE_OUT_RELAY_LP:244, PORTTYPE_OUT_LEDSTATUS:244, PORTTYPE_OUT_DIMMER:244, PORTTYPE_OUT_BUZZER:244, PORTTYPE_IN_AC:244, PORTTYPE_IN_DIGITAL:244, PORTTYPE_IN_ANALOG:244, PORTTYPE_IN_TWINBUTTON:244, PORTTYPE_IN_COUNTER:243, PORTTYPE_SENSOR_HUM:81, PORTTYPE_SENSOR_TEMP:80, PORTTYPE_SENSOR_TEMP_HUM:82, PORTTYPE_SENSOR_DISTANCE:243, PORTTYPE_OUT_BLIND:244, PORTTYPE_OUT_ANALOG:244}

PORT_TYPENAME={PORTTYPE_OUT_DIGITAL:"Switch", PORTTYPE_OUT_RELAY_LP:"Switch", PORTTYPE_OUT_LEDSTATUS:"Switch", PORTTYPE_OUT_DIMMER:"Dimmer", PORTTYPE_OUT_BUZZER:"Switch", PORTTYPE_IN_AC:"Switch", PORTTYPE_IN_DIGITAL:"Switch", PORTTYPE_IN_ANALOG:"Voltage", PORTTYPE_IN_TWINBUTTON:"Selector Switch", PORTTYPE_IN_COUNTER:"Counter Incremental", PORTTYPE_SENSOR_HUM:"Humidity", PORTTYPE_SENSOR_TEMP:"Temperature", PORTTYPE_SENSOR_TEMP_HUM:"Temp+Hum", PORTTYPE_SENSOR_DISTANCE:"Distance", PORTTYPE_OUT_BLIND:"Switch", PORTTYPE_OUT_ANALOG:"Dimmer", PORTTYPE_CUSTOM:"Dimmer"}

PORTTYPES={
        "DISABLED":0x0000,          # port not used
        "OUT_DIGITAL":0x0002,       # relay output
        "OUT_RELAY_LP":0x0004,      # relay output
        "OUT_LEDSTATUS":0x0008,     # output used as led status
        "OUT_DIMMER":0x0010,        # dimmer output
        "OUT_BUZZER":0x0020,        # buzzer output (2 ports, push-pull)
        "IN_AC":0x0040,             # input AC 50Hz (with optocoupler)
        "IN_DIGITAL":0x0080,        # input digital
        "IN_ANALOG":0x0100,         # input analog (ADC)
        "IN_TWINBUTTON":0x0200,     # 2 buttons connected to a single input through a resistor
        "IN_COUNTER":0x0400,        # pulse counter
        "DISTANCE":0x2000,          # measure distance in mm
        "TEMPERATURE":0x4000,       # temperature
        "HUMIDITY":0x8000,          # relative humidity
        "TEMP+HUM":0xc000,          # temp+hum
        "OUT_BLIND":0x01000000,     # blind with up/down/stop command
        "OUT_ANALOG":0x02000000,    # 0-10V output, 0-100, 1% step
        "CUSTOM":0x80000000,        # Custom port (enabled only if PORTOPT is specified)
        }

PORTOPTS={
        "NORMAL":0x0000,            # no options defined
        "INVERTED":0x0001,          # input or output is inverted (logic 1 means the corresponding GPIO is at GND
        "SELECTOR":0x0002,            # Selection switch
        "DIMMER":0x0004,            # Dimmer
        }

PORTTYPENAME={  #Used to set the device TypeName
        "DISABLED":"Switch",
        "OUT_DIGITAL":"Switch",
        "OUT_RELAY_LP":"Switch",
        "OUT_LEDSTATUS":"Switch", # output used as led status
        "OUT_DIMMER":"Dimmer",
        "OUT_BUZZER":"Switch",
        "IN_AC":"Switch",
        "IN_DIGITAL":"Switch",
        "IN_ANALOG":"Voltage",
        "IN_TWINBUTTON":"Selector Switch",
        "IN_COUNTER":"Counter Incremental",
        "HUMIDITY":"Humidity",
        "TEMPERATURE":"Temperature",
        "TEMP+HUM":"Temp+Hum",
        "DISTANCE":"Distance",
#        "OUT_BLIND":"Venetian Blinds EU", #not available in domoticz yet. hardware/plugins/PythonObjects.cpp must be updated!
        "OUT_BLIND":"Switch",
        "OUT_ANALOG":"Dimmer",
        "CUSTOM":"Switch",
        }

DCMD_IN_EVENTS={
        "NONE":     0,
        "OFF":      1,
        "ON":       2,
        "PULSE":    3,      #short pulse
        "PULSE1":   4,      #1s pulse
        "PULSE2":   5,      #2s pulse
        "PULSE4":   6,      #4s pulse
        "DIMMER":   7,      #Dimming control
        "VALUE":    8,      #value (sensor, voltage, ...)
        "ONUP":     9,      #Twinbutton UP
        "PULSEUP":  10,
        "PULSEUP1": 11,
        "PULSEUP2": 12,
        "PULSEUP4": 13,
        "MAX":      14,      #max number of events
        }

DCMD_OUT_CMDS={
        "NONE":     0,      
        "OFF":      1,      #Turn off output
        "ON":       2,      #turn ON output
        "TOGGLE":   3,      #toggle output ON -> OFF -> ON -> ....
        "DIMMER":   4,      #set value
        "DOWN":     5,      #Blind DOWN
        "UP":       6,      #Blind UP
        "MAX":      7,      #Max number of commands
        }


rxbuffer=bytearray()         #serial rx buffer
txbuffer=bytearray()         #serial tx buffer
rxbufferindex=0             
frameLen=0
frameAddr=0
checksumValue=0
devicesFull=False   # True if no more space to store new devices

LASTRX=0        # first field in modules[]
LASTTX=1        # second field in modules[]
LASTSTATUS=2    # third field in modules[]
LASTPROTOCOL=3  # forth field in modules[]
LASTRETRY=4     # fifth field in modules[]: number of retries (used to compute the tx period)
modules={}      # modules[frameAddr]=[lastRx, lastTx, lastSentStatus, protocol]
modulesAskConfig=[] #Dict with list of modules that require a new request of configuration (send CMD_CONFIG to ask for their configuration)

TXQ_CMD=0
TXQ_CMDLEN=1
TXQ_CMDACK=2
TXQ_PORT=3
TXQ_ARGS=4
TXQ_RETRIES=5
#txQueue[frameAddr].append([cmd,cmdLen,cmdAck,port,args,retries])
txQueue={}      # tx queue for each module

LOG_NONE=0
LOG_ERR=1
LOG_WARN=2
LOG_INFO=3
LOG_DEBUG=4
LOG_DUMP=5
LOG_DUMPALL=6

LOG_NAMES=["NONE: ","ERROR:","WARN: ","INFO: ","DEBUG:","DUMP: "]
logLevel=LOG_NONE  #defaul log level: will be initialized by onStart()

PORTSDISABLED="plugins/CreasolDomBus/portsDisabled.json"

counterTime={}  #record the last counter update
counterOld={}   #record the last value of counter (incremental value) to avoid decoding the same command twice, and avoid loosing of data

def __init__(self):
    return

def portsDisabledWriteNow():
    global portsDisabled
    with open(PORTSDISABLED, 'w') as fd:
        json.dump(portsDisabled,fd)

def portsDisabledInit():
    global portsDisabled, portsDisabledWrite
    portsDisabledWrite=0    #time*heartbeat before writing the portsDisabled dict on json file
    try:
        fd=open(PORTSDISABLED)
    except:
        # doesn't exist?
        #Log(LOG_ERR,"Error opening file "+PORTDISABLED+": initializing portsDisabled...")
        portsDisabled=dict()
        portsDisabledWriteNow()
    else:
        portsDisabled=json.load(fd)
        #portsDisabled={0xff23="3:4:5:11", 0xff31="7:8"}  disabled ports, separated by colon. Port 1 can never be disabled!

def Log(level, msg):
    global logLevel
    if (level<=logLevel):
        Domoticz.Log(LOG_NAMES[level]+msg)
    return

def HRstatus(hum): #return normal, comfort, dry, wet status depending by relative humdity argument
    if (hum<25):
        return "2"  #dry
    elif (hum>70):
        return "3"  #wet
    elif (hum>=40 and hum<=60): #comfortable
        return "1"
    else:
        return "0"  #normal

def checksum(protocol, buffer):
    global checksumValue
    checksumValue=0
    if (protocol==1):
        length=buffer[FRAME_LEN]+FRAME_HEADER
    else:
        length=buffer[FRAME_LEN2]+FRAME_HEADER2
    for i in range(0, length):
        checksumValue+=buffer[i]
    checksumValue&=0xff        
    return

def dump(protocol, buffer,frameLen,direction):
    #buffer=frame buffer
    #frameLen=length of frame in bytes
    #direction="RX" or "TX"
    if (protocol==1):
        f="P:1 "
        fl=frameLen if (frameLen<=len(buffer)) else len(buffer) #manage the case that frame is received only partially
        for i in range(0, fl):
            f+="%.2x " % int(buffer[i])
        Log(LOG_DUMP,direction+" frame: "+f)
    elif (logLevel>=LOG_INFO):
        f="P:2 "
        f+="%.2x " % int(buffer[0])
        f+="%.4x " % (int(buffer[3])*256+int(buffer[4]))
        f+="-> "
        f+="%.4x " % (int(buffer[1])*256+int(buffer[2]))
        f+="%.2d " % int(buffer[5]) #length
        i=FRAME_HEADER2
        while (i<frameLen-1):
            #TODO: write SET 01 00 SET 02 01 SET 03 0a SET
            #TODO: write CFG 02 FF
            cmd=int(buffer[i])&CMD_MASK
            cmdAck=int(buffer[i])&CMD_ACK
            cmdLen=(int(buffer[i])&CMD_LEN_MASK)*2
            if (cmdLen==0):
                cmdLen=2    #minimum cmdLen
            if (cmdAck):
                f+='A-'
            if (cmd==CMD_CONFIG):
                f+='CFG '
                if (cmdAck and (int(buffer[i+1])&0xf0)):
                    #whole port configuration => cmdLen without any sense
                    cmdLen=frameLen - FRAME_HEADER2 - 2 #force cmdLen to the whole frame
            elif (cmd==CMD_SET):
                f+='SET '
            elif (cmd==CMD_GET):
                f+='GET '
            elif (cmd==CMD_DCMD_CONFIG):
                f+='DCMDCFG '
            elif (cmd==CMD_DCMD):
                f+='DCMD '
            else:
                f+="%.2x " % int(buffer[i])
            i+=1
            if (i+cmdLen) > len(buffer):
                Log(LOG_WARN, f"Frame error: cmdLen={cmdLen} is too much for the available data in buffer[]")
                Log(LOG_WARN, f"Frame={f}...")
                return 
            for j in range(0, cmdLen):
                f+="%.2x " % int(buffer[i+j])
            f+='| '
            i+=cmdLen
        f+="%.2x " % int(buffer[i]) #checksum
        if (cmd==CMD_DCMD): #log DCMD command with priority INFO, so it's possible to monitor traffic between DomBus modules
            Log(LOG_INFO,direction+" frame: "+f)
        else:
            Log(LOG_DUMP,direction+" frame: "+f)
    return

def getDeviceID(frameAddr, port):
    global deviceID
    global devID
    global deviceAddr
    deviceID="H{:04x}_P{:04x}".format(frameAddr, port)
    devID="{:x}.{:x}".format(frameAddr, port)
    deviceAddr="0x{:04x}".format(frameAddr)
    return

def getDeviceUnit(Devices,findUnitFreeMax):
    #given the deviceID (global var), scan Devices to find the Unit of the associated deviceID
    #Also, set the variables 
    #  UnitFree=greatest free unit, or lowest free unit
    #
    global deviceID, UnitFree
    found=0xffff
    unitmax=0
    UnitFree=0xffff
    for unit in range(1,256):
        if (unit in Devices):
            #Devices[unit] exists
            if (Devices[unit].DeviceID==deviceID):
                #this unit corresponds with the deviceID
                found=unit
                unitmax=unit+1
                if (findUnitFreeMax==0):
                    #stop scanning
                    break
        else:
            #unit is free
            if (UnitFree>unit):
                UnitFree=unit
    #Log(LOG_DEBUG,"getDeviceUnit(): found unit="+str(found)+" UnitFree="+str(UnitFree)+" UnitMax="+str(unitmax))
    if (unitmax>0 and unitmax<256):
        UnitFree=unitmax
    return found

def getOpt(d, parameter):
    #get parameter value from device description
    #d=device
    #parameter=STRING (must be uppercase)
    #e.g. a=getOpt("A=")
    #return a string:
    # "false" if not defined
    # "value" if A=value
    # "1" if A=   (needed when getOpt("INVERTED"), for example
    opts=d.Description.upper().split(',')
    length=len(parameter)
    for opt in opts:
        opt=opt.strip()
        if (opt[:length]==parameter):
            value=(opt[length:])
            if (len(value)):
                return value
            else:
                return "1"
    return "false"

def convertValueToDombus(d, value):
    #convert a value (20.5°C, 13.6V, ...) to dombus value (16bit integer)
    #return the converted value
    if (d.Type==PORTTYPE[PORTTYPE_SENSOR_TEMP]):
        return int(value*10+2731)   #temperature
    elif (d.Type==PORTTYPE[PORTTYPE_SENSOR_HUM]):
        return int(value*10)    #Humidity
    elif (d.Type==243 and (d.SubType==27 or d.SubType==8)): #distance or voltage
        #extract A and B, if defined, to compute the right value VALUE=A*dombus_value+B => dombus_value=(VALUE-B)/A
        v=getOpt(d,"A=")
        a=float(v) if (v!="false") else 1
        v=getOpt(d,"B=")
        b=float(v) if (v!="false") else 0
        return int((value-b)/a)

def txQueueAdd(protocol, frameAddr, cmd,cmdLen,cmdAck,port,args,retries,now):
    #add a command in the tx queue for the specified module (frameAddr)
    #if that command already exists, update it
    #cmdLen=length of data after command (port+args[])
    global txQueue
    sec=int(time.time())
    ms=int(time.time()*1000)
    #Log(LOG_DEBUG,f"txQueueAdd({protocol}, {frameAddr}, cmd={cmd},  cmdLen={cmdLen}, cmdAck={cmdAck}, port={port}, args={args}, retries={retries}, now={now})")
    if protocol==0:
        # check if module already in modules[]
        if (frameAddr in modules):
            protocol=modules[frameAddr][LASTPROTOCOL]
    if len(txQueue)==0 or frameAddr not in txQueue:
        #create txQueue[frameAddr]
        txQueue[frameAddr]=[[cmd, cmdLen, cmdAck, port, args, retries]]
        #Log(LOG_DEBUG,"txQueueAdd (frameAddr does not exist) frameAddr="+hex(frameAddr)+" cmd="+hex(cmd|cmdAck|cmdLen)+" port="+hex(port))
    else:
        found=0
        for f in txQueue[frameAddr]:
            #f=[cmd,cmdlen,cmdAck,port,args[]]
            if (f[TXQ_CMD]==cmd and f[TXQ_CMDLEN]==cmdLen and f[TXQ_PORT]==port and (cmd!=CMD_CONFIG or len(args)==0 or args[0]==f[TXQ_ARGS][0])): #if CMD_CONFIG, also check that SUBCMD is the same
                #command already in txQueue: update values
                f[TXQ_CMDACK]=cmdAck
                f[TXQ_ARGS]=args
                if (f[TXQ_RETRIES]<retries):
                    f[TXQ_RETRIES]=retries
                found=1
                break
        if (found==0):
            txQueue[frameAddr].append([cmd,cmdLen,cmdAck,port,args,retries])
        #txQueueRetry: don't modify it... transmit when retry time expires (maybe now or soon)
    #check that modules[frameAddr] exists
    #Log(LOG_DEBUG,"txQueueAdd: frameAddr="+hex(frameAddr)+" cmd="+hex(cmd|cmdAck|cmdLen)+" port="+hex(port))
    if (frameAddr not in modules):
        # add frameAddr in modules
        #                   lastRx  lastTx  lastSentStatus => lastTx=0 => transmit now
        modules[frameAddr]=[sec,    ms,     sec+3-PERIODIC_STATUS_INTERVAL, protocol, 0] #transmit output status in 3 seconds
    else:
        #frameAddr already in modules[]
        if (protocol!=0):
            modules[frameAddr][LASTPROTOCOL]=protocol
        if (now):
            modules[frameAddr][LASTTX]=0 #transmit now
    return        

def txQueueAskConfig(protocol,frameAddr):
    global devicesFull
    if (devicesFull==0):
        txQueueAdd(protocol, frameAddr,CMD_CONFIG,1,0,0xff,[],TX_RETRY,1)    #port=0xff to ask full configuration 
    return 

def txQueueRemove(frameAddr,cmd,port,arg1):
    # if txQueue[frameAddr] esists, remove cmd and port from it.
    # if cmd==255 and port==255 => remove all frames for module frameAddr
    global txQueue
    removeItems=[]
    if len(txQueue)!=0 and frameAddr in txQueue:
        for f in txQueue[frameAddr][:]:
            #Log(LOG_DEBUG,"f="+str(f))
            #f=[cmd,cmdlen,cmdAck,port,args[],retries]
            if (((cmd&port)==255) or (f[TXQ_CMD]==cmd and f[TXQ_PORT]==port and (len(f[TXQ_ARGS])==0 or f[TXQ_ARGS][0]==arg1))):
                txQueue[frameAddr].remove(f)
    return
    
def txOutputsStatus(Devices,frameAddr):
    # transmit the status of outputs for the device frameAddr
    # Domoticz.Log("Send outputs status for device "+hex(frameAddr))
    for Device in Devices:
        deviceIDMask="H{:04x}_P".format(frameAddr)
        d=Devices[Device]
        if (d.Used==1 and d.DeviceID[:7]==deviceIDMask):
            # device is used and matches frameAddr
            # check that this is an output
            if (d.Type==PORTTYPE[PORTTYPE_OUT_DIGITAL] and re.search('(OUT_DIGITAL|OUT_RELAY_LP|OUT_DIMMER|OUT_BUZZER|OUT_ANALOG)',d.Description)):
                # output! get the port and output state
                port=int("0x"+d.DeviceID[7:11],0)
                if (hasattr(d,'SwitchType') and d.SwitchType==7): #dimmer
                    if (re.search("OUT_ANALOG|CUSTOM.SELECTOR",d.Description)):
                        level=int(d.sValue) if d.nValue==1 else 0 #1% step
                    else:
                        level=int(int(d.sValue)/5) if d.nValue==1 else 0    #soft dimmer => 5% step
                    txQueueAdd(0, frameAddr,CMD_SET,2,0,port,[level],TX_RETRY,1) #Level: from 0 to 20 = 100%
                elif (hasattr(d,'SwitchType') and d.SwitchType==18): #selector
                    txQueueAdd(0, frameAddr,CMD_SET,2,0,port,[d.nValue],TX_RETRY,1) #Level: 0, 10, 20, ....
                else:
                    txQueueAdd(0, frameAddr,CMD_SET,2,0,port,[d.nValue],TX_RETRY,1)
    return

def parseCommand(Devices, unit, Command, Level, Hue, frameAddr, port):
    newstate=1 if Command=="On" else 0
    #send command to the module
    d=Devices[unit]
    deviceID=d.DeviceID
    #add command in the queue
    if (not hasattr(d,'SwitchType')): 
        Log(LOG_DEBUG,"Type="+str(d.Type)+" SubType="+str(d.SubType)+" Name="+str(d.Name))
    if (d.Type==PORTTYPE[PORTTYPE_OUT_DIGITAL]):
        if (hasattr(d,'SwitchType') and d.SwitchType==7): #dimmer
            if (Command=='Off'):
                txQueueAdd(0, frameAddr,CMD_SET,2,0,port,[0],TX_RETRY,1) #Level: from 0 to 20 = 100%
            else:
                if (re.search('OUT_ANALOG|CUSTOM.DIMMER',d.Description)):
                    txQueueAdd(0, frameAddr,CMD_SET,2,0,port,[int(Level)],TX_RETRY,1) #Level: from 0 to 100 = 100% (1% step)
                    #Log(LOG_INFO,f"Dimmer level set to {Level} by parseCommand()")
                else:
                    txQueueAdd(0, frameAddr,CMD_SET,2,0,port,[int(Level/5)],TX_RETRY,1) #Level: from 0 to 20 = 100% (5% step)
                #nValue=0 if Level==0 else 1
                #d.Update(nValue=1, sValue="20")
        elif (hasattr(d,'SwitchType') and d.SwitchType==18): #selector
            txQueueAdd(0, frameAddr,CMD_SET,2,0,port,[Level],TX_RETRY,1) #Level: 0, 10, 20, ....
        elif (hasattr(d,'SwitchType') and (d.SwitchType==15 or d.SwitchType==14)): #venetian blinds
            if (Command=='Off'): #Open
                v=getOpt(d,"TIMEOPEN=")
                duration=int(v) if (v!="false") else 25
                if (duration<=0 or duration>120): duration=25
                newstate=0x80|duration   
            elif (Command=='On'): #Close
                v=getOpt(d,"TIMECLOSE=")
                duration=int(v) if (v!="false") else 25
                if (duration<=0 or duration>120): duration=25
                newstate=duration   
            else: #Stop
                newstate=0
            #newstate=0 => STOP. newstate&0x80 => open for newstate&0x7f seconds, else close for newstate&0x7f seconds
            txQueueAdd(0, frameAddr,CMD_SET,2,0,port,[newstate],TX_RETRY,1) 
        else: #normal switch
            txQueueAdd(0, frameAddr,CMD_SET,2,0,port,[newstate],TX_RETRY,1)
    else:
        Log(LOG_DEBUG,"parseCommand: ignore command because Type="+str(d.Type)+" SubType="+str(d.SubType)+" Name="+d.Name+" has not attribute SwitchType")
    return

def parseTypeOpt(Devices, Unit, opts, frameAddr, port):
    #read device description and set the last defined type (if written in the description, else transmit 0xffff that mean NO-CHANGE) and the port options (ORed) (if no options are written in the description, transmits 0xffff that means NO CHANGE).
    global txQueue, portsDisabled, portsDisabledWrite, deviceID, devID
    getDeviceID(frameAddr,port)
    Log(LOG_DEBUG,"Parse Description field for device "+devID)
    setOpt=0
    setType=0
    setNewAddr=''   #new address for modbus device
    setHwaddr=0
    setCal=32768    #calibration offset 32768=ignore
    setTypeName=''
    typeName=''
    setOptDefined=0
    setTypeDefined=0
    setDisableDefined=0
    setOptNames=""
    setMaxCurrent=0
    setMaxPower=0
    setStartPower=0
    setStopTime=0
    setAutoStart=0
    Options=Devices[Unit].Options
    #dombus command
#    dcmd=[  # IN_EVENT,                 inValueL,   inValueH,   hwaddr, port,   outCmd,                 outValue
#            [ DCMD_IN_EVENTS['NONE'],   0,          0,          0,      0,      DCMD_OUT_CMDS['NONE'],  0 ],
#            ]
    dcmd=[]   
    for opt in opts:
        opt=opt.strip()
        optu=opt.upper()
        Log(LOG_DEBUG,"opt="+str(opt))
        if optu in PORTTYPES:
            #opt="OUT_DIGITAL" or DISTANCE or ....
            setType=PORTTYPES[optu]      # setType=0x2000 if DISTANCE is specified
            setTypeDefined=1            #setTypeDefined=1
            setTypeName=optu             #setTypeName=DISTANCE
            typeName=PORTTYPENAME[optu]  #typeName=Temperature
            Log(LOG_DEBUG,"opt="+str(opt)+" setType="+str(PORTTYPES[optu])+" typeName="+str(PORTTYPENAME[optu]))
        elif optu in PORTOPTS:
            if (optu=="NORMAL"):
                setOpt=0
                setOptNames=''
            else:
                setOpt=setOpt|PORTOPTS[optu]
                setOptNames+=opt+","
            setOptDefined=1
        elif optu[:2]=="A=":
            setOptNames+=opt+","
        elif optu[:2]=="B=":
            setOptNames+=opt+","
        elif optu[:9]=="FUNCTION=":
            # Used to convert an analog value to another
            Options['function']=optu[9:]
            setType=PORTTYPE[PORTTYPE_SENSOR_TEMP]
            setTypeDefined=1
            typeName="Temperature"
            setOptNames+=opt+","
        elif optu[:4]=="CAL=":       # calibration value: should be expressed as integer (e.g. temperatureOffset*10)
            setCal=int(float(opt[4:])*10) 
        elif optu[:9]=="TYPENAME=":
            typeName=opt[9:]
            setOptNames+=opt+","
        elif optu[:9]=="OPPOSITE=": #Used with kWh meter to set power to 0 when the opposite counter received a pulse (if import power >0, export power must be 0, and vice versa)
            if (Devices[Unit].Type==243 and Devices[Unit].SubType==29): #kWh
                opposite=int(float(opt[9:]))
                if (Devices[opposite].Type==243 and Devices[opposite].SubType==29): #opposite unit is a kWh meter
                    Options['opposite']=str(opposite)
                    setOptNames+=opt+","
        elif optu[:8]=="DIVIDER=": #Used with kWh meter to set how many pulses per kWh, e.g. 1000 (default), 2000, 1600, ...
            if (Devices[Unit].Type==243 and Devices[Unit].SubType==29): #kWh
                divider=int(float(opt[8:]))
                if divider!=0:
                    Options['divider']=str(divider)
                    Log(LOG_DEBUG, "Set DIVIDER option: Options="+str(Options))
                    setOptNames+=opt+","
        elif optu[:5]=="ADDR=":
            addr=int(float(opt[5:]))
            #Request command to change address of modbus device
            if (addr!=None): 
                # Send command to program this device to the new address
                if (addr>=1 and addr<=5): 
                    setNewAddr=addr
        elif optu[:11]=="MAXCURRENT=" and ("EV Mode" in Devices[Unit].Name or "EV State" in Devices[Unit].Name):
            setMaxCurrent=int(float(opt[11:]))
            if (setMaxCurrent<6 or setMaxCurrent>36):
                setMaxCurrent=16   # default value
            setOptNames+=f"MAXCURRENT={setMaxCurrent},"
            Log(LOG_INFO,f"setOptNames={setOptNames}")
        elif optu[:9]=="MAXPOWER=" and ("EV Mode" in Devices[Unit].Name or "EV State" in Devices[Unit].Name):
            setMaxPower=int(float(opt[9:]))
            if (setMaxPower<1000 or setMaxPower>25000):
                setMaxPower=3300   # default value
            setOptNames+=f"MAXPOWER={setMaxPower},"
        elif optu[:11]=="STARTPOWER=" and ("EV Mode" in Devices[Unit].Name or "EV State" in Devices[Unit].Name):
            setStartPower=int(float(opt[11:]))
            if (setStartPower<800 or setStartPower>25000):
                setStartPower=1200   # default value
            setOptNames+=f"STARTPOWER={setStartPower},"
        elif optu[:9]=="STOPTIME=" and ("EV Mode" in Devices[Unit].Name or "EV State" in Devices[Unit].Name):
            setStopTime=int(float(opt[9:]))
            if (setStopTime<5 or setStopTime>600):
                setStopTime=90   # default value
            setOptNames+=f"STOPTIME={setStopTime},"
        elif optu[:10]=="AUTOSTART=" and ("EV Mode" in Devices[Unit].Name or "EV State" in Devices[Unit].Name):
            setAutoStart=int(float(opt[10:]))
            if (setAutoStart>1):
                setAutoStart=1   # default value
            setOptNames+=f"AUTOSTART={setAutoStart},"
        elif optu[:9]=="HWADDR=0X" and len(opt)==13:
            #set hardware address
            hwaddr=int(optu[7:],16)
            if (hwaddr>=1 and hwaddr<65535):
                setHwaddr=hwaddr
        elif optu[:8]=="DISABLE=":
            #set which ports are enabled, for this device, and remove disabled ports to free space in Devices[]
            #syntax: DISABLE=1:2:3:4:5:10:11
            setOptNames+="DISABLE="
            portsDisabled[deviceAddr]=[]
            portsDisabledWrite=3   #write portDisabled on a json file in 3*heartbeat_interval (3*10s)
            setDisableDefined=1
            modulesAskConfig.append(frameAddr)

            for ps in optu[8:].split(':'):
                try:
                    p=int(ps)
                except:
                    #skip this
                    Log(LOG_WARN,"DISABLE= wrong port "+ps)
                else:
                    if (p>1 and p<=32):
                        if p not in portsDisabled[deviceAddr]:
                            portsDisabled[deviceAddr].append(p)
                        #remove the device corresponding to frameAddr and disabled port
                        deviceid="H{:04x}_P{:04x}".format(frameAddr, p)
                        for u in Devices:
                            if Devices[u].DeviceID==deviceid:
                                Log(LOG_DEBUG,"Removing device "+deviceid+"...")
                                Devices[u].Delete()
                                #TODO
                                break
                        setOptNames+=str(p)+":"
            if (setOptNames[-1:]==':'):
                setOptNames=setOptNames[:-1]    #remove trailing ':'
            setOptNames+=','                
        elif (optu[:6]=="DESCR=" or optu[:10]=="TIMECLOSE=" or optu[:9]=="TIMEOPEN="):
            setOptNames+=opt+","
        elif (optu[:5]=="DCMD("):
            #command to another dombus
            errmsg=''  
            d=[ DCMD_IN_EVENTS['NONE'], 0, 0, 0, 0, DCMD_OUT_CMDS['NONE'], 0 ] #temp list to store a DCMD command
            
            opt=re.sub("ERROR=.*", "", opt) #remove any Error=blablabla from the command
            optu=opt.upper()
            inputs=re.search('DCMD\((.+)\)=(.+\..+:.+)', optu)
            if inputs:
                #syntax of DCMD command semms to be ok
                inArr=inputs.group(1).split(':')    #inArr=['Value','0','20.5'] (inputs) 
                outArr=inputs.group(2).split(':')   #
                if (len(inArr)>=1):
                    Log(LOG_INFO,"DCMD: "+opt+" Input event="+str(inArr)+" Output command="+str(outArr))
                    if (inArr[0] in DCMD_IN_EVENTS): 
                        d[0]=DCMD_IN_EVENTS[inArr[0]]
                        d[1]=0
                        d1ok=0
                        d[2]=0
                        d2ok=0
                        if (len(inArr)>=2):
                            # inArr[1] contains a temperature, humidity, voltage,... convert this value to a integer representation used by DomBus
                            try:
                                d[1]=float(inArr[1])
                            except:
                                errmsg+="ValueLow should be a number, like 20.5. "
                                d[1]=0
                            else:
                                d1ok=1

                            try:
                                d[2]=float(inArr[2])
                            except:
                                errmsg+="ValueHigh should be a number, like 21.2. "
                                d[2]=0
                            else:
                                d2ok=1
                            if (inArr[0]=='VALUE'):
                                #convert d[1] and d[2] in temperature, RH, voltage, value according to the sensor type and A and B parameters
                                if (d1ok):
                                    d[1]=convertValueToDombus(Devices[Unit],d[1])
                                if (d2ok):
                                    d[2]=convertValueToDombus(Devices[Unit],d[2])
                                Log(LOG_DEBUG,"d[1]="+str(d[1])+" d[2]="+str(d[2]))
                        if (len(outArr)>=2):
                            if (outArr[1] in DCMD_OUT_CMDS):
                                #outArr[0]=101.4
                                #outArr[1]=ON
                                hwaddrport=outArr[0].split('.')
                                #TODO: ALL.BLIND
                                d[3]=int(hwaddrport[0],16)
                                d[4]=int(hwaddrport[1],16)
                                d[5]=int(DCMD_OUT_CMDS[outArr[1]])
                                d[6]=0    #outValue
                                if (len(outArr)>=3):
                                    #outArr[2]=30m
                                    # From 0 to 60s => 31.25ms resolution      0=0, 1920=60s
                                    # From 1m to 1h with 1s resolution         1921=61s, 3540+1920=5460=1h
                                    # From 1h to 1d with 1m resolution         5461=1h+1m, 1380+5460=6840=24h
                                    # From 1d to forever with 1h resolution    6841=25h
                                    if (outArr[2].isnumeric()):
                                        #value * 31.5ms
                                        d[6]=int(outArr[2])
                                    else:
                                        outValue=(outArr[2][:-1])
                                        outUM=outArr[2][-1:]
                                        #value in seconds
                                        if (outValue.isnumeric()):
                                            outValue=int(outValue)
                                            if (outUM=='S'):    #seconds
                                                if (outValue<=60):
                                                    d[6]=outValue*32
                                                elif (outValue<=3600):
                                                    d[6]=1920+(outValue-60)
                                            elif (outUM=='M'): #minutes
                                                if (outValue<=1):
                                                    d[6]=outValue*60*32
                                                elif (outValue<=60):
                                                    d[6]=1920+(outValue-1)*60
                                                elif (outValue<=1440):
                                                    d[6]=5460+(outValue-60)
                                            elif (outUM=='H'):  #hours
                                                if (outValue<=1):
                                                    d[6]=outValue*5460
                                                elif(outValue<=24):
                                                    d[6]=5460+(outValue-1)*60
                                                else:
                                                    d[6]=6840+(outValue-24)
                                            elif (outUM=='D'):
                                                d[6]=6840+(outValue-1)*24
                                            if (d[6]>65535):
                                                d[6]=1826*24+6840 #max 5 years = 1826 days
                                                errmsg+='Max time = 1826 days;'
                                dcmd.append(d)  #add record to dcmd[]
                            else:
                                errmsg="Command not recognized;"
                                Log(LOG_WARN,"DCMD: Command "+outArr[1]+" not recognized, possible commands="+str(list(DCMD_OUT_CMDS)))
                                Log(LOG_DEBUG,"DCMD: "+opt)
                        else:
                            errmsg="At least HWADDR.PORT:COMMAND expected after =;"
                            Log(LOG_WARN,"DCMD: Address.Port:Command : invalid syntax. Address.Port="+outArr[0]+" ,Command="+outArr[1])
                            Log(LOG_DEBUG,"DCMD: "+opt)
                    else:
                        errmsg="Event not recognized inside ();"
                        Log(LOG_WARN,"DCMD: Event not recognized: event="+inArr[0]+" , possible events="+str(list(DCMD_IN_EVENTS.keys())))
                        Log(LOG_DEBUG,"DCMD: "+opt)
                else:
                    errmsg="At least 1 parameter expected inside ();"
                    Log(LOG_WARN,"DCMD: no parameters specified inside ()")
                    Log(LOG_DEBUG,"DCMD: "+opt)
            else:
                errmsg="Invalid syntax;"
                Log(LOG_WARN,"DCMD: invalid syntax")
                Log(LOG_DEBUG,"DCMD: "+opt)
            if (len(errmsg)>0):
                opt+=':Error='+errmsg+': Valid command is like DCMD(Value:0:20.5)=101.1:ON:30m'
                #reset values inside the current DCMD array
            setOptNames+="\n"+opt+","   #DCMD OK: 
    if (setOptNames!=''):
        setOptNames=setOptNames[:-1]    #remove last comma ,
    Log(LOG_INFO,"Config device "+hex(frameAddr)+": type="+hex(setType)+" typeName="+typeName+" Options="+str(Options))
    if (setHwaddr!=0 and setHwaddr!=0xffff): #hwaddr not 0 
        # remove HWADDR=0X1234 option from the device description
        # send command to change hwaddr
        txQueueAdd(0, frameAddr, CMD_CONFIG, 3, 0, 0, [(setHwaddr >> 8), (setHwaddr&0xff)], TX_RETRY,1)
    elif (setNewAddr!=''): # set modbus address
        # send command to change modbus addr
        Log(LOG_INFO,f"Send command to change modbus device address to {setNewAddr}")
        txQueueAdd(0, frameAddr, CMD_CONFIG, 4, 0, port, [SUBCMD_SET, (setNewAddr>>8), (setNewAddr&0xff)], TX_RETRY, 1)
    else:
        if modules[frameAddr][LASTPROTOCOL]==1:
            txQueueAdd(0, frameAddr, CMD_CONFIG, 5, 0, port, [((setType>>8)&0xff), (setType&0xff), (setOpt >> 8), (setOpt&0xff)], TX_RETRY,0) #PORTTYPE_VERSION=1
        else:
            txQueueAdd(0, frameAddr, CMD_CONFIG, 7, 0, port, [((setType>>24)&0xff), ((setType>>16)&0xff), ((setType>>8)&0xff), (setType&0xff), (setOpt >> 8), (setOpt&0xff)], TX_RETRY,0) #PORTTYPE_VERSION=2
    if modules[frameAddr][LASTPROTOCOL] != 1:
        #Transmit Dombus CMD config
        dcmdnum=0
        for i in range(0,min(len(dcmd),8)):
            d=dcmd[i]
            #note: port|=0, 0x20, 0x40, 0x60 (4 DCMD for each port)
            if (d[0]!=0 and d[0]<DCMD_IN_EVENTS["MAX"]):
                dcmdnum+=1
                txQueueAdd(0, frameAddr,CMD_DCMD_CONFIG, 12, 0, port|(i<<5), [ d[0], 
                    d[1]>>8, d[1]&0xff, 
                    d[2]>>8, d[2]&0xff, 
                    d[3]>>8, d[3]&0xff, d[4], d[5], 
                    d[6]>>8, d[6]&0xff ], TX_RETRY, 0) 
        if (dcmdnum==0): #DCMD not defined => transmits an empty DCMD_CONFIG 
            txQueueAdd(0, frameAddr, CMD_DCMD_CONFIG, 2, 0, port, [ DCMD_IN_EVENTS["NONE"] ], TX_RETRY, 0)
    else:
        #protocol==1 does not support DCMD_CONFIG command (too long)
        Log(LOG_WARN,"Device "+devID+" does not support protocol #2 and DCMD commands")
    
    descr='ID='+devID+','+setTypeName+','+setOptNames if (setTypeDefined==1) else setOptNames
    if setTypeDefined != 0: #type was defined in the description => change TypeName, if different
        if (setTypeName=="IN_TWINBUTTON" or setTypeName=="OUT_BLIND"): #selector
            if "LevelNames" not in Options:
                Options["LevelNames"]="Off|Down|Up"
                #Options["LevelOffHidden"]="false"
                #Options["SelectorStyle"]="0"
        if (len(Options)>0):
            Log(LOG_INFO,"TypeName="+str(typeName)+", nValue="+str(Devices[Unit].nValue)+", sValue="+str(Devices[Unit].sValue)+", Description="+str(descr)+", Options="+str(Options))
            Devices[Unit].Update(TypeName=typeName, nValue=Devices[Unit].nValue, sValue=Devices[Unit].sValue, Description=str(descr), Options=Options)  # Update description (removing HWADDR=0x1234)
        else:
            Log(LOG_INFO,"TypeName="+str(typeName)+", nValue="+str(Devices[Unit].nValue)+", sValue="+str(Devices[Unit].sValue)+", Description="+str(descr))
            Devices[Unit].Update(TypeName=typeName, nValue=Devices[Unit].nValue, sValue=Devices[Unit].sValue, Description=str(descr))  # Update description (removing HWADDR=0x1234)
    else: #type not defined in description => don't change it!
        Devices[Unit].Update(nValue=Devices[Unit].nValue, sValue=Devices[Unit].sValue, Description=str(descr))  # Update description (removing HWADDR=0x1234)
    if (setCal!=32768): #new calibration value
        if (setCal<0): 
            setCal+=65536
        txQueueAdd(0, frameAddr, CMD_CONFIG, 4, 0, port, [SUBCMD_CALIBRATE, ((setCal>>8)&0xff), (setCal&0xff)], TX_RETRY, 0)
    if (setMaxCurrent!=0): # EV Mode: set max current (cable) 
        txQueueAdd(0, frameAddr, CMD_CONFIG, 4, 0, port, [SUBCMD_SET, 0, setMaxCurrent], TX_RETRY,0) 
    if (setMaxPower!=0): # EV Mode: set max power from grid 
        txQueueAdd(0, frameAddr, CMD_CONFIG, 4, 0, port, [SUBCMD_SET2, setMaxPower>>8, setMaxPower&0xff], TX_RETRY,0) 
    if (setStartPower!=0):
        txQueueAdd(0, frameAddr, CMD_CONFIG, 4, 0, port, [SUBCMD_SET3, ((setStartPower>>8)&0xff), (setStartPower&0xff)], TX_RETRY,0) 
    if (setStopTime!=0):
        txQueueAdd(0, frameAddr, CMD_CONFIG, 4, 0, port, [SUBCMD_SET4, ((setStopTime>>8)&0xff), (setStopTime&0xff)], TX_RETRY,0) 
    if (setAutoStart!=0):
        txQueueAdd(0, frameAddr, CMD_CONFIG, 4, 0, port, [SUBCMD_SET5, ((setAutoStart>>8)&0xff), (setAutoStart&0xff)], TX_RETRY,0) 
        
    return

def decode(Devices):
    # align rxbuffer[] so it starts with a preamble
    global modules, txQueue, rxbuffer, UnitFree, deviceID, protocol
    while (len(rxbuffer)>=FRAME_LEN_MIN and (rxbuffer[0]!=PREAMBLE_DEVICE and rxbuffer[0]!=PREAMBLE)):
        rxbuffer.pop(0)
    if (len(rxbuffer)<FRAME_LEN_MIN): 
        return
    frameError=1
    #decode frame RXed from serial port
    #frame structure was explained above, in the comments
    if (rxbuffer[0]==PREAMBLE_DEVICE):
        #protocol 1.0 (short version, without sender address)
        protocol=1
        frameLen=int(rxbuffer[FRAME_LEN])+FRAME_HEADER+1
        if frameLen < FRAME_LEN_MIN:
            frameError=4    # Error, frame must be longer!
        elif len(rxbuffer) >= frameLen:
            #length of frame is in the range
            # compute and compare checksum
            checksum(protocol, rxbuffer)
            if (logLevel>=LOG_DUMPALL):
                dump(protocol, rxbuffer,frameLen,"RX")
            if (checksumValue == rxbuffer[frameLen-1]):
                #frame checksum is ok
                frameAddr=int(rxbuffer[1])*256+int(rxbuffer[2])
                frameIdx=FRAME_HEADER
                dstAddr=0;  #not specified in this protocol => force dstAddr=0
                if frameAddr!=0xffff and frameAddr not in modules: 
                    # first time receive data from this module: ask for configuration?
                    modules[frameAddr]=[int(time.time()),0,0,protocol,0]   #transmit now the output status
                    #ask for port configuration?
                    if rxbuffer[FRAME_HEADER]!=CMD_CONFIG:
                        txQueueAskConfig(protocol,frameAddr) #ask for configuration only if the device is not already sending configuration
                #broadcast or txQueue exists or txQueue just created
                frameError=0
            else:
                frameError=2    #2=checksum error
        else:
            frameError=3        #3=insufficient data

    elif (rxbuffer[0]==PREAMBLE):
        #protocol 2
        protocol=2
        frameLen=int(rxbuffer[FRAME_LEN2])+FRAME_HEADER2+1
        if frameLen<FRAME_LEN_MIN2:
            frameError=4    # Error, frame must be longer!
        elif len(rxbuffer)>=frameLen:
            #length of frame is in the range
            # compute and compare checksum
            checksum(protocol, rxbuffer)
            if logLevel>=LOG_DUMP:
                dump(protocol, rxbuffer,frameLen,"RX")
            if (checksumValue == rxbuffer[frameLen-1]):
                #frame checksum is ok
                frameAddr=int(rxbuffer[3])*256+int(rxbuffer[4]) #sender
                dstAddr=int(rxbuffer[1])*256+int(rxbuffer[2])   #destination
                frameIdx=FRAME_HEADER2
                if (frameAddr!=0xffff and frameAddr not in modules): 
                    # first time receive data from this module: ask for configuration?
                    modules[frameAddr]=[int(time.time()),0,0,protocol,0]   #transmit now the output status
                    #ask for port configuration?
                    if rxbuffer[FRAME_HEADER2]!=CMD_CONFIG:
                        txQueueAskConfig(protocol,frameAddr) #ask for configuration only if the device is not already sending configuration
                #broadcast or txQueue exists or txQueue just created
                frameError=0               
            else:
                frameError=2    #2=checksum error
        else:
            frameError=3        #3=insufficient data
                
    if (frameError==0): #parse frame
        while (frameIdx<frameLen-2):
            cmd=rxbuffer[frameIdx]
            cmdAck=cmd&CMD_ACK
            portIdx=frameIdx+1
            cmdLen=cmd&CMD_LEN_MASK
            if (protocol!=1):
                cmdLen*=2
            cmd&=CMD_MASK
            port=rxbuffer[portIdx]
            arg1=rxbuffer[portIdx+1] if (cmdLen>=2) else 0 #port 
            arg2=rxbuffer[portIdx+2] if (cmdLen>=3) else 0
            arg3=rxbuffer[portIdx+3] if (cmdLen>=4) else 0
            arg4=rxbuffer[portIdx+4] if (cmdLen>=5) else 0
            arg5=rxbuffer[portIdx+5] if (cmdLen>=6) else 0
            arg6=rxbuffer[portIdx+6] if (cmdLen>=7) else 0
            arg7=rxbuffer[portIdx+7] if (cmdLen>=8) else 0
            arg8=rxbuffer[portIdx+8] if (cmdLen>=9) else 0
            arg9=rxbuffer[portIdx+9] if (cmdLen>=10) else 0
            arg10=rxbuffer[portIdx+10] if (cmdLen>=11) else 0
            arg11=rxbuffer[portIdx+11] if (cmdLen>=12) else 0
            getDeviceID(frameAddr, port)

            modules[frameAddr][LASTPROTOCOL]=protocol
            modules[frameAddr][LASTRX]=int(time.time())
            if (cmdAck):
                # received an ack: remove cmd+arg1 from txQueue, if present
                if (frameAddr!=0xffff):
                    #Received ACK from a slave module
                    txQueueRemove(frameAddr,cmd, port, arg1)
                    modules[frameAddr][LASTRETRY]=0
                if (cmd==CMD_CONFIG and dstAddr==0):
                    if (port==0xfe):  # Version
                        if (cmdLen>=8):
                            strVersion=rxbuffer[portIdx+1:portIdx+5].decode()
                            strModule=rxbuffer[portIdx+5:portIdx+cmdLen-1].decode()
                            Log(LOG_INFO,"Module "+str(strModule)+" Rev."+str(strVersion)+" Addr="+hex(frameAddr))
                            if (frameAddr in modules):
                                modules[frameAddr][LASTSTATUS]=0    #force transmit output status
                    elif ((port&0xf0)==0xf0):   #0xff or 0xf0, 0xf1, 0xf2, ...0xfd
                        #0xff VERSION PORTTYPE PORTOPT PORTCAPABILITIES PORTIMAGE PORTNAME
                        #arg1 contains the PORTTYPE_VERSION (to extend functionality in the future)
                        portVer=arg1  #protocol version used to exchange information
                        frameIdx=portIdx+2
                        if (port==0xff):
                            port=1        #port starts with 1
                        else:
                            port=arg2   # arg2 set the starting port number (needed to configure dombus devices with several ports)
                            frameIdx+=1 # start from arg3
                        while (frameIdx < frameLen-1): #scan all ports defined in the frame
                            if (portVer==1):
                                portType=int(rxbuffer[frameIdx])*256+int(rxbuffer[frameIdx+1])
                                frameIdx+=2
                                portOpt=int(rxbuffer[frameIdx])*256+int(rxbuffer[frameIdx+1])
                                frameIdx+=2
                                portCapabilities=int(rxbuffer[frameIdx])*256+int(rxbuffer[frameIdx+1]) #not needed, ignored
                                frameIdx+=2
                                portImage=int(rxbuffer[frameIdx]) #not used, ignored
                                frameIdx+=1
                            else:
                                portType=(int(rxbuffer[frameIdx])<<24)+(int(rxbuffer[frameIdx+1])<<16)+(int(rxbuffer[frameIdx+2])<<8)+int(rxbuffer[frameIdx+3])
                                frameIdx+=4
                                portOpt=int(rxbuffer[frameIdx])*256+int(rxbuffer[frameIdx+1])
                                frameIdx+=2
                            portName=""
                            getDeviceID(frameAddr, port)
                            for i in range(0,16): #get the name associated to the current port
                                ch=rxbuffer[frameIdx]
                                frameIdx+=1
                                if (ch==0):
                                    break
                                else:
                                    portName+=chr(ch)
                            #check if this port device already exists?
                            UnitFree=1  # first free unit in Devices[]
                            found=0
                            unit=getDeviceUnit(Devices,1) #1 means that it must scan all devices to find the right value of UnitFree
                            #Log(LOG_DEBUG,"DeviceID="+deviceID+" frameAddr="+hex(frameAddr)+" portType="+hex(portType)+" unit="+hex(unit)+" UnitFree="+str(UnitFree))
                            #check if frameAddr is in portsDisabled, and if the current port is disabled
                            if ((deviceAddr not in portsDisabled or port not in portsDisabled[deviceAddr])):
                                nValue=0
                                sValue=''
                                Used=0
                                if (unit!=0xffff):
                                    #unit found => remove TimedOut if set
                                    if port==1:
                                        Log(LOG_INFO,"Device "+devID+" is now active again")
                                    Devices[unit].Update(nValue=Devices[unit].nValue, sValue=Devices[unit].sValue, TimedOut=0)
                                else:
                                    #port device not found, and is not disabled: create it!
                                    #portType is the numeric number provided by DOMBUS
                                    if (portType not in PORT_TYPENAME): portType=PORTTYPE_IN_DIGITAL   #default: Digital Input
                                    if (UnitFree>255 or UnitFree<1):
                                        Log(LOG_ERR,"Maximum number of devices is reached! Domoticz supports max 255 devices for each hardware.\nUse more serial ports to separate modules in more buses, or disable unused ports in this way:\nselect port 1 of a module, and write in the description DISABLE=3:4:5:11 to disable, for example, ports 3,4,5 and 11 of that module")
                                        devicesFull=True
                                    else:
                                        devicesFull=False
                                        descr='ID='+devID+','
                                        for key, value in PORTTYPES.items():
                                            if (value==portType):
                                                descr+=key+","
                                                break
                                        for key,value in PORTOPTS.items():
                                            if (value==portOpt):
                                                #descr=descr+key+","
                                                descr+=key+","
                                        if (descr!=''):
                                            descr=descr[:-1]    #remove last comma ,

                                        if (portType!=PORTTYPE_CUSTOM or portOpt>=2):
                                            # do not enable CUSTOM device with PORTOPT not specified (ignore it!)
                                            typeName=PORT_TYPENAME[portType]
                                            Options={}
                                            Switchtype=''
                                            if (portType==PORTTYPE_CUSTOM):
                                                if (portOpt==PORTOPT_SELECTOR):
                                                    typeName="Selector Switch"
                                                elif (portOpt==PORTOPT_DIMMER):
                                                    typeName="Dimmer"
                                                elif (portOpt==PORTOPT_ADDRESS):
                                                    typeName="Text"
                                                elif (portOpt==PORTOPT_IMPORT_ENERGY or portOpt==PORTOPT_EXPORT_ENERGY):
                                                    typeName="kWh"
                                                    sValue="0;0"    #power,energy
                                                    if ("Solar" in portName or "Exp" in portName):
                                                        Switchtype=4   # Export energy
                                                    if "EV Solar" in portName or "EV Grid" in portName:
                                                        Options["EnergyMeterMode"]="1"
                                                elif (portOpt==PORTOPT_VOLTAGE):
                                                    typeName="Voltage"
                                                elif (portOpt==PORTOPT_POWER_FACTOR):
                                                    typeName="Percentage"
                                                    descr+=",A=0.1"
                                                elif (portOpt==PORTOPT_FREQUENCY):
                                                    typeName="Custom"
                                                    descr+=",A=0.01"
                                                    Options['Custom']="1;Hz"
                                                descr+=",TypeName="+typeName
                                                Log(LOG_INFO,f"portName={portName}")
                                                if ("EV State" in portName):
                                                    # create two sliders, one for min and one for max SoC
                                                    Domoticz.Device(Name="EV BatteryMin", TypeName="Dimmer", Unit=UnitFree, Description="Battery level under which the EV is charged using energy from the grid. This virtual device is used by script_event_power.lua (@CreasolTech on github)").Create()
                                                    Devices[UnitFree].Update(nValue=1, sValue="50", Used=1, Name="EV BatteryMin")
                                                    unit=getDeviceUnit(Devices,1) # find another free Unit to create EVSE BatteryMax virtual device
                                                    if (UnitFree!=0xffff):
                                                        Domoticz.Device(Name="EV BatteryMax", TypeName="Dimmer", Unit=UnitFree, Description="When battery level is between min and max, only energy from reneable will be used. This virtual device is used by script_event_power.lua (@CreasolTech on github)").Create()
                                                        Devices[UnitFree].Update(nValue=1, sValue="80", Used=1, Name="EV BatteryMax")
                                                    unit=getDeviceUnit(Devices,1)   # find another free Unit to create EVSE CurrentMax virtual device
                                                    if (UnitFree!=0xffff):
                                                        Options["LevelNames"]="0|8|12|16|20|24|28|32|36"
                                                        Log(LOG_INFO,f"MaxCurrent Options={Options}")
                                                        Domoticz.Device(Name="EV CurrentMax", TypeName="Selector Switch", Unit=UnitFree, Description="Maximum charging current. This virtual device is used by script_event_power.lua (@CreasolTech on github)", Options=Options).Create()
                                                        Devices[UnitFree].Update(nValue=1, sValue="80", Used=1)
                                                    unit=getDeviceUnit(Devices,1)
                                                    #ToDo: if UnitFree>255 => no space in Devices[] table for a new device
                                                    Options={"LevelNames": "0|Dis|Con|Ch|Vent|AEV|APO|AW", "LevelActions": "", "LevelOffHidden": "True", "SelectorStyle": "0"}
                                                    typeName="Selector Switch"
                                                    sValue="0"
                                                elif ("EV Mode" in portName):   #Off, Solar, 50%, 75%, 100%, Managed
                                                    Options={"LevelNames": "Off|Solar|25%|50%|75%|100%|Man", "LevelActions": "", "LevelOffHidden": "False", "SelectorStyle": "0"}
                                                    typeName="Selector Switch"
                                                    setMaxCurrent=16
                                                    setMaxPower=3300
                                                    setStartPower=1200
                                                    setStopTime=90
                                                    setAutoStart=1
                                                    descr+=f",MAXCURRENT={setMaxCurrent},MAXPOWER={setMaxPower},STARTPOWER={setStartPower},STOPTIME={setStopTime},AUTOSTART={setAutoStart}"
                                                    nValue=0
                                                    sValue="0"
                                                    # Configure 
                                                    txQueueAdd(0, frameAddr, CMD_CONFIG, 4, 0, port, [SUBCMD_SET, 0, (setMaxCurrent&0xff)], TX_RETRY,0) 
                                                    txQueueAdd(0, frameAddr, CMD_CONFIG, 4, 0, port, [SUBCMD_SET2, ((setMaxPower>>8)&0xff), (setMaxPower&0xff)], TX_RETRY,0) 
                                                    txQueueAdd(0, frameAddr, CMD_CONFIG, 4, 0, port, [SUBCMD_SET3, ((setStartPower>>8)&0xff), (setStartPower&0xff)], TX_RETRY,0) 
                                                    txQueueAdd(0, frameAddr, CMD_CONFIG, 4, 0, port, [SUBCMD_SET4, ((setStopTime>>8)&0xff), (setStopTime&0xff)], TX_RETRY,0) 
                                                    txQueueAdd(0, frameAddr, CMD_CONFIG, 4, 0, port, [SUBCMD_SET5, ((setAutoStart>>8)&0xff), (setAutoStart&0xff)], TX_RETRY,0) 
                                            elif (portOpt==PORTOPT_DIMMER):
                                                typeName="Dimmer"
                                                Log(LOG_INFO,f"DIMMER, portName={portName}")
                                                #if "EVSE Current" in portName:
                                                    # Options={"MaxDimLevel": "32"}    #Max 32A : not implemented in python plugin
                                                    # Log(LOG_INFO,f"portName contains EVSE Current, Options={Options}")
                                            elif (portType==PORTTYPE_IN_COUNTER):
                                                # counter or kWh ?
                                                if "POWER" in portName.upper():
                                                    typeName="kWh"
                                                    sValue="0;0"

                                        Log(LOG_INFO,"Add device "+deviceID+" deviceAddr="+deviceAddr+": "+portName+" portType="+hex(portType)+" portOpt="+str(portOpt)+" Description="+descr)
                                        Log(LOG_INFO,"Name=("+devID+") "+portName+", TypeName="+typeName+", DeviceID="+deviceID+", Unit="+str(UnitFree)+" ,Options="+str(Options))
                                        if (Switchtype!=''): 
                                            Domoticz.Device(Name="("+devID+") "+portName, TypeName=typeName, Switchtype=Switchtype, DeviceID=deviceID, Unit=UnitFree, Options=Options, Description=descr).Create()
                                        else:
                                            Domoticz.Device(Name="("+devID+") "+portName, TypeName=typeName, DeviceID=deviceID, Unit=UnitFree, Options=Options, Description=descr).Create()
                                        if frameAddr<=0xff00 or port==1:
                                            Used=1
                                        Devices[UnitFree].Update(nValue=nValue, sValue=sValue, Used=Used)   # do not enable devices with default address
                            port+=1;

                #TODO elif (cmd==CMD_DCMD): #decode DCMD frame to get the STATUS word?
            else:
                #cmdAck==0 => decode command from slave module
                if (frameAddr!=0xffff and dstAddr==0):
                    #Receive command from a slave module
                    getDeviceID(frameAddr,port)
                    if (cmd==CMD_GET): 
                        if (port==0): #port==0 => request from module to get status of all output!  NOT USED by any module, actually
                            txQueueAdd(protocol, frameAddr,cmd,2,CMD_ACK,port,[arg1],1,1)   #tx ack
                            if (frameAddr in modules):
                                modules[frameAddr][LASTSTATUS]=0    #force transmit output status
                        else: # port specified: return status for that port
                            unit=getDeviceUnit(Devices,0)
                            if (unit!=0xffff):
                                d=Devices[unit]
                                if (d.Type==244):
                                    if (hasattr(d,'SwitchType') and d.SwitchType==18): #selector
                                        if (any(i.isdigit() for i in d.sValue)==False):
                                            arg1=0
                                            d.Update(nValue=0, sValue="0")
                                        else:
                                            arg1=int(d.sValue)
                                    elif (hasattr(d,'SwitchType') and d.SwitchType==7): #dimmer
                                        arg1=int(d.sValue)
                                    else:   
                                        arg1=d.nValue

                                    txQueueAdd(protocol, frameAddr,cmd,2,CMD_ACK,port,[arg1],1,1)

                    elif (cmd==CMD_SET):
                        #digital or analog input changed?
                        UnitFree=1
                        unit=getDeviceUnit(Devices,0)
                        if (unit==0xffff):
                            if ((deviceAddr not in portsDisabled or port not in portsDisabled[deviceAddr])):
                                #got a frame from a unknown device, that is not disabled => ask for configuration
                                #Log(LOG_DEBUG,"Device="+devID+" portsDisabled["+str(deviceAddr)+"]="+portsDisabled[deviceAddr]+" => Ask config")
                                txQueueAskConfig(protocol,frameAddr)
                            else:
                                # ports is disabled => send ACK anyway, to prevent useless retries
                                #Log(LOG_DEBUG,"Send ACK even if port "+str(port)+" is disabled")
                                txQueueAdd(protocol, frameAddr,cmd,2,CMD_ACK,port,[arg1],1,1)

                        else:
                            d=Devices[unit]
                            #got a frame from a well known device
                            getDeviceID(frameAddr,port)
                            if (cmdLen==2):
                                stringval='Off' if arg1==0 else 'On'
                                #update device only if was changed
                                #Log(LOG_DEBUG,"devID="+devID+" d.Type="+str(d.Type)+" d.SwitchType="+str(d.SwitchType)+" nValue="+str(arg1)+" sValue="+str(arg1))
                                if (d.Type==PORTTYPE[PORTTYPE_OUT_DIGITAL]):
                                    if (hasattr(d,'SwitchType') and d.SwitchType==18): #selector
                                        d.Update(nValue=int(arg1), sValue=str(arg1))
                                    elif (hasattr(d,'SwitchType') and d.SwitchType==7): #dimmer
                                        nValue=0 if arg1==0 else 1
                                        d.Update(nValue=nValue, sValue=str(arg1))
                                    else: #normal switch
                                        if (d.nValue!=int(arg1) or d.sValue!=stringval):
                                            d.Update(nValue=int(arg1), sValue=stringval)
                                elif (d.Type==243):
                                    if (d.SubType==28): #Incremental counter
                                        if (arg1>0):
                                            d.Update(nValue=0, sValue=str(arg1));
                                    elif (d.SubType==29): #kWh
                                        if (arg1==0):
                                            # just update the current value, to avoid bad charts
                                            d.Update(nValue=d.nValue, sValue=d.svalue)
                                        else:
                                            sv=d.sValue.split(';')  #sv[0]=POWER, sv[1]=COUNTER
                                            divider=1000    # default divider value
                                            if ("divider" in d.Options):
                                                divider=int(float(d.Options['divider']))
                                                if (divider==0): 
                                                    divider=1000
                                            if (len(sv)!=2): 
                                                sv[0]="0" #power
                                                sv.append("0") #counter
                                            p=int(float(sv[0]))
                                            energy=float(sv[1])+arg1*1000/divider  #total energy in Wh
                                            ms=int(time.time()*1000)
                                            # check that counterTime[d.Unit] exists: used to set the last time a pulse was received. 
                                            # Althought it's possible to save data into d.Options, it's much better to have a dict so it's possible to periodically check all counterTime
                                            if (d.Unit not in counterTime):
                                                counterTime[d.Unit]=0
                                            msdiff=ms-counterTime[d.Unit] #elapsed time since last value
                                            if (msdiff>=4000): #check that frames do not come too fast (Domoticz was busy with database backup??). Dombus tx period >= 5000ms
                                                power=int(arg1*3600000000/(msdiff*divider))
                                            else:
                                                #frame received close to the previous one => ignore power computation
                                                power=p    
                                            counterTime[d.Unit]=ms
                                            svalue=str(power)+';'+str(energy)
                                            Log(LOG_INFO,"kWh meter: count="+str(arg1)+" sValue="+svalue+" Name="+d.Name)
                                            d.Update(nValue=0, sValue=svalue)
                                            if ('opposite' in d.Options):
                                                #opposite = Unit of the opposite kWh counter, so if that counter exists, set it to 0 power
                                                opposite=int(d.Options['opposite'])
                                                if (opposite in Devices and Devices[opposite].Type==243 and Devices[opposite].SubType==29):
                                                    #opposite device exists and it's a kWh counter
                                                    sv=Devices[opposite].sValue.split(';')
                                                    p=int(float(sv[0]))
                                                    energy=float(sv[1])
                                                    if (p!=0):
                                                        Devices[opposite].Update(nValue=0, sValue="0;"+str(energy))
                                else:
                                    #device is not a switch, or has not the SwitchType attribute
                                    Log(LOG_DEBUG,"Ignore SET command because Type="+str(d.Type)+" SubType="+str(d.SubType)+" Name="+d.Name+" has not attribute SwitchType")
    
                                #send ack
                                txQueueAdd(protocol, frameAddr,cmd,2,CMD_ACK,port,[arg1],1,1)
                            elif (cmdLen==3 or cmdLen==4):
                                #analog value, distance, temperature or humidity
                                value=arg1*256+arg2
                                if (d.Type==PORTTYPE[PORTTYPE_SENSOR_TEMP]):
                                    if 'function' in d.Options:
                                        Ro=10000
                                        To=25.0
                                        temp=0  #default
                                        stringval="0"
                                        if (d.Options['function']=='3950'):
                                            #value=0..65535
                                            beta=3950
                                            if (value==65535): value=65534  #Avoid division by zero
                                            r=value*Ro/(65535-value)
                                            temp=math.log(r / Ro) / beta      # log(R/Ro) / beta
                                            temp+=1.0/(To + 273.15)
                                            temp=round((1.0/temp)-273.15, 1)
                                    else:
                                        temp=round(value/10.0-273.1,1)
                                        #Log(LOG_DEBUG,"Temperature: value="+str(value)+" temp="+str(temp)) 
                                    #Now manage A and B
                                    v=getOpt(d,"B=")
                                    b=float(v) if (v!="false") else 0
                                    temp=temp+b
                                    if (temp>-50 and d.sValue!=stringval):
                                        d.Update(nValue=int(temp), sValue=str(temp))
                                elif (d.Type==PORTTYPE[PORTTYPE_SENSOR_HUM]):
                                    hum=int(value/10)
                                    if (hum>5 and d.nValue!=hum):
                                        d.Update(nValue=hum, sValue=HRstatus(hum))
                                elif (d.Type==243): #distance, voltage, frequency, power factor, ...
                                    #extract A and B, if defined, to compute the right value VALUE=A*dombus_value+B
                                    v=getOpt(d,"A=")
                                    a=float(v) if (v!="false") else 1
                                    v=getOpt(d,"B=")
                                    b=float(v) if (v!="false") else 0
                                    Value=a*value+b
                                    if (d.sValue!=str(Value)):
                                        d.Update(nValue=int(Value), sValue=str(Value))
                                    #Log(LOG_DEBUG,"Value="+str(a)+"*"+str(value)+"+"+str(b)+"="+str(Value))
                                #txQueueAdd(protocol, frameAddr,CMD_SET,3,CMD_ACK,port,[arg1,arg2,arg3],1,1)
                                txQueueAdd(protocol, frameAddr,cmd,2,CMD_ACK,port,[arg1],1,1) #limit the number of data in ACK to cmd|ACK + port
                            elif (cmdLen==5 or cmdLen==6):
                                value=arg1*256+arg2
                                value2=arg3*256+arg4
                                #temp+hum?
                                if (d.Type==PORTTYPE[PORTTYPE_SENSOR_TEMP_HUM]):
                                    temp=round(value/10.0-273.1,1)
                                    hum=int(value2/10)
                                    stringval=str(float(temp))+";"+str(hum)+";"+HRstatus(hum) 
                                    #Log(LOG_DEBUG,"TEMP_HUM: nValue="+str(temp)+" sValue="+stringval)
                                    if (temp>-50 and hum>5 and d.sValue!=stringval):
                                        d.Update(nValue=int(temp), sValue=stringval)
                                    #txQueueAdd(protocol, frameAddr,CMD_SET,5,CMD_ACK,port,[arg1,arg2,arg3,arg4,0],1,1)
                                    txQueueAdd(protocol, frameAddr,cmd,1,CMD_ACK,port,[arg1],1,1) #limit the number of data in ACK to cmd|ACK + port
                                elif (d.Type==243):
                                    power=0 #default: set power=0
                                    #counter: value=newcountvalue, value2=oldcountvalue, arg5=0
                                    counter=value-value2
                                    if (counter<0): #16bit overflow
                                        counter+=65536;
                                    if d.Unit not in counterOld:
                                        counterOld[d.Unit]=0
                                    value_minus_old=value-counterOld[d.Unit]
                                    if (value_minus_old<0):     #16bit overflow
                                        value_minus_old+=65536
                                    old_minus_value2=counterOld[d.Unit]-value2
                                    if (old_minus_value2<0):     #16bit overflow
                                        old_minus_value2+=65536

                                    if d.Unit in counterOld and counterOld[d.Unit]!=0 and old_minus_value2<1000 and value2!=0:
                                        counter=value_minus_old # value - counterOld[d.Unit] +65536 if overflow
                                        power=counter
                                        # Log(LOG_DEBUG,"Counter: counter in sync =>  Name="+d.Name+" counter="+str(counter)+" value="+str(value)+", value2="+str(value2)+", counterOld="+str(counterOld[d.Unit])+", power="+str(power))
                                    elif counter != 0:  # show warning only if this counter is running. If it's not connected to anything, ignore it
                                        Log(LOG_WARN,"Counter: counterOld NOT in sync! Name="+d.Name+" counter="+str(counter)+" value="+str(value)+", value2="+str(value2)+", counterOld="+str(counterOld[d.Unit])+", power="+str(power))
                                    counterOld[d.Unit]=value
                                    if (d.SubType==28): #Incremental counter
                                        if (counter>0):
                                            d.Update(nValue=0, sValue=str(counter));
                                    elif (d.SubType==29): #kWh
###                                        if (counter>0):
                                        sv=d.sValue.split(';')  #sv[0]=POWER, sv[1]=COUNTER
                                        divider=1000    # default divider value
                                        if ("divider" in d.Options):
                                            divider=int(float(d.Options['divider']))
                                            if (divider==0): 
                                                divider=1000
                                        if (len(sv)!=2): 
                                            Log(LOG_INFO,"Counter kWh: sValue has not 2 items: Name="+d.Name+" sValue="+str(d.sValue)+" len(sv)="+str(len(sv)))
                                            sv[0]="0" #power
                                            sv.append("0") #counter
                                        p=int(float(sv[0]))
                                        energy=float(sv[1])+counter*1000/divider  #total energy in Wh
                                        ms=int(time.time()*1000)
                                        # check that counterTime[d.Unit] exists: used to set the last time a pulse was received. 
                                        # Althought it's possible to save data into d.Options, it's much better to have a dict so it's possible to periodically check all counterTime
                                        if (d.Unit not in counterTime):
                                            counterTime[d.Unit]=0
                                        msdiff=ms-counterTime[d.Unit] #elapsed time since last value
                                        if (msdiff>=4000): #check that frames do not come too fast (Domoticz was busy with database backup??). Dombus tx period >= 5000ms
                                            power=int(power*3600000000/(msdiff*divider))
                                        else:
                                            #frame received close to the previous one => ignore power computation
                                            power=p
                                        if (energy-float(sv[1])) > 16:
                                            Log(LOG_WARN,"Counter kWh: > 16Wh increment!!!  Name="+d.Name+" sValueOld="+d.sValue+" sv[0]="+str(sv[0])+" sv[1]="+str(sv[1])+" counter="+str(counter)+" divider="+str(divider)+" energyNow="+str(energy)+" msdiff="+str(msdiff))
                                        counterTime[d.Unit]=ms
                                        svalue=str(power)+';'+str(energy)
                                        # Log(LOG_DEBUG,"Counter kWh:  Name="+d.Name+" count="+str(counter)+" sValue="+svalue+" Name="+d.Name)
                                        d.Update(nValue=0, sValue=svalue)
                                        if ('opposite' in d.Options):
                                            #opposite = Unit of the opposite kWh counter, so if that counter exists, set it to 0 power
                                            opposite=int(d.Options['opposite'])
                                            if (opposite in Devices and Devices[opposite].Type==243 and Devices[opposite].SubType==29):
                                                #opposite device exists and it's a kWh counter
                                                sv=Devices[opposite].sValue.split(';')
                                                p=int(float(sv[0]))
                                                energy=float(sv[1])
                                                if (p!=0):
                                                    Devices[opposite].Update(nValue=0, sValue="0;"+str(energy))
                                #txQueueAdd(protocol, frameAddr,CMD_SET,5,CMD_ACK,port,[arg1,arg2,arg3,arg4,0],1,1)
                                txQueueAdd(protocol, frameAddr,cmd,1,CMD_ACK,port,[arg1],1,1) #limit the number of data in ACK to cmd|ACK + port
                            elif (cmdLen==7 or cmdLen==8):
                                # transmitted 32bit value + 16bit value
                                value=(arg1<<24) + (arg2<<16) + (arg3<<8) + arg4
                                value2=(arg5<<8) + arg6
                                #kWh?
                                if (d.Type==243 and d.SubType==29): #kWh
                                    #value=N*10Wh
                                    #value2=Watt, signed
                                    if (value2&0x8000):
                                        power=value2-65536
                                    else:
                                        power=value2
                                    energy=value*10
                                    stringval=str(power)+";"+str(energy) 
                                    if (d.sValue!=stringval):
                                        d.Update(nValue=power, sValue=stringval)
                                    #txQueueAdd(protocol, frameAddr,CMD_SET,7,CMD_ACK,port,[arg1,arg2,arg3,arg4,arg5,arg6,0],1,1)
                                    txQueueAdd(protocol, frameAddr,cmd,1,CMD_ACK,port,[arg1],1,1) #limit the number of data in ACK to cmd|ACK + port
                                    Log(LOG_DEBUG,f"kWh: value={value}, value2={value2}, power={power}, energy={energy}, stringval={stringval}, name={d.Name}")
            frameIdx=frameIdx+cmdLen+1
        #remove current frame from buffer
        for i in range(0,frameLen):
            rxbuffer.pop(0)
    else:
        if (frameError==2):
            # checksum error
            Log(LOG_WARN,"Checksum error")
            rxbuffer.pop(0)
        elif (frameError==3): 
            #frame len error: remove only the first byte corresponding to a false PREAMBLE
            Log(LOG_DEBUG,"Frame len error: skip this false PREAMBLE")
            if len(rxbuffer)<frameLen:
                Log(LOG_DEBUG,"decode: rxbuffer="+str(len(rxbuffer))+" < frameLen="+str(frameLen))
        elif frameError == 4:
            # Invalid frame: length field too small
            rxbuffer.pop(0)
        dump(1, rxbuffer,frameLen,"RXbad")
        rxbuffer.pop(0)
        Log(LOG_WARN,"Frame length error:"+str(frameLen)+" while len(rxbuffer)="+str(len(rxbuffer)))
    return
def send(Devices, SerialConn):
    #create frames from txQueue[], 1 for each address, and start transmitting
    global modules, txQueue, ms
    # txQueue[frameAddr]=[[cmd, cmdLen, cmdAck, port, [arg1, arg2, arg3, ...], retries]]
    tx=0
    sec=int(time.time())
    ms=int(time.time()*1000)
    # scan all modules
    delmodules=[]
    #TODO: set protocol !!!!
    for frameAddr,module in modules.items():
        timeFromLastTx=ms-module[LASTTX]        #number of milliseconds since last TXed frame
        timeFromLastRx=sec-module[LASTRX]       #number of seconds since last RXed frame
        timeFromLastStatus=sec-module[LASTSTATUS]     #number of seconds since last TXed output status
        protocol=module[LASTPROTOCOL]           # 1=old protocol, 2=new protocol
        if (len(txQueue[frameAddr])>0):
            retry=module[LASTRETRY]                         #number of retris (0,1,2,3...): used to compute the retry period
            if (retry>TX_RETRY):
                retry=TX_RETRY
            if (timeFromLastTx>(TX_RETRY_TIME<<(retry+1))):
                #Log(LOG_DEBUG,"send(): frameAddr="+hex(frameAddr)+" protocol="+str(protocol)+" timeFromLastTx="+str(timeFromLastTx)+"ms timeFromLastRx="+str(timeFromLastRx)+"s lastStatus="+str(timeFromLastStatus)+"s")
                if (protocol==0 and retry>=TX_RETRY-5 and (retry&1)):
                    protocol=1  #protocol not defined: maybe it's a old device that does not transmit periodic status
                #start txing
                tx=1
                txbuffer=bytearray()
                if (protocol==1):
                    txbuffer.append(PREAMBLE_MASTER)
                    txbuffer.append(frameAddr>>8)
                    txbuffer.append(frameAddr&0xff)
                    txbuffer.append(0)
                    txbufferIndex=FRAME_HEADER
                else: #protocol=2 or protocol=0
                    txbuffer.append(PREAMBLE)
                    txbuffer.append(frameAddr>>8)       #dstAddr
                    txbuffer.append(frameAddr&0xff)
                    txbuffer.append(0)                  #master address
                    txbuffer.append(0)
                    txbuffer.append(0)                  #length
                    txbufferIndex=FRAME_HEADER2
                # transmit ACK first: build a new queue with all ACK and commands for the selected module frameAddr
                txQueueNow=[]
                for txq in txQueue[frameAddr][:]:    #iterate a copy of txQueue[frameAddr]
                    (cmd,cmdLen,cmdAck,port,args,retry)=txq
                    if (cmdAck): txQueueNow.append(txq)
                for txq in txQueue[frameAddr][:]:    #iterate a copy of txQueue[frameAddr]
                    (cmd,cmdLen,cmdAck,port,args,retry)=txq
                    if (cmdAck==0): txQueueNow.append(txq)

                for txq in txQueueNow:    #iterate txQueueNow
                    #[cmd,cmdLen,cmdAck,port,[*args]]
                    (cmd,cmdLen,cmdAck,port,args,retry)=txq
                    #Log(LOG_DEBUG,"protocol="+str(protocol)+" cmd="+str(cmd)+" port="+str(port))
                    if (txbufferIndex+cmdLen+2>=FRAME_LEN_MAX): 
                        break   #frame must be truncated
                    if (protocol==1):
                        txbuffer.append((cmd|cmdLen|cmdAck))
                        txbufferIndex+=1
                    else:
                        txbuffer.append((cmd|cmdAck|int((cmdLen+1)/2)))   #cmdLen field is the number of cmd payload/2, so if after cmd there are 3 or 4 bytes, cmdLen field must be 2 (corresponding to 4 bytes)
                        txbufferIndex+=1
                    txbuffer.append(port)
                    txbufferIndex+=1
                    for i in range(0,cmdLen-1):
                        txbuffer.append((args[i]&0xff))
                        txbufferIndex+=1

                    if (protocol!=1 and (cmdLen&1)):  #cmdLen is odd => add a dummy byte to get even cmdLen
                        txbuffer.append(0)
                        txbufferIndex+=1

                    # if this cmd is an ACK, or values[0]==1, remove command from the queue
                    if (cmdAck or retry<=1):
                        txQueue[frameAddr].remove(txq)
                    else:
                        txq[TXQ_RETRIES]=retry-1   #command, no ack: decrement retry
                if (protocol==1):
                    txbuffer[FRAME_LEN]=(txbufferIndex-FRAME_HEADER)
                else:
                    txbuffer[FRAME_LEN2]=(txbufferIndex-FRAME_HEADER2)
                module[LASTRETRY]+=1    #increment RETRY to multiply the retry period * 2
                if (module[LASTRETRY]>=TX_RETRY):
                    module[LASTRETRY]=4;
                    module[LASTPROTOCOL]=0  #module does not renspond => reset protocol so both protocol 1 and 2 will be checked next time
                checksum(protocol, txbuffer)
                txbuffer.append(checksumValue)
                txbufferIndex+=1
                SerialConn.Send(txbuffer)
                if (logLevel>=LOG_DUMPALL or (logLevel>=LOG_DUMP and protocol!=1)):
                    dump(protocol, txbuffer, txbufferIndex,"TX")
                modules[frameAddr][LASTTX]=ms

        else: #No frame to be TXed for this frameAddr
            #check that module is active
            if (timeFromLastRx>MODULE_ALIVE_TIME):
                if (protocol==2 or PROTOCOL1_WITH_PERIODIC_TX):
                    # too long time since last RX from this module: remove it from modules
                    Log(LOG_INFO,"Remove module "+hex(frameAddr)+" because it's not alive")
                    delmodules.append(frameAddr)
                    # also remove any cmd in the txQueue
                    Log(LOG_INFO,"Remove txQueue for "+hex(frameAddr))
                    txQueueRemove(frameAddr,255,255,0)
                    Log(LOG_INFO,"Set devices in timedOut mode (red header) for this module")
                    deviceIDMask="H{:04x}_P".format(frameAddr)
                    for Device in Devices:
                        d=Devices[Device]
                        if (d.Used==1 and d.DeviceID[:7]==deviceIDMask):
                            # device is used and matches frameAddr
                            d.Update(nValue=d.nValue, sValue=d.sValue, TimedOut=1) #set device in TimedOut mode (red bar)

                #Note: if protocol==1, maybe it uses an old firmware that does not transmit status periodically: don't remove it
                
    for d in delmodules:    #remove module address of died modules (that do not answer since long time (MODULE_ALIVE_TIME))
        del modules[d]

    if (tx==0): #nothing has been transmitted: send outputs status for device with older lastStatus
        olderFrameAddr=0
        olderTime=sec
        # find the device that I sent the output status earlier
        for frameAddr,module in modules.items():
            if module[LASTSTATUS]<olderTime:
                #this is the older device I sent status, till now
                olderTime=module[LASTSTATUS]
                olderFrameAddr=frameAddr
        # transmit only the output status of the older device, if last time I transmitted the status was at least PERIODIC_STATUS_INTERVAL seconds ago
        if (sec-olderTime > PERIODIC_STATUS_INTERVAL):
            modules[olderFrameAddr][LASTSTATUS]=sec+(olderFrameAddr&0x000f)   #set current time + extra seconds to avoid all devices been refresh together
            #Log(LOG_DEBUG,"send(): Transmit outputs Status for "+hex(olderFrameAddr))
            txOutputsStatus(Devices, olderFrameAddr)
    return
def heartbeat(Devices):
    #function called periodically
    global portsDisabledWrite, modulesAskConfig
    #should I save portsDisabled dict on json file?
    if (portsDisabledWrite>0):
        portsDisabledWrite-=1
        if (portsDisabledWrite==0):
            portsDisabledWriteNow() #Write json file with list of disabled ports
            #ask configuration to enable ports that were previosly disabled and now are enabled again
            for frameAddr in modulesAskConfig:
                txQueueAskConfig(0,frameAddr)
            modulesAskConfig=[]

    #check counters: if configured as kWh => update decrease power in case of timeout
    delmodules=[]
    for u in counterTime:
        if (Devices[u].Type==243 and Devices[u].SubType==29): #kWh meter
            ms=int(time.time()*1000)
            msdiff=ms-counterTime[u] #elapsed time since last value
            if (msdiff>6000): 
                # start power decay only if more than 6s since last pulse (DomBus in counter mode transmits no more than 1 frame per 2s)
                sv=Devices[u].sValue.split(';')
                p=int(float(sv[0]))
                if (p>0):
                    pc=int(3600000/msdiff)
                    if (pc<p): #power was reduced
                        sv=str(pc)+";"+sv[1]
                        Devices[u].Update(nValue=0, sValue=sv)
        else:
            # this unit is not configured as a power meter => no need to compute power
            delmodules.append(u)
    for u in delmodules:
        del counterTime[u]

    return    
