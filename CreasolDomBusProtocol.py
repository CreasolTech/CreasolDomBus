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
TX_RETRY_TIME=50                # retry every TX_RETRY_TIME milliseconds
PERIODIC_STATUS_INTERVAL=300    #seconds: refresh output status to device every 5 minutes
MODULE_ALIVE_TIME=900           #if no frame is received in this time, module is considered dead (and periodic output status will not be transmitted)

CMD_CONFIG=0x00                 #Config port
CMD_GET=0x10                    #Get status
CMD_SET=0x20                    #Set outputs/values
CMD_DCMD_CONFIG=0xe0            #Send DCMD configuration
CMD_DCMD=0xf0                   #Receive DCMD command from Dombus

SUBCMD_CALIBRATE=0x00

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
PORTTYPE_1WIRE=0x1000           #1 wire
PORTTYPE_SENSOR_DISTANCE=0x2000 #distance measurement (send a pulse and measure echo delay)
PORTTYPE_SENSOR_TEMP=0x4000     #Temperature
PORTTYPE_SENSOR_HUM=0x8000      #Relative Humidity
PORTTYPE_SENSOR_TEMP_HUM=0xc000 #Temp+Hum
PORTTYPE_OUT_BLIND=0x01000000   #Blind output, close command (next port of DomBus device will be automatically used as Blind output, open command)
PORTTYPE_OUT_ANALOG=0x02000000  #0-10V output, 1% step, 0-100

PORTOPT_NONE=0x0000             #No options
PORTOPT_INVERTED=0x0001         #Logical inverted: MUST BE 1

PORTTYPE={PORTTYPE_OUT_DIGITAL:244, PORTTYPE_OUT_RELAY_LP:244, PORTTYPE_OUT_LEDSTATUS:244, PORTTYPE_OUT_DIMMER:244, PORTTYPE_OUT_BUZZER:244, PORTTYPE_IN_AC:244, PORTTYPE_IN_DIGITAL:244, PORTTYPE_IN_ANALOG:244, PORTTYPE_IN_TWINBUTTON:244, PORTTYPE_SENSOR_HUM:81, PORTTYPE_SENSOR_TEMP:80, PORTTYPE_SENSOR_TEMP_HUM:82, PORTTYPE_SENSOR_DISTANCE:243, PORTTYPE_OUT_BLIND:244, PORTTYPE_OUT_ANALOG:244,}

PORT_TYPENAME={PORTTYPE_OUT_DIGITAL:"Switch", PORTTYPE_OUT_RELAY_LP:"Switch", PORTTYPE_OUT_LEDSTATUS:"Switch", PORTTYPE_OUT_DIMMER:"Dimmer", PORTTYPE_OUT_BUZZER:"Switch", PORTTYPE_IN_AC:"Switch", PORTTYPE_IN_DIGITAL:"Switch", PORTTYPE_IN_ANALOG:"Voltage", PORTTYPE_IN_TWINBUTTON:"Selector Switch", PORTTYPE_SENSOR_HUM:"Humidity", PORTTYPE_SENSOR_TEMP:"Temperature", PORTTYPE_SENSOR_TEMP_HUM:"Temp+Hum", PORTTYPE_SENSOR_DISTANCE:"Distance", PORTTYPE_OUT_BLIND:"Switch", PORTTYPE_OUT_DIMMER:"Dimmer",}

PORTTYPES={
        "DISABLED":0x0000,      # port not used
        "OUT_DIGITAL":0x0002,   # relay output
        "OUT_RELAY_LP":0x0004,  # relay output
        "OUT_LEDSTATUS":0x0008, # output used as led status
        "OUT_DIMMER":0x0010,    # dimmer output
        "OUT_BUZZER":0x0020,    # buzzer output (2 ports, push-pull)
        "IN_AC":0x0040,         # input AC 50Hz (with optocoupler)
        "IN_DIGITAL":0x0080,    # input digital
        "IN_ANALOG":0x0100,     # input analog (ADC)
        "IN_TWINBUTTON":0x0200, # 2 buttons connected to a single input through a resistor
        "DISTANCE":0x2000,      # measure distance in mm
        "TEMPERATURE":0x4000,   # temperature
        "HUMIDITY":0x8000,      # relative humidity
        "TEMP+HUM":0xc000,      # temp+hum
        "OUT_BLIND":0x01000000, # blind with up/down/stop command
        "OUT_ANALOG":0x02000000,# 0-10V output, 0-100, 1% step
        }

PORTOPTS={
        "NORMAL":0x0000,          # no options defined
        "INVERTED":0x0001,      # input or output is inverted (logic 1 means the corresponding GPIO is at GND
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
        "HUMIDITY":"Humidity",
        "TEMPERATURE":"Temperature",
        "TEMP+HUM":"Temp+Hum",
        "DISTANCE":"Distance",
#        "OUT_BLIND":"Venetian Blinds EU", #not available in domoticz yet. hardware/plugins/PythonObjects.cpp must be updated!
        "OUT_BLIND":"Switch",
        "OUT_ANALOG":"Dimmer",
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
        "MAX":      9,      #max number of events
        }

DCMD_OUT_CMDS={
        "NONE":     0,      
        "OFF":      1,      #Turn off output
        "ON":       2,      #turn ON output
        "TOGGLE":   3,      #toggle output ON -> OFF -> ON -> ....
        "DIMMER":   4,      #set value
        #TODO: BLIND, BLINDSTOP, BLINDOPEN, BLINDCLOSE
        "MAX":      5,      #Max number of commands
        }

rxbuffer=bytearray()         #serial rx buffer
txbuffer=bytearray()         #serial tx buffer
rxbufferindex=0             
frameLen=0
frameAddr=0
checksumValue=0

LASTRX=0        # first field in modules[]
LASTTX=1        # second field in modules[]
LASTSTATUS=2    # third field in modules[]
LASTPROTOCOL=3  # forth field in modules[]
LASTRETRY=4     # fifth field in modules[]: number of retries (used to compute the tx period)
modules={}      # modules[frameAddr]=[lastRx, lastTx, lastSentStatus, protocol]

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
        if (logLevel>=LOG_DUMPALL):
            f="P:1 "
            for i in range(0, frameLen):
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
                if (cmdAck and int(buffer[i+1]==0xff)):
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
        if (unit in Devices.keys()):
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
    elif (d.Type==PORTTYPE[PORTTYPE_SENSOR_DISTANCE] or d.Type==PORTTYPE[PORTTYPE_IN_ANALOG]):
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
    if protocol==0:
        # check if module already in modules[]
        if (frameAddr in modules.keys()):
            protocol=modules[frameAddr][LASTPROTOCOL]
    if len(txQueue)==0 or frameAddr not in txQueue.keys():
        #create txQueue[frameAddr]
        txQueue[frameAddr]=[[cmd, cmdLen, cmdAck, port, args, retries]]
#        Log(LOG_DEBUG,"txQueueAdd (frameAddr do not exist) frameAddr="+hex(frameAddr)+" cmd="+hex(cmd|cmdAck|cmdLen)+" port="+hex(port))
    else:
        found=0
        for f in txQueue[frameAddr]:
            #f=[cmd,cmdlen,cmdAck,port,args[]]
            if (f[TXQ_CMD]==cmd and f[TXQ_CMDLEN]==cmdLen and f[TXQ_PORT]==port):
                #command already in txQueue: update values
                f[TXQ_CMDACK]=cmdAck
                f[TXQ_ARGS]=args
                if (f[TXQ_RETRIES]<retries):
                    f[TXQ_RETRIES]=retries
                found=1
                break
        if (found==0):
            txQueue[frameAddr].append([cmd,cmdLen,cmdAck,port,args,retries])
#            Log(LOG_DEBUG,"txQueueAdd: frameAddr="+hex(frameAddr)+" cmd="+hex(cmd|cmdAck|cmdLen)+" port="+hex(port))
        #txQueueRetry: don't modify it... transmit when retry time expires (maybe now or soon)
    #check that modules[frameAddr] exists
    if (frameAddr not in modules.keys()):
        # add frameAddr in modules
        #                   lastRx         lastTx   lastSentStatus => lastTx=0 => transmit now
        modules[frameAddr]=[sec, 0, sec+3-PERIODIC_STATUS_INTERVAL, protocol, 0] #transmit output status in 3 seconds
    else:
        #frameAddr already in modules[]
        if (protocol!=0):
            modules[frameAddr][LASTPROTOCOL]=protocol
        if (now):
            modules[frameAddr][LASTTX]=0 #transmit now
    return        

def txQueueAskConfig(protocol, frameAddr):
    txQueueAdd(protocol, frameAddr,CMD_CONFIG,1,0,0xff,[],TX_RETRY,1)    #port=0xff to ask full configuration 
    return 

def txQueueRemove(frameAddr,cmd,port):
    # if txQueue[frameAddr] esists, remove cmd and port from it.
    # if cmd==255 and port==255 => remove all frames for module frameAddr
    global txQueue
    removeItems=[]
    if len(txQueue)!=0 and frameAddr in txQueue.keys():
        for f in txQueue[frameAddr][:]:
            #Log(LOG_DEBUG,"f="+str(f))
            #f=[cmd,cmdlen,cmdAck,port,args[],retries]
            if (((cmd&port)==255) or (f[TXQ_CMD]==cmd and f[TXQ_PORT]==port)):
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
                    if (re.search("OUT_ANALOG",d.Description)):
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
                if (re.search('OUT_ANALOG',d.Description)):
                    txQueueAdd(0, frameAddr,CMD_SET,2,0,port,[int(Level)],TX_RETRY,1) #Level: from 0 to 100 = 100% (1% step)
                else:
                    txQueueAdd(0, frameAddr,CMD_SET,2,0,port,[int(Level/5)],TX_RETRY,1) #Level: from 0 to 20 = 100% (5% step)
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
    #read device description and set the last defined type (if written in the description, else transmit 0xffff that mean NO-CHANGE) and the port options (ORed) (if no options are written in the description, transmits 0xffff that mean NO CHANGE).
    global txQueue, portsDisabled, portsDisabledWrite, deviceID, devID
    getDeviceID(frameAddr,port)
    Log(LOG_DEBUG,"Parse Description field for device "+devID)
    setOpt=0
    setType=0
    setHwaddr=0
    setCal=32768    #calibration offset 32768=ignore
    setTypeName=''
    typeName=''
    setOptDefined=0
    setTypeDefined=0
    setOptNames=""
    #dombus command
#    dcmd=[  # IN_EVENT,                 inValueL,   inValueH,   hwaddr, port,   outCmd,                 outValue
#            [ DCMD_IN_EVENTS['NONE'],   0,          0,          0,      0,      DCMD_OUT_CMDS['NONE'],  0 ],
#            ]
    dcmd=[]   
    for opt in opts:
        opt=opt.strip()
        if opt in PORTTYPES.keys():
            #opt="OUT_DIGITAL" or DISTANCE or ....
            setType=PORTTYPES[opt]      # setType=0x2000 if DISTANCE is specified
            setTypeDefined=1            #setTypeDefined=1
            setTypeName=opt             #setTypeName=DISTANCE
            typeName=PORTTYPENAME[opt]  #typeName=Temperature
        elif opt in PORTOPTS.keys():
            if (opt=="NORMAL"):
                setOpt=0
                setOptNames=''
            else:
                setOpt=setOpt|PORTOPTS[opt]
                setOptNames+=opt+","
            setOptDefined=1
        elif opt[:2]=="A=":
            setOptNames+=opt+","
        elif opt[:2]=="B=":
            setOptNames+=opt+","
        elif opt[:4]=="CAL=":       # calibration value: should be expressed as integer (e.g. temperatureOffset*10)
            setCal=int(float(opt[4:])*10) 
        elif opt[:9]=="TYPENAME=":
            typeName=opt[9:]
            setOptNames+=opt+","
        elif opt[:9]=="HWADDR=0X" and len(opt)==13:
            #set hardware address
            hwaddr=int(opt[7:],16)
            if (hwaddr>=1 and hwaddr<65535):
                setHwaddr=hwaddr
        elif opt[:8]=="DISABLE=":
            #set which ports are enabled, for this device, and remove disabled ports to free space in Devices[]
            #syntax: ENABLE=1:2:3:4:5
            setOptNames+="DISABLE="
            portsDisabled[deviceAddr]=[]
            portsDisabledWrite=6   #write portDisabled on a json file in 6*heartbeat_interval (6*10s)
            portsDisabledWrite=1   ##DEBUG

            for ps in opt[8:].split(':'):
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
        elif (opt[:6]=="DESCR=" or opt[:10]=="TIMECLOSE=" or opt[:9]=="TIMEOPEN="):
            setOptNames+=opt+","
        elif (opt[:5]=="DCMD("):
            #command to another dombus
            errmsg=''  
            d=[ DCMD_IN_EVENTS['NONE'], 0, 0, 0, 0, DCMD_OUT_CMDS['NONE'], 0 ] #temp list to store a DCMD command
            
            opt=re.sub("ERROR=.*", "", opt) #remove any Error=blablabla from the command
            inputs=re.search('DCMD\((.+)\)=(.+\..+:.+)', opt)
            if inputs:
                #syntax of DCMD command semms to be ok
                inArr=inputs.group(1).split(':')    #inArr=['Value','0','20.5'] (inputs) 
                outArr=inputs.group(2).split(':')   #
                if (len(inArr)>=1):
                    Log(LOG_INFO,"DCMD: "+opt+" Input event="+str(inArr)+" Output command="+str(outArr))
                    if (inArr[0] in DCMD_IN_EVENTS.keys()): 
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
                            if (outArr[1] in DCMD_OUT_CMDS.keys()):
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
    Log(LOG_INFO,"Config device "+hex(frameAddr)+": type="+hex(setType)+" typeName="+typeName+" opts="+hex(setOpt))
    txQueueAdd(0, frameAddr, CMD_CONFIG, 5, 0, port, [((setType>>8)&0xff), (setType&0xff), (setOpt >> 8), (setOpt&0xff)], TX_RETRY,1) #PORTTYPE_VERSION=1
    txQueueAdd(0, frameAddr, CMD_CONFIG, 7, 0, port, [((setType>>24)&0xff), ((setType>>16)&0xff), ((setType>>8)&0xff), (setType&0xff), (setOpt >> 8), (setOpt&0xff)], TX_RETRY,1) #PORTTYPE_VERSION=2
    if (modules[frameAddr][LASTPROTOCOL]!=1):
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
    if (setTypeDefined): #type was defined in the description => change TypeName, if different
        if (typeName=="Selector Switch"):
            Options = {"LevelActions": "||","LevelNames": "Off|Down|Up", "LevelOffHidden": "false","SelectorStyle": "0"}
            Devices[Unit].Update(TypeName=typeName, nValue=0, sValue="Off", Description=str(descr), Options=Options)  # Update description (removing HWADDR=0x1234)
        else:
            Devices[Unit].Update(TypeName=typeName, nValue=Devices[Unit].nValue, sValue=Devices[Unit].sValue, Description=str(descr))  # Update description (removing HWADDR=0x1234)
    else: #type not defined in description => don't change it!
        Devices[Unit].Update(nValue=Devices[Unit].nValue, sValue=Devices[Unit].sValue, Description=str(descr))  # Update description (removing HWADDR=0x1234)
    if (setCal!=32768): #new calibration value
        if (setCal<0): 
            setCal+=65536
        txQueueAdd(0, frameAddr, CMD_CONFIG, 4, 0, port, [SUBCMD_CALIBRATE, ((setCal>>8)&0xff), (setCal&0xff)], TX_RETRY, 1)
    if (setHwaddr!=0 and setHwaddr!=0xffff): #hwaddr not 0 
        # remove HWADDR=0X1234 option from the device description
        # send command to change hwaddr
        txQueueAdd(0, frameAddr, CMD_CONFIG, 3, 0, 0, [(setHwaddr >> 8), (setHwaddr&0xff)], TX_RETRY,1)
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
        if (frameLen>=FRAME_LEN_MIN and len(rxbuffer)>=frameLen ):
            #length of frame is in the range
            # compute and compare checksum
            checksum(protocol, rxbuffer)
            dump(protocol, rxbuffer,frameLen,"RX")
            if (checksumValue == rxbuffer[frameLen-1]):
                #frame checksum is ok
                frameAddr=int(rxbuffer[1])*256+int(rxbuffer[2])
                frameIdx=FRAME_HEADER
                dstAddr=0;  #not specified in this protocol => force dstAddr=0
                if frameAddr!=0xffff and frameAddr not in modules.keys(): 
                    # first time receive data from this module: ask for configuration?
                    modules[frameAddr]=[int(time.time()),0,0,protocol,0]   #transmit now the output status
                    #ask for port configuration?
                    if rxbuffer[FRAME_HEADER]!=CMD_CONFIG:
                        txQueueAskConfig(protocol, frameAddr) #ask for configuration only if the device is not already sending configuration
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
        if (frameLen>=FRAME_LEN_MIN2 and len(rxbuffer)>=frameLen ):
            #length of frame is in the range
            # compute and compare checksum
            checksum(protocol, rxbuffer)
            dump(protocol, rxbuffer,frameLen,"RX")
            if (checksumValue == rxbuffer[frameLen-1]):
                #frame checksum is ok
                frameAddr=int(rxbuffer[3])*256+int(rxbuffer[4]) #sender
                dstAddr=int(rxbuffer[1])*256+int(rxbuffer[2])   #destination
                frameIdx=FRAME_HEADER2
                if (frameAddr!=0xffff and frameAddr not in modules.keys()): 
                    # first time receive data from this module: ask for configuration?
                    modules[frameAddr]=[int(time.time()),0,0,protocol,0]   #transmit now the output status
                    #ask for port configuration?
                    if rxbuffer[FRAME_HEADER2]!=CMD_CONFIG:
                        txQueueAskConfig(protocol, frameAddr) #ask for configuration only if the device is not already sending configuration
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
                    txQueueRemove(frameAddr,cmd, port)
                    modules[frameAddr][LASTRETRY]=0
                if (cmd==CMD_CONFIG and dstAddr==0):
                    if (port==0xff):    
                        #0xff VERSION PORTTYPE PORTOPT PORTCAPABILITIES PORTIMAGE PORTNAME
                        #arg1 contains the PORTTYPE_VERSION (to extend functionality in the future)
                        port=1        #port starts with 1
                        portVer=arg1  #protocol version used to exchange information
                        frameIdx=portIdx+2
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
                            if ((deviceAddr not in portsDisabled.keys() or port not in portsDisabled[deviceAddr])):
                                if (unit!=0xffff):
                                    #unit found => remove TimedOut if set
                                    if port==1:
                                        Log(LOG_INFO,"Device "+"{:x}".format(frameAddr)+" has become active again")
                                    Devices[unit].Update(nValue=Devices[unit].nValue, sValue=Devices[unit].sValue, TimedOut=0)
                                else:
                                    #port device not found, and is not disabled: create it!
                                    #portType is the numeric number provided by DOMBUS
                                    if (portType not in PORT_TYPENAME): portType=PORTTYPE_IN_DIGITAL   #default: Digital Input
                                    if (UnitFree>255 or UnitFree<1):
                                        Log(LOG_ERR,"Maximum number of devices is reached! Domoticz supports max 255 devices for each protocol.\nUse more serial ports to separate modules in more buses, or disable unused ports in this way:\nselect port 1 of a module, and write in the description DISABLE=3:4:5:11 to disable, for example, ports 3,4,5 and 11 of that module")
                                    else:
                                        descr='ID='+devID+','
                                        for key, value in PORTTYPES.items():
                                            if (value==portType):
                                                descr+=key+","
                                                break
                                        for key,value in PORTOPTS.items():
                                            if (value&portOpt):
                                                #descr=descr+key+","
                                                descr+=key+","
                                        if (descr!=''):
                                            descr=descr[:-1]    #remove last comma ,
                                        Log(LOG_INFO,"Add device "+deviceID+" deviceAddr="+deviceAddr+": "+portName+" Type="+str(portType)+" Opt="+str(portOpt)+" Description="+descr)
                                        Log(LOG_INFO,"Name=["+devID+"] "+portName+", TypeName="+PORT_TYPENAME[portType]+", DeviceID="+deviceID+", Unit="+str(UnitFree))
                                        Domoticz.Device(Name="["+devID+"] "+portName, TypeName=PORT_TYPENAME[portType], DeviceID=deviceID, Unit=UnitFree).Create()
                                        if (frameAddr<=0xff00 or port==1):
                                            Devices[UnitFree].Update(nValue=0, sValue='', Description=descr, Used=1)  # Add description (cannot be added with Create method)
                            port+=1;
                #TODO elif (cmd==CMD_DCMD): #decode DCMD frame to get the STATUS word?
            else:
                #cmdAck==0 => decode command from slave module
                if (frameAddr!=0xffff and dstAddr==0):
                    #Receive command from a slave module
                    getDeviceID(frameAddr,port)
                    if (cmd==CMD_SET):
                        #digital or analog input changed?
                        UnitFree=1
                        unit=getDeviceUnit(Devices,0)
                        if (unit==0xffff):
                            #got a frame from a unknown device: ask for configuration
                            txQueueAskConfig(protocol,frameAddr)
                        else:
                            d=Devices[unit]
                            #got a frame from a well known device
                            getDeviceID(frameAddr,port)
                            if (cmdLen==2):
                                stringval='Off' if arg1==0 else 'On'
                                #update device only if was changed
                                if (d.Type==PORTTYPE[PORTTYPE_OUT_DIGITAL]):
                                    if (hasattr(d,'SwitchType') and d.SwitchType==18): #selector
                                        d.Update(nValue=int(arg1), sValue=str(arg1))
                                        #Log(LOG_DEBUG,"devID="+devID+" d.SwitchType="+str(d.SwitchType)+" nValue="+str(arg1)+" sValue="+str(arg1))
                                    elif (hasattr(d,'SwitchType') and d.SwitchType==7): #dimmer
                                        if (d.Level!=int(arg1)):
                                            d.Update(Level=int(arg1))
                                    else: #normal switch
                                        if (d.nValue!=int(arg1) or d.sValue!=stringval):
                                            d.Update(nValue=int(arg1), sValue=stringval)
                                else:
                                    #device is not a switch, or has not the SwitchType attribute
                                    Log(LOG_DEBUG,"Ignore SET command because Type="+str(d.Type)+" SubType="+str(d.SubType)+" Name="+d.Name+" has not attribute SwitchType")
    
                                #send ack
                                txQueueAdd(protocol, frameAddr,CMD_SET,2,CMD_ACK,port,[arg1],1,1)
                            elif (cmdLen==3 or cmdLen==4):
                                #analog value, distance, temperature or humidity
                                value=arg1*256+arg2
                                if (d.Type==PORTTYPE[PORTTYPE_SENSOR_TEMP]):
                                    temp=round(value/10.0-273.1,1)
                                    #Log(LOG_DEBUG,"Temperature: value="+str(value)+" temp="+str(temp)) 
                                    stringval=str(float(temp))
                                    if (temp>-50 and d.sValue!=stringval):
                                        d.Update(nValue=int(temp), sValue=stringval)
                                elif (d.Type==PORTTYPE[PORTTYPE_SENSOR_HUM]):
                                    hum=int(value/10)
                                    if (hum>5 and d.nValue!=hum):
                                        d.Update(nValue=hum, sValue=HRstatus(hum))
                                elif (d.Type==PORTTYPE[PORTTYPE_SENSOR_DISTANCE] or d.Type==PORTTYPE[PORTTYPE_IN_ANALOG]):
                                    #extract A and B, if defined, to compute the right value VALUE=A*dombus_value+B
                                    v=getOpt(d,"A=")
                                    a=float(v) if (v!="false") else 1
                                    v=getOpt(d,"B=")
                                    b=float(v) if (v!="false") else 0
                                    Value=a*value+b
                                    if (d.sValue!=str(Value)):
                                        d.Update(nValue=int(Value), sValue=str(Value))
                                    #Log(LOG_DEBUG,"Value="+str(a)+"*"+str(value)+"+"+str(b)+"="+str(Value))
                                txQueueAdd(protocol, frameAddr,CMD_SET,3,CMD_ACK,port,[arg1,arg2,0],1,1)
                            elif (cmdLen==5 or cmdLen==6):
                                #temp+hum
                                value=arg1*256+arg2
                                value2=arg3*256+arg4
                                if (d.Type==PORTTYPE[PORTTYPE_SENSOR_TEMP_HUM]):
                                    temp=round(value/10.0-273.1,1)
                                    hum=int(value2/10)
                                    stringval=str(float(temp))+";"+str(hum)+";"+HRstatus(hum) 
                                    #Log(LOG_DEBUG,"TEMP_HUM: nValue="+str(temp)+" sValue="+stringval)
                                    if (temp>-50 and hum>5 and d.sValue!=stringval):
                                        d.Update(nValue=int(temp), sValue=stringval)
                                    txQueueAdd(protocol, frameAddr,CMD_SET,5,CMD_ACK,port,[arg1,arg2,arg3,arg4,0],1,1)
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
                return
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
        #Log(LOG_DEBUG,"send(): frameAddr="+hex(frameAddr)+" protocol="+str(protocol)+" timeFromLastTx="+str(timeFromLastTx)+"ms timeFromLastRx="+str(timeFromLastRx)+"s lastStatus="+str(timeFromLastStatus)+"s")
        if (len(txQueue[frameAddr])>0):
            retry=module[LASTRETRY]                         #number of retris (0,1,2,3...): used to compute the retry period
            if (retry>TX_RETRY):
                retry=TX_RETRY
            if (timeFromLastTx>(TX_RETRY_TIME<<retry)):
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
                else: #protocol=2
                    txbuffer.append(PREAMBLE)
                    txbuffer.append(frameAddr>>8)       #dstAddr
                    txbuffer.append(frameAddr&0xff)
                    txbuffer.append(0)                  #master address
                    txbuffer.append(0)
                    txbuffer.append(0)                  #length
                    txbufferIndex=FRAME_HEADER2
                for txq in txQueue[frameAddr][:]:    #iterate a copy of txQueue[frameAddr]
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
                    txQueueRemove(frameAddr,255,255)
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



