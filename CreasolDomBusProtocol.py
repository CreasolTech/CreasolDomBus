# Define constants and functions needed by CreasolDom RS485 bus protocol 

"""
Protocol definition (in bytes):
    preamble addr[2] length cmd1 arg1_1 [arg1_2] [arg1_3] cmd2 arg2_1 [arg2_2] [arg2_3] ...... checksum

    preamble: 0x5a when transmitted by the master controller, 0xda when transmitted by the slave device

    address (2 bytes): 
        0 => master (domoticz)
        0xffff => broadcast
        1 => default slave address

    length: payload lenght (from cmd1 to checksum, excluding checksum)    

    cmd: syntax: CCCC ALLL  where CCCC is the command family, A=1 when this is an acknowledge, LLL is the number of arguments for that command
    
    args: at least 1 argument for each command

    checksum: simply a checksum from preamble to checksum, excluding checksum

"""
import Domoticz
import time
import struct
import re

#some constants
FRAME_LEN_MIN=6
FRAME_LEN_MAX=31    #max length for TX (devices cannot handle long frames)
FRAME_LEN=3
FRAME_HEADER=4
PREAMBLE_MASTER=0x5a
PREAMBLE_DEVICE=0xda
CMD_LEN_MASK=0x07
CMD_MASK=0xf0
CMD_ACK=0x08

TX_RETRY=10                     #max number of retries
TX_RETRY_TIME=500               # retry every TX_RETRY_TIME milliseconds
PERIODIC_STATUS_INTERVAL=300    #seconds: refresh output status to device every 5 minutes
MODULE_ALIVE_TIME=900           #if no frame is received in this time, module is considered dead (and periodic output status will not be transmitted)

CMD_CONFIG=0x00
CMD_GET=0x10
CMD_SET=0x20

PORTTYPE_DISABLED=0x0000        #port not used
PORTTYPE_OUT_DIGITAL=0x0002     #digital, opto or relay output
PORTTYPE_OUT_RELAY_LP=0x0004    #relay output with lowpower PWM
PORTTYPE_IN_AC=0x0040           #input AC 50Hz (with optocoupler)
PORTTYPE_IN_DIGITAL=0x0080      #input digital
PORTTYPE_IN_ANALOG=0x0100       #input analog (ADC)
PORTTYPE_1WIRE=0x1000           #1 wire
PORTTYPE_DISTANCE=0x2000        #distance measurement (send a pulse and measure echo delay)

PORTOPT_NONE=0x0000             #No options
PORTOPT_INVERTED=0x0001         #Logical inverted: MUST BE 1

PORTTYPE={PORTTYPE_OUT_DIGITAL:244, PORTTYPE_OUT_RELAY_LP:244, PORTTYPE_IN_AC:244, PORTTYPE_IN_DIGITAL:244, PORTTYPE_DISTANCE:243}
PORT_TYPENAME={PORTTYPE_OUT_DIGITAL:"Switch", PORTTYPE_OUT_RELAY_LP:"Switch", PORTTYPE_IN_AC:"Switch", PORTTYPE_IN_DIGITAL:"Switch", PORTTYPE_DISTANCE:"Distance"}

PORTTYPES={
        "DISABLED":0x0000,      # port not used
        "OUT_DIGITAL":0x0002,   # relay output
        "OUT_RELAY_LP":0x0004,  # relay output
        "IN_AC":0x0040,         # input AC 50Hz (with optocoupler)
        "IN_DIGITAL":0x0080,    # input digital
        "IN_ANALOG":0x0100,     # input analog (ADC)
        "1WIRE":0x1000,         # 1 wire
        "DISTANCE":0x2000,      # measure distance in mm
        }

PORTOPTS={
        "NORMAL":0x0000,          # no options defined
        "INVERTED":0x0001,      # input or output is inverted (logic 1 means the corresponding GPIO is at GND
        }

PORTTYPENAME={  #Used to set the device TypeName
        "DISABLED":"Switch",
        "OUT_DIGITAL":"Switch",
        "OUT_RELAY_LP":"Switch",
        "IN_AC":"Switch",
        "IN_DIGITAL":"Switch",
        "IN_ANALOG":"Percentage",
        "1WIRE":"Temperature",
        "DISTANCE":"Distance",
        }

rxbuffer=bytearray()         #serial rx buffer
txbuffer=bytearray()         #serial tx buffer
rxbufferindex=0             
frameLen=0
frameAddr=0
checksumValue=0

LASTRX=0        # first field in modules[]
LASTTX=1        # second field in modules[]
LASTSTATUS=2    # third filed in modules
modules={}      # modules[frameAddr]=[lastRx, lastTx, lastSentStatus]
txQueue={}      # tx queue for each module

def __init__(self):
    return

def checksum(buffer):
    global checksumValue, FRAME_LEN, FRAME_HEADER
    checksumValue=0
    length=buffer[FRAME_LEN]+FRAME_HEADER
    for i in range(0, length):
        checksumValue+=buffer[i]
    checksumValue&=0xff        
    return

def dump(buffer,frameLen,direction):
    #buffer=frame buffer
    #frameLen=length of frame in bytes
    #direction="RX" or "TX"
    f=""
    for i in range(0, frameLen):
        f+="%.2x " % int(buffer[i])
    Domoticz.Debug(direction+" frame: "+f)
    return

def getDeviceID(frameAddr, port):
    global deviceID
    deviceID="H{:04x}_P{:04x}".format(frameAddr, port)
    return

def getDeviceUnit(Devices,findUnitFreeMax):
    #given the deviceID (global var), scan Devices to find the Unit of the associated deviceID
    #also, set the variables 
    #  UnitFree=first free unit
    #  UnitMax=the greatest unit+1
    global deviceID, UnitFree, UnitMax
    found=0xffff
    for Device in Devices:
        unit=Devices[Device].Unit
        if (findUnitFreeMax): 
            if (unit==UnitFree): UnitFree+=1
            if (unit>=UnitMax): UnitMax=unit+1
        if (Devices[Device].DeviceID==deviceID):
            #port device found: ignore
            found=unit
            if findUnitFreeMax==0:
                #stop scanning, don't need to find UnitFree and UnitMax to store new device
                break
    return found

def txQueueAdd(frameAddr,cmd,cmdLen,cmdAck,port,args,retries,now):
    #add a command in the tx queue for the specified module (frameAddr)
    #if that command already exists, update it
    global txQueue
    sec=int(time.time())
    if len(txQueue)==0 or frameAddr not in txQueue.keys():
        #create txQueue[frameAddr]
        txQueue[frameAddr]=[[cmd, cmdLen, cmdAck, port, args, retries]]
#        Domoticz.Debug("txQueueAdd (frameAddr do not exist) frameAddr="+hex(frameAddr)+" cmd="+hex(cmd|cmdAck|cmdLen)+" port="+hex(port))
    else:
        found=0
        for f in txQueue[frameAddr]:
            #f=[cmd,cmdlen,cmdAck,port,args[]]
            if (f[0]==cmd and f[3]==port):
                f[1]=cmdLen
                f[2]=cmdAck
                f[4]=args
                if (f[5]<retries):
                    f[5]=retries
                found=1
                break
        if (found==0):
            txQueue[frameAddr].append([cmd,cmdLen,cmdAck,port,args,retries])
#            Domoticz.Debug("txQueueAdd: frameAddr="+hex(frameAddr)+" cmd="+hex(cmd|cmdAck|cmdLen)+" port="+hex(port))
        #txQueueRetry: don't modify it... transmit when retry time expires (maybe now or soon)
    #check that modules[frameAddr] exists
    if (frameAddr not in modules.keys()):
        # add frameAddr in modules
        #                   lastRx         lastTx   lastSentStatus => lastTx=0 => transmit now
        modules[frameAddr]=[sec, 0, sec+3-PERIODIC_STATUS_INTERVAL] #transmit output status in 3 seconds
    elif (now):
        modules[frameAddr][LASTTX]=0 #transmit now
    return        

def txQueueAskConfig(frameAddr):
    txQueueAdd(frameAddr,CMD_CONFIG,1,0,0xff,[],TX_RETRY,1)    #port=0xff to ask entire configuration
    return 

def txQueueRemove(frameAddr,cmd,port):
    # if txQueue[frameAddr] esists, remove cmd and port from it.
    # if cmd==255 and port==255 => remove all frames for module frameAddr
    global txQueue
    removeItems=[]
    if len(txQueue)!=0 and frameAddr in txQueue.keys():
        for f in txQueue[frameAddr][:]:
            Domoticz.Debug("f="+str(f))
            #f=[cmd,cmdlen,cmdAck,port,args[],retries]
            if (((cmd&port)==255) or (f[0]==cmd and f[3]==port)):
                txQueue[frameAddr].remove(f)
    return
    
def txOutputsStatus(Devices,frameAddr):
    # transmit the status of outputs for the device frameAddr
    Domoticz.Log("Send outputs status for device "+hex(frameAddr))
    for Device in Devices:
        deviceIDMask="H{:04x}_P".format(frameAddr)
        if (Devices[Device].Used==1 and Devices[Device].DeviceID[:7]==deviceIDMask):
            # device is used and matches frameAddr
            # check that this is an output
            if (re.search('(OUT_DIGITAL|OUT_RELAY_LP)',Devices[Device].Description)):
                # output! get the port and output state
                port=int("0x"+Devices[Device].DeviceID[7:11],0)
                state=1 if Devices[Device].nValue==1 else 0
                txQueueAdd(frameAddr,CMD_SET,2,0,port,[state],1,0) #retries=1
    return

def parseCommand(Devices, unit, Command, Level, Hue, frameAddr, port):
    newstate=1 if Command=="On" else 0
    #send command to the module
    d=Devices[unit]
    deviceID=d.DeviceID
    #add command in the queue
    if (d.Type==PORTTYPE[PORTTYPE_OUT_DIGITAL]):
        txQueueAdd(frameAddr,CMD_SET,2,0,port,[newstate],TX_RETRY,1)
    else:
        Domoticz.Debug("Ignore command due to unmanaged Type="+d.Type)
    return

def parseTypeOpt(Devices, Unit, opts, frameAddr, port):
    #read device description and set the last defined type (if written in the description, else transmit 0xffff that mean NO-CHANGE) and the port options (ORed) (if no options are written in the description, transmits 0xffff that mean NO CHANGE).
    global txQueue
    setOpt=0
    setType=0
    setHwaddr=0
    setTypeName=''
    typeName='Switch'
    setOptDefined=0
    setTypeDefined=0
    setOptNames=""
    for opt in opts:
        opt=opt.strip()
        if opt in PORTTYPES.keys():
            #opt="OUT_DIGITAL" or DISTANCE or ....
            setType=PORTTYPES[opt]
            setTypeDefined=1
            setTypeName=opt
            typeName=PORTTYPENAME[opt]
        elif opt in PORTOPTS.keys():
            setOpt=setOpt|PORTOPTS[opt]
            setOptDefined=1
            setOptNames+=opt+","
        elif opt[:2]=="A=":
            setOptNames+=opt+","
        elif opt[:2]=="B=":
            setOptNames+=opt+","
        elif opt[:9]=="TYPENAME=":
            typeName=opt[9:]
            setOptNames+=opt+","
        elif opt[:9]=="HWADDR=0X" and len(opt)==13:
            #set hardware address
            hwaddr=int(opt[7:],16)
            if (hwaddr>=1 and hwaddr<65535):
                setHwaddr=hwaddr
    if (setOptNames!=''):
        setOptNames=setOptNames[:-1]    #remove last comma ,
    Domoticz.Debug("Config device "+str(frameAddr)+": type="+hex(setType)+" typeName="+typeName+" opts="+hex(setOpt))
    txQueueAdd(frameAddr, CMD_CONFIG, 5, 0, port, [(setType >> 8), (setType&0xff), (setOpt >> 8), (setOpt&0xff)], TX_RETRY,1)
    descr=setTypeName+','+setOptNames if (setTypeDefined==1) else setOptNames
    Devices[Unit].Update(TypeName=typeName, nValue=Devices[Unit].nValue, sValue=Devices[Unit].sValue, Description=str(descr))  # Update description (removing HWADDR=0x1234)
    if (setHwaddr!=0 and setHwaddr!=0xffff): #hwaddr not 0 
        # remove HWADDR=0X1234 option from the device description
        # send command to change hwaddr
        txQueueAdd(frameAddr, CMD_CONFIG, 3, 0, 0, [(setHwaddr >> 8), (setHwaddr&0xff)], TX_RETRY,1)
    return

def decode(Devices):
    # align rxbuffer[] so it starts with a preamble
    global modules, txQueue, rxbuffer, UnitFree, UnitMax, deviceID
#    dump(rxbuffer,len(rxbuffer),"<<< RXNOCHECK ")
    while (len(rxbuffer)>=FRAME_LEN_MIN and rxbuffer[0]!=PREAMBLE_DEVICE):
        rxbuffer.pop(0)
    if (len(rxbuffer)<FRAME_LEN_MIN): 
        return
    #decode frame RXed from serial port
    #frame structure was explained above, in the comments
    frameLen=int(rxbuffer[FRAME_LEN])+FRAME_HEADER+1
    if (frameLen>=FRAME_LEN_MIN and len(rxbuffer)>=frameLen ):
        #length of frame is in the range
        # compute and compare checksum
        checksum(rxbuffer)
        dump(rxbuffer,frameLen,"<<< RX")
        if (checksumValue == rxbuffer[frameLen-1]):
            #frame checksum is ok
            frameAddr=int(rxbuffer[1])*256+int(rxbuffer[2])
            frameIdx=FRAME_HEADER
            if frameAddr!=0xffff and frameAddr not in modules.keys(): 
                # first time receive data from this module: ask for configuration?
                modules[frameAddr]=[int(time.time()),0,0]   #transmit now the output status
                #ask for port configuration?
                if rxbuffer[FRAME_HEADER]!=0x00:
                    txQueueAskConfig(frameAddr) #ask for configuration only if the device is not already sending configuration
            #broadcast or txQueue exists or txQueue just created
            while (frameIdx<frameLen-2):
                cmd=rxbuffer[frameIdx]
                cmdAck=cmd&CMD_ACK
                cmdLen=cmd&CMD_LEN_MASK
                cmd&=CMD_MASK
                if (cmdLen==0):
                    Domoticz.Debug("Error: cmdLen==0")
                    break;
                arg1=rxbuffer[frameIdx+1] if (cmdLen>=1) else 0  
                arg2=rxbuffer[frameIdx+2] if (cmdLen>=2) else 0
                arg3=rxbuffer[frameIdx+3] if (cmdLen>=3) else 0
                getDeviceID(frameAddr, arg1)
                if (cmdAck):
                    # received an ack: remove cmd+arg1 from txQueue, if present
                    if (frameAddr!=0xffff):
                        #Received ACK from a slave module
                        txQueueRemove(frameAddr,cmd,arg1)
                    if (cmd==CMD_CONFIG):
                        if (arg1==0xff):    #arg1 = first module
                            #0xff VERSION PORTTYPE PORTOPT PORTCAPABILITIES PORTIMAGE PORTNAME
                            #arg2 contains the PORTTYPE_VERSION (to extend functionality in the future)
                            frameIdx+=2     #points to the configuration array
                            port=1          #port starts with 1
                            portVer=rxbuffer[frameIdx]  #protocol version used to exchange information
                            frameIdx+=1
                            while (frameIdx < frameLen-1):
                                portType=int(rxbuffer[frameIdx])*256+int(rxbuffer[frameIdx+1])
                                frameIdx+=2
                                portOpt=int(rxbuffer[frameIdx])*256+int(rxbuffer[frameIdx+1])
                                frameIdx+=2
                                portCapabilities=int(rxbuffer[frameIdx])*256+int(rxbuffer[frameIdx+1])
                                frameIdx+=2
                                portImage=int(rxbuffer[frameIdx])
                                frameIdx+=1
                                portName=""
                                getDeviceID(frameAddr, port)
                                for i in range(0,16):
                                    ch=rxbuffer[frameIdx]
                                    frameIdx+=1
                                    if (ch==0):
                                        break
                                    else:
                                        portName+=chr(ch)
                                #check if this port device already exists?
                                UnitFree=1  # first free unit in Devices[]
                                UnitMax=1   # max value+1 of unit in Devices[] (to add a group of devices with continuous units)
                                found=0
                                unit=getDeviceUnit(Devices,1) #1 means that it must scan all devices to find the right value of UnitFree and UnitMax
                                if (unit==0xffff):
                                    #port device not found: create it
                                    #portType is the numeric number provided by DOMBUS
                                    if (portType not in PORT_TYPENAME): portType=PORTTYPE_IN_DIGITAL   #default: Digital Input
                                    UnitAvailable=UnitMax if (UnitMax<=255) else UnitFree
                                    if (UnitAvailable>=256):
                                        Domoticz.Log("Maximum number of devices is reached! Domoticz supported max 256 devices for each protocol. Please remove unconnected devices!")
                                    else:
                                        descr=''
                                        for key, value in PORTTYPES.items():
                                            if (value==portType):
                                                descr=key+","
                                                break
                                        for key,value in PORTOPTS.items():
                                            if (value&portOpt):
                                                #descr=descr+key+","
                                                descr+=key+","
                                        if (descr!=''):
                                            descr=descr[:-1]    #remove last comma ,
                                        Domoticz.Debug("Add device "+deviceID+": "+portName+" Type="+str(portType)+" Opt="+str(portOpt)+" Cap="+str(portCapabilities)+ " Description="+descr)
                                        Domoticz.Device(Name="[H{:04x}] ".format(frameAddr)+portName, TypeName=PORT_TYPENAME[portType], Image=portImage, DeviceID=deviceID, Unit=UnitAvailable).Create()
                                        if (frameAddr<=0xff00 or port==1):
                                            Devices[UnitAvailable].Update(nValue=0, sValue='', Description=descr, Used=1)  # Add description (cannot be added with Create method)
                                     
                                port+=1;
                else:
                    #cmdAck==0 => decode command from slave module
                    if (frameAddr!=0xffff):
                        #Receive command from a slave module
                        getDeviceID(frameAddr,arg1)
                        if (cmd==CMD_SET):
                            #digital or analog input changed?
                            UnitFree=1
                            UnitMax=1
                            unit=getDeviceUnit(Devices,0)
                            if (unit==0xffff):
                                #got a frame from a unknown device: ask for configuration
                                txQueueAskConfig(frameAddr)
                            else:
                                #got a frame from a well known device
                                getDeviceID(frameAddr,arg1)
                                if (cmdLen==2):
                                    stringval='Off' if arg2==0 else 'On'
                                    #update device only if was changed
                                    if (Devices[unit].nValue!=int(arg2) or Devices[unit].sValue!=stringval):
                                        Devices[unit].Update(nValue=int(arg2), sValue=stringval)
                                    #send ack
                                    txQueueAdd(frameAddr,CMD_SET,2,CMD_ACK,arg1,[arg2],1,1)
                                elif (cmdLen==3):
                                    #analog value or distance
                                    value=arg2*256+arg3
                                    #extract A and B, if defined, to compute the right value VALUE=A*dombus_value+B
                                    opts=Devices[unit].Description.upper().split(',')
                                    a=1
                                    b=0
                                    for opt in opts:
                                        opt=opt.strip()
                                        if (opt[:2]=="A="):
                                            a=float(opt[2:])
                                        if (opt[:2]=="B="):
                                            b=float(opt[2:])
                                    value=a*value+b
#                                    if (Devices[unit].Type==243): 
#                                        if (Devices[unit].SubType==27):
#                                            #Distance
                                    Devices[unit].Update(nValue=int(value), sValue=str(value));
                                    txQueueAdd(frameAddr,CMD_SET,3,CMD_ACK,arg1,[arg2,arg3],1,1)
                frameIdx=frameIdx+cmdLen+1
            #remove current frame from buffer
            for i in range(0,frameLen):
                rxbuffer.pop(0)
        else:
            # checksum error
            Domoticz.Debug("Checksum error")
            rxbuffer.pop(0)
    else: 
        #frame len error: remove only the first byte corresponding to a false PREAMBLE
        if len(rxbuffer)<frameLen:
            Domoticz.Debug("decode: rxbuffer="+str(len(rxbuffer))+" < frameLen="+str(frameLen))
            return
        dump(rxbuffer,frameLen,"<<< RXbad")
        rxbuffer.pop(0)
        Domoticz.Debug("Frame length error:"+str(frameLen)+" while len(rxbuffer)="+str(len(rxbuffer)))

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
    for frameAddr,module in modules.items(): 
        timeFromLastRx=sec-module[LASTRX]             #number of seconds since last RXed frame
        timeFromLastTx=ms-module[LASTTX]                           #number of milliseconds since last TXed frame
        timeFromLastStatus=sec-module[LASTSTATUS]     #number of seconds since last TXed output status
        #check that module is active
        if (timeFromLastRx>MODULE_ALIVE_TIME):
            # too long time since last RX from this module: remove it from modules
            Domoticz.Debug("Remove module "+hex(frameAddr)+" because it's not alive")
            delmodules.append(frameAddr)
            # also remove any cmd in the txQueue
            Domoticz.Debug("Remove txQueue for "+hex(frameAddr))
            txQueueRemove(frameAddr,255,255)
        elif (len(txQueue[frameAddr])>0 and timeFromLastTx>TX_RETRY_TIME):
            #start txing
           tx=1
           txbuffer=bytearray()
           txbuffer.append(PREAMBLE_MASTER)
           txbuffer.append(frameAddr>>8)
           txbuffer.append(frameAddr&0xff)
           txbuffer.append(0)
           txbufferIndex=FRAME_HEADER
#           Domoticz.Debug("txQueue[frameAddr]="+str(txQueue[frameAddr]))
           for txq in txQueue[frameAddr][:]:    #iterate a copy of txQueue[frameAddr]
               #[cmd,cmdLen,cmdAck,port,[*args]]
               (cmd,cmdLen,cmdAck,port,args,retry)=txq
#               Domoticz.Debug("frameAddr="+hex(frameAddr)+" cmd="+hex(cmd|cmdAck|cmdLen)+" port="+hex(port)+" txbufferIndex="+str(txbufferIndex))
               txbuffer.append((cmd|cmdLen|cmdAck))
               txbufferIndex+=1
               txbuffer.append(port)
               txbufferIndex+=1
               for i in range(0,cmdLen-1):
                   txbuffer.append(args[i])
                   txbufferIndex+=1
               # if this cmd is an ACK, or values[0]==1, remove command from the queue
               if (cmdAck or retry<=1):
                   txQueue[frameAddr].remove(txq)
               else:
                   txq[5]=retry-1   #command, no ack: decrement retry
               if (txbufferIndex>FRAME_LEN_MAX-5): 
#                   Domoticz.Debug("txbufferIndex>FRAME_LEN_MAX-5: txbufferIndex="+str(txbufferIndex)+" > "+str(FRAME_LEN_MAX-5))
                   break   #frame must be truncated
           txbuffer[FRAME_LEN]=(txbufferIndex-FRAME_HEADER)
           checksum(txbuffer)
           txbuffer.append(checksumValue)
           txbufferIndex+=1
           SerialConn.Send(txbuffer)
           dump(txbuffer,txbufferIndex,">>> TX")
           modules[frameAddr][LASTTX]=ms
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
            txOutputsStatus(Devices, olderFrameAddr)
    return



