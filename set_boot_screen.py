#!/usr/bin/python
#
# install a boot screen into a CF835 
#

import sys
import time
import serial
import threading
from isc_logo import logo_width,logo_height,logo_bytes

STATE_IDLE = 0
STATE_LEN = 1
STATE_DATA = 2
STATE_CRC1 = 3
STATE_CRC2 = 4

TYPE_STRING = 1
TYPE_DATA = 2

# Classes
###############################
class COMMPacket:

    type = 0
    length = 0
    data = []
    incrc = 0
    def __init__(self,intype):
        self.type = intype
        self.clear()

    def clear(self):
        self.length = 0
        self.data = []

    def dataadd(self,x):
        self.data.append(x)

    def getdatacount(self):
        return len(self.data)

    def getlength(self):
        return self.length

    def getdatastring(self):
        st = ""
        for x in self.data:
            try:
                st += chr(x)
            except:
                st += x
        return st

class COMMSerial(threading.Thread):

    remainderstr = ""
    rawstr = ""

    def __init__(self,ser,rxcallback):
        super(COMMSerial, self).__init__()
        self.key = 0xAB
        self.packet = None
        self.state = STATE_IDLE

        self.ser = ser
        self.rxcallback = rxcallback;
        self.running = True

    def run(self):
        while(self.running):
            if self.ser.inWaiting:
                out = ''
                while self.ser.inWaiting() > 0:
                    out += self.ser.read(1)
                if(len(out)>0):
                    pkt = commserial.inputstr(out)
                    if(pkt!=None and self.rxcallback!=None):
                        self.rxcallback(pkt)

    def close(self):
        self.running = False

    def crc16(self,st):
        crc = 0xFFFF
        for c in st:
            x = ord(c)
            for y in range(8):
                if((crc ^ x) & 0x01):
                    crc >>= 1;
                    crc ^= 0x8408;
                else:
                    crc >>= 1;
                x >>= 1;

        return ((~crc)&0xFFFF)

    def inputstr(self, st):
        #print "  RX %d bytes" % len(st)
        for s in st:
            x = ord(s)
            # calc CRC
            if(self.state<STATE_CRC1):
                self.rawstr += chr(x)

            # Process data
            if(self.state==STATE_IDLE):
                self.state = STATE_LEN;
                self.packet = COMMPacket(x)
                #print "State idle done, type %d" % x
            elif(self.state==STATE_LEN):
                self.packet.length = x
                if(x==0):
                    self.state = STATE_CRC1
                else:
                    self.state=STATE_DATA
                #print "State len done, %d" % self.packet.length
            elif(self.state==STATE_DATA):
                self.packet.dataadd(x);
                #print "State dataadd %02X, count %d" % (x,self.packet.getdatacount());
                if(self.packet.getdatacount()==self.packet.getlength()):
                    self.state = STATE_CRC1

            elif(self.state==STATE_CRC1):
                self.packet.incrc = x;
                self.state = STATE_CRC2
                #print "CRC1 x%02X" % x
            elif(self.state==STATE_CRC2):
                self.packet.incrc += x<<8
                crc = self.crc16(self.rawstr)
                if(crc != self.packet.incrc):
                    print "Error in CRC x%04X != x%04X" % (self.packet.incrc&0xFFFF,crc)
                self.state = STATE_IDLE
                pk = self.packet
                self.packet = None
                self.rawstr = ""
                return pk
                # TODO: Save the remainder of the packet incase there is more

        #stlen = len(st)
        #print "Got %d bytes" % stlen

        return None

    def tx(self,type,st,padto=0):
        # print "TX: %s" % st
        strlen = len(st)
        out = ""
        out += chr(type);
        if(strlen>16): strlen=16
        if(padto>strlen):
            out += chr(padto)
            for ch in st[:padto]:
                out += ch
        else:
            out += chr(strlen)
            for ch in st[:strlen]:
                out += ch
        if(padto>strlen):
            cnt = padto-strlen
            for ch in range(cnt):
                out += " "

        crc = self.crc16(out)
        # Append CRC
        out += chr(crc&0xFF)
        out += chr((crc>>8)&0xFF)
        #pbuf("TX: ",out)
        self.ser.write(out);

# Functions
###############################

def pbuf(st,b):
    for x in b:
        try:
            st += "x%02X," % ord(x)
        except:
            st += "x%02X," % x
    print st

def serialRX(pkt):
    #pbuf(out)
    if(pkt.type==0x80):
        key = pkt.data[0]
        #print "Key: x%02X" % key
        if(key==1): print "Up"
        elif(key==2): print "Down"
        elif(key==3): print "Left"
        elif(key==4): print "Right"
        elif(key==5): print "Enter"
        elif(key==6): print "Exit"
        else: return;
    else:
        print "Got packet!! type x%02X '%s'" % (pkt.type, pkt.getdatastring())
        #pbuf("Data: ", pkt.data)
    print(">> "),
    sys.stdout.flush()

# Main
###############################


ser_port = "/dev/cu.usbmodem41431"
ser_speed = 19200

# configure the serial connections (the parameters differs on the device you are connecting to)
try:
    ser = serial.Serial(
        port=ser_port,
        baudrate=ser_speed,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
    )
except:
    print "Can't setup port %s at %d" % (ser_port,ser_speed)
    raise
    exit()
    
try:
    if(ser.isOpen()==False):
        ser.open()
    if(ser.isOpen()==False):
        print "Error testing if port is open"
        exit();
except:
    print "Can't open port %s at %d" % (ser_port,ser_speed)
    raise
    exit()

commserial = COMMSerial(ser,serialRX)
commserial.start()
input=""

# clear the display
commserial.tx(6,"")

# write four lines
#indata="hello hello hello"
#commserial.tx(0x1f,chr(0) + chr(0) + indata[5:],16)
#commserial.tx(0x1f,chr(0) + chr(1) + indata[5:],16)
#commserial.tx(0x1f,chr(0) + chr(2) + indata[5:],16)
#commserial.tx(0x1f,chr(0) + chr(3) + indata[5:],16)

# put an image to the display
y=0
x=0

print logo_width
print logo_height

# note that colors have to be inverted, shade is backwards on this display! 
while (y < logo_height):
    x=0
    while (x < logo_width):
        print "%d %d %c" % (x, y, logo_bytes[(y*logo_width) + x] & 0x0000ff)
        commserial.tx(0x28,chr(5) + chr(x) + chr(y) + chr(255 - (logo_bytes[(y*logo_width) + x] & 0x0000ff)))
        x = x + 1
    y = y + 1

# make this state the boot state.
commserial.tx(0x04,"");

commserial.close()
commserial.join()
ser.close()
ser = None
exit()
