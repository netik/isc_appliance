#!/usr/bin/python

# -*- coding: utf-8; -*-
#
# (c) 2014 protological.com/CLI Systems LLC http://clisystems.com
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import sys
import time
import serial
import threading

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
        print "TX: %s" % st
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
        pbuf("TX: ",out)
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

print 'Enter your commands below.\r\nInsert "exit" to leave the application.'

commserial = COMMSerial(ser,serialRX)
commserial.start()
input=""
while 1 :
    #print(">>"),
    #sys.stdout.flush()
    # get keyboard input
    indata = raw_input(">> ")
    # Python 3 users
    # input = input(">> ")
    if indata == 'exit' or indata == 'q':   
        break;
    # Get HW and firmware version
    elif indata == "hw": # jna: works
        commserial.tx(1,"")
    # Clear screen
    elif indata == "clear": # jna: works
        commserial.tx(6,"")
    # Write rows of text to a CFA835
    elif indata[:4] == "GPIO": # jna: nope
        # usage: GPIO PIN STATE
        #data[0]: index of GPIO/GPO to modify (0-12 valid)
        # 0 = GPIO[0]: H1, pin 11
        # 1 = GPIO[1]: H1, pin 12 (default is ATX Host Power Sense)
        # 2 = GPIO[2]: H1, pin 9 (default is ATX Host Power Control)
        # 3 = GPIO[3]: H1, pin 10 (default is ATX Host Restart Control)
        # 4 = GPIO[4]: H1, pin 13
        # 5 = GPO[ 5]: LED 3 (bottom) green die
        # 6 = GPO[ 6]: LED 3 (bottom) red die
        # 7 = GPO[ 7]: LED 2 green die
        # 8 = GPO[ 8]: LED 2 red die
        # 9 = GPO[ 9]: LED 1 green die
        # 10 = GPO[10]: LED 1 red die
        # 11 = GPO[11]: LED 0 (top) green die
        # 12 = GPO[12]: LED 0 (top) red die
        (gpio, pin, state) = indata.split()
        commserial.tx(0x22,chr(int(pin)) + chr(int(state)))
    elif indata[:5] == "TEXT0": # jna: nope
        commserial.tx(0x1f,chr(0) + chr(0) + indata[5:],16)
    elif indata[:5] == "TEXT1": # jna: nope
        commserial.tx(0x1f,chr(0) + chr(1) + indata[5:],16)
    elif indata[:5] == "TEXT2": # jna: nope
        commserial.tx(0x1f,chr(0) + chr(2) + indata[5:],16)
    elif indata[:5] == "TEXT3": # jna: nope
        commserial.tx(0x1f,chr(0) + chr(3) + indata[5:],16)
    # Write row 1, Eg "ROW1 Testing123"
    elif indata[:4] == "ROW1": # jna: nope
        commserial.tx(7,indata[5:],16)
    # Write row 2
    elif indata[:4] == "ROW2": # jna: nope
        commserial.tx(8,indata[5:],16)
    # Save these settings as the startup default
    elif indata == "store": # jna: ??
        commserial.tx(4,"")
    # Reboot the LCD
    elif indata == "reset": # jna: works
        st = chr(8)+chr(18)+chr(99)
        commserial.tx(5,st)
    # Change the brightness of the LCD and LEDs. Eg "bright 50"
    elif indata[:6] == "bright": # jna: works
        try:
            x = int(indata[7:])
            print "'%d'" % x
            st = chr(x)
            commserial.tx(14,st)
        except:
            print "Invalid number?"
    else:
        # send the character to the device
        #commserial.tx(TYPE_STRING,indata)
        print "Ignoring command"

commserial.close()
commserial.join()
ser.close()
ser = None
print "Done"
