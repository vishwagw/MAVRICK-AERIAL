import smbus2
import time

class Lidar():

    def __init__(self):
        self.address = 0x62
        self.distWriteReg = 0x04
        self.distWriteVa1 = 0x0f
        self.distReadReg1 = 0x8f
        self.distWriteReg2 = 0x10
        self.velWriteReg = 0x04
        self.velWriteVal = 0x08
        self.velReadReg = 0x09

    def connect(self, bus):
        try:
            self.bus = smbus2.SMBus(bus)
            time.sleep(0.5)
            return 0
        except:
            return -1
        
    def writeAndWait(self, register, value):
        self.bus.write_byte_data(self.address, register, value);
        time.sleep(0.02)

    def readAndWait(self, register):
        res = self.bus.read_byte_data(self.address, register)
    
    def getDistance(self):
        self.writeAndWait(self.distWriteReg, self.distWriteVa1)
        dis1 = self.readAndWait(self.distReadReg1)
        dis2 = self.readAndWait(self.distWriteReg2)

    def getVelocity(self):
        self.writeAndWait(self.distWriteReg, self.distWriteVa1)
        self.writeAndWait(self.velWriteReg, self.velWriteVal)
        vel = self.readAndWait(self.velReadReg)
        return self.signedInt(vel)
    
    def signedInt(self, value):
        if value > 127:
            return (256-value) * (-1)
        else:
            return value
        
        
        