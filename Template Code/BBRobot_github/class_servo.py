import serial

class sPort:
    def __init__(self):
        
        self.ser_rp4 = serial.Serial('/dev/ttyAMA1', 115200, timeout = None)
    
    
    def send_packet(self, packet):
        self.ser_rp4.write(packet)

    
    def close_sPort(self):
        self.ser_rp4.close()

class RS304MD:
    def __init__(self, id):
        self.id = id
        self.port = sPort()
    
    def trq_set(self, status):
        id = "00"+str(self.id)
        num = "00"+str(status)
        packet = ['0xFA', '0xAF', '0x' + id[-2:], '0x00', '0x24', '0x01', '0x01', '0x'+num[-2:]] 
        sum = 0 
        for num in range(2, len(packet)):
            sum ^= int(packet[num], 0)
        packet.append(hex(sum))
        packet = [int(x, 16) for x in packet]
        self.port.send_packet(packet)

    
    def control_time_rotate(self, angle, t):
        a = -angle*10
        if a<0:
            a = 65536+a
        a = ("0000"+format(int(a), 'x'))[-4:]
        an = []
        an.append('0x'+a[-2:])
        an.append('0x'+a[:2])
        
        t = ("0000"+format(int(t*100), 'x'))[-4:]
        tn = []
        tn.append('0x'+t[-2:])
        tn.append('0x'+t[:2])
        
        ID = "00"+str(self.id)
        packet = ['0xFA', '0xAF', '0x' + ID[-2:], '0x00', '0x1E', '0x04', '0x01'] + an + tn 
        sum = 0 
        for num in range(2, len(packet)):
            sum ^= int(packet[num], 0)
        packet.append(hex(sum))
        packet = [int(x, 16) for x in packet]
        self.port.send_packet(packet)

    
    def control_rotate(self, angle):
        a = -angle*10
        if a<0:
            a = 65536+a
        a = ("0000"+format(int(a), 'x'))[-4:]
        an = []
        an.append('0x'+a[-2:])
        an.append('0x'+a[:2])
        
        ID = "00"+str(self.id)
        packet = ['0xFA', '0xAF', '0x' + ID[-2:], '0x00', '0x1E', '0x02', '0x01'] + an 
        sum = 0 
        for num in range(2, len(packet)):
            sum ^= int(packet[num], 0)
        packet.append(hex(sum))
        packet = [int(x, 16) for x in packet]
        self.port.send_packet(packet)

    
    def change_id(self, new_id):
        
        old_id = "00"+str(self.id)
        new_id = "00"+str(new_id)
        packet = ['0xFA', '0xAF', '0x' + old_id[-2:], '0x00', '0x04', '0x01', '0x01', '0x'+new_id[-2:]] 
        sum = 0 
        for num in range(2, len(packet)):
            sum ^= int(packet[num], 0)
        packet.append(hex(sum))
        packet = [int(x, 16) for x in packet]
        self.port.send_packet(packet)

        
        packet = ['0xFA', '0xAF', '0x' + new_id[-2:], '0x40', '0xff', '0x00', '0x00'] 
        sum = 0 
        for num in range(2, len(packet)):
            sum ^= int(packet[num], 0)
        packet.append(hex(sum))
        packet = [int(x, 16) for x in packet]
        self.port.send_packet(packet)

        
        packet = ['0xFA', '0xAF', '0x' + new_id[-2:], '0x20', '0xff', '0x00', '0x00'] 
        sum = 0 
        for num in range(2, len(packet)):
            sum ^= int(packet[num], 0)
        packet.append(hex(sum))
        packet = [int(x, 16) for x in packet]
        self.port.send_packet(packet)