# Import serial library for UART communication
import serial


class sPort:
    def __init__(self):
        """
        Serial Port Initialization
        
        This sets up UART communication with the servo controller.
        """

        # Open serial port
        # '/dev/ttyAMA1' → Raspberry Pi UART port (can change depending on setup)
        # 115200 → baud rate (MUST match servo specification)
        # timeout=None → blocking read (waits forever for data)
        #
        # 🔧 YOU MAY NEED TO CHANGE:
        # - Port name if using USB (e.g., '/dev/ttyUSB0')
        # - Baud rate if your servo uses a different one
        self.ser_rp4 = serial.Serial('/dev/ttyAMA1', 115200, timeout=None)
    
    
    def send_packet(self, packet):
        """
        Sends a packet (list of bytes) over serial.
        
        packet: list of integers (0–255)
        """
        self.ser_rp4.write(packet)

    
    def close_sPort(self):
        """
        Closes the serial port safely.
        Always call this when exiting your program.
        """
        self.ser_rp4.close()


class RS304MD:
    def __init__(self, id):
        """
        Servo Initialization
        
        id: Servo ID (usually 1–127 depending on hardware)
        """
        self.id = id

        # Create a serial port instance
        # NOTE: Each object creates its own port (could be optimized to share one)
        self.port = sPort()
    
    def trq_set(self, status):
        """
        Enable or disable servo torque
        
        status:
            1 → Torque ON (motor holds position)
            0 → Torque OFF (motor is free)
        """

        # Format ID and status as 2-digit hex strings
        id = "00" + str(self.id)
        num = "00" + str(status)

        # Build packet according to servo protocol
        # Structure:
        # [Header, Header, ID, Flags, Address, Length, Count, Data, Checksum]
        packet = [
            '0xFA', '0xAF',               # Header bytes (fixed)
            '0x' + id[-2:],              # Servo ID
            '0x00',                      # Flags
            '0x24',                      # Address (torque control)
            '0x01',                      # Data length
            '0x01',                      # Number of servos
            '0x' + num[-2:]              # Torque value (0 or 1)
        ]

        # Compute checksum using XOR of bytes (from index 2 onward)
        sum = 0
        for num in range(2, len(packet)):
            sum ^= int(packet[num], 0)

        # Append checksum
        packet.append(hex(sum))

        # Convert hex strings to integers
        packet = [int(x, 16) for x in packet]

        # Send packet
        self.port.send_packet(packet)

    
    def control_time_rotate(self, angle, t):
        """
        Rotate servo to a specific angle over a given time
        
        angle: desired angle (degrees)
        t: time (seconds)
        
        NOTE:
        - Angle is multiplied by 10 (servo uses 0.1° units)
        - Negative sign may depend on motor orientation
        """

        # Convert angle to servo units (0.1 degrees)
        a = -angle * 10

        # Convert to unsigned 16-bit if negative
        if a < 0:
            a = 65536 + a

        # Convert to 4-digit hex string
        a = ("0000" + format(int(a), 'x'))[-4:]

        # Split into low and high bytes
        an = []
        an.append('0x' + a[-2:])   # Low byte
        an.append('0x' + a[:2])    # High byte
        
        # Convert time to servo units (centiseconds = t * 100)
        t = ("0000" + format(int(t * 100), 'x'))[-4:]

        # Split time into bytes
        tn = []
        tn.append('0x' + t[-2:])
        tn.append('0x' + t[:2])
        
        # Format ID
        ID = "00" + str(self.id)

        # Build packet
        packet = [
            '0xFA', '0xAF',
            '0x' + ID[-2:],
            '0x00',
            '0x1E',   # Address for position + time control
            '0x04',   # Data length (angle + time)
            '0x01'
        ] + an + tn 

        # Compute checksum
        sum = 0
        for num in range(2, len(packet)):
            sum ^= int(packet[num], 0)

        packet.append(hex(sum))

        # Convert to integers
        packet = [int(x, 16) for x in packet]

        # Send packet
        self.port.send_packet(packet)

    
    def control_rotate(self, angle):
        """
        Rotate servo to a specific angle (no time control)
        
        angle: desired angle (degrees)
        """

        # Convert angle to servo units
        a = -angle * 10

        # Handle negative values (convert to unsigned)
        if a < 0:
            a = 65536 + a

        # Convert to hex
        a = ("0000" + format(int(a), 'x'))[-4:]

        # Split into bytes
        an = []
        an.append('0x' + a[-2:])
        an.append('0x' + a[:2])
        
        # Format ID
        ID = "00" + str(self.id)

        # Build packet
        packet = [
            '0xFA', '0xAF',
            '0x' + ID[-2:],
            '0x00',
            '0x1E',   # Position control address
            '0x02',   # Data length (angle only)
            '0x01'
        ] + an 

        # Compute checksum
        sum = 0
        for num in range(2, len(packet)):
            sum ^= int(packet[num], 0)

        packet.append(hex(sum))

        # Convert to integers
        packet = [int(x, 16) for x in packet]

        # Send packet
        self.port.send_packet(packet)

    
    def change_id(self, new_id):
        """
        Change the servo ID
        
        WARNING:
        - Make sure only ONE servo is connected when doing this
        - Otherwise, all servos may get the same ID
        """

        old_id = "00" + str(self.id)
        new_id = "00" + str(new_id)

        # -------- Step 1: Write new ID --------
        packet = [
            '0xFA', '0xAF',
            '0x' + old_id[-2:],
            '0x00',
            '0x04',   # Address for ID
            '0x01',
            '0x01',
            '0x' + new_id[-2:]
        ]

        sum = 0
        for num in range(2, len(packet)):
            sum ^= int(packet[num], 0)

        packet.append(hex(sum))
        packet = [int(x, 16) for x in packet]
        self.port.send_packet(packet)

        
        # -------- Step 2: Save to ROM --------
        packet = [
            '0xFA', '0xAF',
            '0x' + new_id[-2:],
            '0x40',
            '0xff',
            '0x00',
            '0x00'
        ]

        sum = 0
        for num in range(2, len(packet)):
            sum ^= int(packet[num], 0)

        packet.append(hex(sum))
        packet = [int(x, 16) for x in packet]
        self.port.send_packet(packet)

        
        # -------- Step 3: Reboot servo --------
        packet = [
            '0xFA', '0xAF',
            '0x' + new_id[-2:],
            '0x20',
            '0xff',
            '0x00',
            '0x00'
        ]

        sum = 0
        for num in range(2, len(packet)):
            sum ^= int(packet[num], 0)

        packet.append(hex(sum))
        packet = [int(x, 16) for x in packet]
        self.port.send_packet(packet)