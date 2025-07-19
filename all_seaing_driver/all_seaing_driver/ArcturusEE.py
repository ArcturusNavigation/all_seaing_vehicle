import struct

class BMS:
    def __init__(self, ser, id):
        """
        BMS constructor.

        Args:
            ser (serial.Serial): Pyserial interface.
            id (int): 0x02 for EE BMS, 0x03 for THR A BMS, 0x04 for THR B BMS.
        """
        self.ser = ser
        self.id = id

    def cell_voltage(self, cell):
        """
        Cell voltage (mV).

        Args:
            Cell (int): 1-4 for THR BMS, 1-6 for EE BMS.
    
        Returns:
            int: Cell voltage in mV.
        """
        self.ser.write((bytes([self.id, cell - 1])))
        return struct.unpack("<h", self.ser.read(2))[0]
    
    def stack_voltage(self):
        """
        Stack voltage (cV).
    
        Returns:
            int: Stack voltage in cV.
        """
        self.ser.write((bytes([self.id, 0x06])))
        return struct.unpack("<h", self.ser.read(2))[0]
    
    def current(self):
        """
        Current (mA).
    
        Returns:
            int: Current (mA).
        """ 
        self.ser.write((bytes([self.id, 0x07])))
        return struct.unpack("<h", self.ser.read(2))[0]
    
    def temp(self):
        """
        Temperature (deg C).
    
        Returns:
            int: Temperature (deg C).
        """
        self.ser.write((bytes([self.id, 0x08])))
        return round(struct.unpack("<f", self.ser.read(4))[0], 2)
    
    def output(self):
        """
        Status of BMS output.
    
        Returns:
            bool: True if output on.
        """
        self.ser.write((bytes([self.id, 0x09])))
        return self.ser.read(1)[0]
    
    def estop(self):
        """
        Status of ESTOP.
    
        Returns:
            bool: True if ESTOP on.
        """
        self.ser.write((bytes([self.id, 0x0A])))
        return self.ser.read(1)[0]
    
class main_power:
    def __init__(self, ser):
        """
        Main power constructor.

        Args:
            ser (serial.Serial): Pyserial interface.
        """
        self.ser = ser
        self.id = 0x01

    def wait(self):
        """
        Helper function, waits for EE system to respond before sending another command.
        """
        self.ser.read()

    def current_3v3(self):
        """
        3.3V current (A).

        Returns:
            float: current in A.
        """
        self.ser.write((bytes([self.id, 0x00])))
        return struct.unpack("<f", self.ser.read(4))[0]
    
    def current_5v(self):
        """
        5V current (A).

        Returns:
            float: current in A.
        """
        self.ser.write((bytes([self.id, 0x01])))
        return struct.unpack("<f", self.ser.read(4))[0]

    def current_12v(self):
        """
        12V current (A).

        Returns:
            float: current in A.
        """
        self.ser.write((bytes([self.id, 0x02])))
        return struct.unpack("<f", self.ser.read(4))[0]

    def current_19v(self):
        """
        19V current (A).

        Returns:
            float: current in A.
        """
        self.ser.write((bytes([self.id, 0x03])))
        return struct.unpack("<f", self.ser.read(4))[0]

    def estop(self):
        """
        Status of ESTOP.
    
        Returns:
            bool: True if ESTOP on.
        """
        self.ser.write((bytes([self.id, 0x04])))
        return self.ser.read(1)[0]

    def fan1(self, value):
        """
        Control fan 1.
    
        Args:
            value (bool): True to turn on fan 1, False to turn off.
        """
        self.ser.write((bytes([(1 << 7) + self.id, 0x05, value])))
        self.wait()

    def fan2(self, value):
        """
        Control fan 2.
    
        Args:
            value (bool): True to turn on fan 2, False to turn off.
        """
        self.ser.write((bytes([(1 << 7) + self.id, 0x06, value])))
        self.wait()

class ESTOP:
    def __init__(self, ser):
        """
        ESTOP constructor.

        Args:
            ser (serial.Serial): Pyserial interface.
        """
        self.ser = ser
        self.id = 0x00
    
    def remote_estop(self):
        """
        Status of remote ESTOP.
    
        Returns:
            bool: True if remote ESTOP on.
        """
        self.ser.write((bytes([self.id, 0x00])))
        return self.ser.read(1)[0]
    
    def manual(self):
        """
        Drive mode.
    
        Returns:
            bool: True if device is in manual drive mode.
        """
        self.ser.write((bytes([self.id, 0x01])))
        return self.ser.read(1)[0]
    
    def drive_x1(self):
        """
        Joystick 1 X value.
    
        Returns:
            float: Joystick 1 X value range -1 to 1.
        """
        self.ser.write((bytes([self.id, 0x02])))
        val = self.ser.read(1)[0]
        ret = (val - 127.0) / 127.0
        if ret != 0:
            ret *= -1
        return ret
    
    def drive_y1(self):
        """
        Joystick 1 Y value.
    
        Returns:
            float: Joystick 1 Y value range -1 to 1.
        """
        self.ser.write((bytes([self.id, 0x03])))
        val = self.ser.read(1)[0]
        return (val - 127.0) / 127.0
    
    def drive_x2(self):
        """
        Joystick 2 X value.
    
        Returns:
            float: Joystick 2 X value range -1 to 1.
        """
        self.ser.write((bytes([self.id, 0x04])))
        val = self.ser.read(1)[0]
        ret = (val - 127.0) / 127.0
        if ret != 0:
            ret *= -1
        return ret
    
    def drive_y2(self):
        """
        Joystick 2 Y value.
    
        Returns:
            float: Joystick 2 Y value range -1 to 1.
        """
        self.ser.write((bytes([self.id, 0x05])))
        val = self.ser.read(1)[0]
        return (val - 127.0) / 127.0
    
    def connected(self):
        """
        Status of connection.
        
        Returns:
            bool: True if connected.
        """
        self.ser.write((bytes([self.id, 0x06])))
        return self.ser.read(1)[0]
    
class mech_power:
    def __init__(self, ser, id):
        """
        Mechanisms power constructor.

        Args:
            ser (serial.Serial): Pyserial interface.
            id (int): 0x05 for Mech PWR A, 0x06 for Mech PWR B.
        """
        self.ser = ser
        self.id = id

    def wait(self):
        """
        Helper function, waits for EE system to respond before sending another command.
        """
        self.ser.read()

    def estop(self):
        """
        Status of ESTOP.
    
        Returns:
            bool: True if ESTOP on.
        """
        self.ser.write((bytes([self.id, 0x00])))
        return self.ser.read(1)[0]
    
    def status(self):
        """
        Status of converter (See Page 71 of LM251772 Datasheet).
    
        Returns:
            byte: Status.
        """
        self.ser.write((bytes([self.id, 0x01])))
        return self.ser.read(1)[0]
    
    def current(self):
        """
        Current (A).
    
        Returns:
            float: Current (A).
        """ 
        self.ser.write((bytes([self.id, 0x02])))
        return struct.unpack("<f", self.ser.read(4))[0]
    
    def output(self, value):
        """
        Turn on/off converter output.

        Args:
            value (bool): True to turn on, False to turn off.
        """
        self.ser.write((bytes([(1 << 7) + self.id, 0x03, value])))
        self.wait()
        
    def set_voltage(self, value):
        """
        Set voltage in volts.

        Args:
            value (float): Voltage in volts.
        """
        self.ser.write((bytes([(1 << 7) + self.id, 0x04])))
        self.ser.write(struct.pack("<f", value))
        self.wait()
