import struct

class LED:
    def __init__(self, ser):
        self.adr = 0x11
        self.ser = ser #pyserial interface

    def wait(self):
        """
        Helper function, waits for central hub to respond before sending another command
        """
        self.ser.read()

    def setMPL(self, led, value):
        """
        Turn on/off one of the 3 multipurpose LEDs on central hub PCB

        Args:
            led (int): led number, 1/2/3
            value (int): 0 for off, 1 for on
        """
        self.ser.write(bytes([(self.adr << 1) + 1, led, value]))
        self.wait()

    def fillStrip(self, r, g, b):
        """
        Fills the LED strip a given color

        Args:
            r (int): Red (0-255)
            g (int): Green (0-255)
            b (int): Blue (0-255)
        """
        self.ser.write(bytes([(self.adr << 1) + 1, 0x04, r, g, b]))
        self.wait()

    def idvClr(self, px, r, g, b):
        """
        Sets a pixel to a given color

        Args:
            px (int): Pixel (1-144)
            r (int): Red (0-255)
            g (int): Green (0-255)
            b (int): Blue (0-255)
        """
        self.ser.write(bytes([(self.adr << 1) + 1, 0x05, px - 1, r, g, b]))
        self.wait()

    def clrStrip(self):
        """
        Clears strip
        """
        self.ser.write(bytes([(self.adr << 1) + 1, 0x06]))
        self.wait()

class BMS:
    def __init__(self, ser, adr):
        self.ser = ser #pyserial interface
        self.adr = adr #0x08 for thruster battery #1, 0x09 for #2, 0x0A for EE battery

    def wait(self):
        """
        Helper function, waits for central hub to respond before sending another command
        """
        self.ser.read()

    def output(self, value):
        """
        Turn on/off the output of BMS

        WARNING: TURNING OFF THE OUTPUT OF EE BMS WILL PERMANENTLY DISABLE ALL ELECTRONICS

        WARNING: TURNING ON THRUSTER BATTERY BMS OUTPUT WILL OVERRIDE ESTOP

        Args:
            value (int): 0 for off, 1 for on
        """
        self.ser.write(bytes([(self.adr << 1) + 1, 0x01, value]))
        self.wait()

    def voltage(self):
        """
        Battery voltage

        Returns:
            float: battery voltage
        """
        self.ser.write((bytes([self.adr << 1, 4, 0x02])))
        return struct.unpack("<f", self.ser.read(4))

    def current(self):
        """
        Current

        Returns:
            float: current
        """
        self.ser.write((bytes([self.adr << 1, 4, 0x03])))
        return struct.unpack("<f", self.ser.read(4))

    def cell1(self):
        """
        Cell 1 voltage

        Returns:
            float: cell 1 voltage
        """
        self.ser.write((bytes([self.adr << 1, 4, 0x04])))
        return struct.unpack("<f", self.ser.read(4))

    def cell2(self):
        """
        Cell 2 voltage

        Returns:
            float: cell 2 voltage
        """
        self.ser.write((bytes([self.adr << 1, 4, 0x05])))
        return struct.unpack("<f", self.ser.read(4))

    def cell3(self):
        """
        Cell 3 voltage

        Returns:
            float: cell 3 voltage
        """
        self.ser.write((bytes([self.adr << 1, 4, 0x06])))
        return struct.unpack("<f", self.ser.read(4))

    def cell4(self):
        """
        Cell 4 voltage

        Returns:
            float: cell 4 voltage
        """
        self.ser.write((bytes([self.adr << 1, 4, 0x07])))
        return struct.unpack("<f", self.ser.read(4))

    def cell5(self):
        """
        Cell 5 voltage -- EE BMS only

        Returns:
            float: cell 5 voltage
        """
        self.ser.write((bytes([self.adr << 1, 4, 0x08])))
        return struct.unpack("<f", self.ser.read(4))

    def cell6(self):
        """
        Cell 6 voltage -- EE BMS only

        Returns:
            float: cell 6 voltage
        """
        self.ser.write((bytes([self.adr << 1, 4, 0x09])))
        return struct.unpack("<f", self.ser.read(4))

    def temp(self):
        """
        Temperature - Thruster BMS only

        Returns:
            float: temperature (deg F)
        """
        self.ser.write((bytes([self.adr << 1, 4, 0x0A])))
        return struct.unpack("<f", self.ser.read(4))

class ESTOP:
    def __init__(self, ser):
        self.ser = ser #pyserial interface
        self.adr = 0x12

    def wait(self):
        """
        Helper function, waits for central hub to respond before sending another command
        """
        self.ser.read()

    def reset(self):
        """
        Resets LoRa module
        """
        self.ser.write(bytes([(self.adr << 1) + 1, 0x08]))
        self.wait()

    def estop(self):
        """
        Is ESTOP activated?

        Returns:
            int: 1 if ESTOP is activated, 0 otherwise
        """
        self.ser.write((bytes([self.adr << 1, 1, 0x01])))
        return self.ser.read(1)[0]

    def mode(self):
        """
        Drive Mode

        Returns:
            int: 0 for auto, 1 for manual
        """
        self.ser.write((bytes([self.adr << 1, 1, 0x02])))
        return self.ser.read(1)[0]

    def drive_x(self):
        """
        Joystick X-direction

        Returns:
            float: Joystick X-direction between -1 and 1
        """
        self.ser.write((bytes([self.adr << 1, 4, 0x03])))
        return struct.unpack("<f", self.ser.read(4))

    def drive_y(self):
        """
        Joystick Y-direction

        Returns:
            float: Joystick Y-direction between -1 and 1
        """
        self.ser.write((bytes([self.adr << 1, 4, 0x04])))
        return struct.unpack("<f", self.ser.read(4))

    def connected(self):
        """
        Is controller connected?

        Returns:
            int: 1 if connected, 0 otherwise
        """
        self.ser.write((bytes([self.adr << 1, 1, 0x05])))
        return self.ser.read(1)[0]

    def rssi(self):
        """
        Received Signal Strength Indicator

        Returns:
            int: RSSI
        """
        self.ser.write((bytes([self.adr << 1, 2, 0x06])))
        return struct.unpack("<h", self.ser.read(2))

    def snr(self):
        """
        Signal to Noise Ratio

        Returns:
            int: snr
        """
        self.ser.write((bytes([self.adr << 1, 2, 0x07])))
        return struct.unpack("<h", self.ser.read(2))

class Buck:
    def __init__(self, ser):
        self.ser = ser #pyserial interface
        self.adr = 0x13

    def wait(self):
        """
        Helper function, waits for central hub to respond before sending another command
        """
        self.ser.read()

    def adj1_en(self, value):
        """
        Turn on/off adjustable converter #1

        Args:
            value (int): 0 for off, 1 for on
        """
        self.ser.write(bytes([(self.adr << 1) + 1, 0x06, value]))
        self.wait()

    def adj2_en(self, value):
        """
        Turn on/off adjustable converter #2

        Args:
            value (int): 0 for off, 1 for on
        """
        self.ser.write(bytes([(self.adr << 1) + 1, 0x07, value]))
        self.wait()

    def fan1(self, value):
        """
        Turn on/off fan #1

        Args:
            value (int): 0 for off, 1 for on
        """
        self.ser.write(bytes([(self.adr << 1) + 1, 0x0A, value]))
        self.wait()

    def fan2(self, value):
        """
        Turn on/off fan #2

        Args:
            value (int): 0 for off, 1 for on
        """
        self.ser.write(bytes([(self.adr << 1) + 1, 0x0B, value]))
        self.wait()

    def adj1_voltage(self, value):
        """
        Set voltage on adjustable converter #1

        Args:
            value (float): voltage between .8 and 20
        """
        self.ser.write(bytes([(self.adr << 1) + 1, 0x08]))
        self.ser.write(struct.pack("<f", value))
        self.wait()

    def adj2_voltage(self, value):
        """
        Set voltage on adjustable converter #2

        Args:
            value (float): voltage between .8 and 20
        """
        self.ser.write(bytes([(self.adr << 1) + 1, 0x09]))
        self.ser.write(struct.pack("<f", value))
        self.wait()

    def current_5v(self):
        """
        Current on 5V output

        Returns:
            float: current (amps)
        """
        self.ser.write((bytes([self.adr << 1, 4, 0x01])))
        return struct.unpack("<f", self.ser.read(4))

    def current_12v(self):
        """
        Current on 12V output

        Returns:
            float: current (amps)
        """
        self.ser.write((bytes([self.adr << 1, 4, 0x02])))
        return struct.unpack("<f", self.ser.read(4))

    def current_19v(self):
        """
        Current on 19V output

        Returns:
            float: current (amps)
        """
        self.ser.write((bytes([self.adr << 1, 4, 0x03])))
        return struct.unpack("<f", self.ser.read(4))

    def current_adj1(self):
        """
        Current on adjustable converter #1 output

        Returns:
            float: current (amps)
        """
        self.ser.write((bytes([self.adr << 1, 4, 0x04])))
        return struct.unpack("<f", self.ser.read(4))

    def current_adj2(self):
        """
        Current on adjustable converter #2 output

        Returns:
            float: current (amps)
        """
        self.ser.write((bytes([self.adr << 1, 4, 0x05])))
        return struct.unpack("<f", self.ser.read(4))

class Mechanisms:
    def __init__(self, ser):
        self.ser = ser #pyserial interface
        self.adr = 0x0B

    def wait(self):
        """
        Helper function, waits for central hub to respond before sending another command
        """
        self.ser.read()

    def reset_launched(self):
        """
        Reset the balls launched count
        """
        self.ser.write((bytes([(self.adr << 1) + 1, 0x01])))
        self.wait()

    def set12V(self, value):
        """
        Set the 12V output

        Args:
            value (byte): PWM duty cycle, 0 for off, 255 for on, n for (n/255)% duty cycle
        """
        self.ser.write((bytes([(self.adr << 1) + 1, 0x02, value])))
        self.wait()

    def set20V(self, value):
        """
        Set the 20V output

        Args:
            value (byte): PWM duty cycle, 0 for off, 255 for on, n for (n/255)% duty cycle
        """
        self.ser.write((bytes([(self.adr << 1) + 1, 0x03, value])))
        self.wait()

    def stop_servo1(self):
        """
        Stop/detach servo 1
        """
        self.ser.write((bytes([(self.adr << 1) + 1, 0x04])))
        self.wait()

    def stop_servo2(self):
        """
        Stop/detach servo 2
        """
        self.ser.write((bytes([(self.adr << 1) + 1, 0x05])))
        self.wait()

    def servo1_angle(self, value):
        """
        Writes a value to servo1, controlling the shaft accordingly.
        On a standard servo, this will set the angle of the shaft (in degrees), moving the shaft to that orientation.
        On a continuous rotation servo, this will set the speed of the servo (with 0 being full-speed in one direction,
        180 being full speed in the other, and a value near 90 being no movement). Source: Arduino Docs

        Args:
            value: angle (degrees)
        """
        self.ser.write((bytes([(self.adr << 1) + 1, 0x06, value])))
        self.wait()

    def servo2_angle(self, value):
        """
        Writes a value to servo2, controlling the shaft accordingly.
        On a standard servo, this will set the angle of the shaft (in degrees), moving the shaft to that orientation.
        On a continuous rotation servo, this will set the speed of the servo (with 0 being full-speed in one direction,
        180 being full speed in the other, and a value near 90 being no movement). Source: Arduino Docs

        Args:
            value: angle (degrees)
        """
        self.ser.write((bytes([(self.adr << 1) + 1, 0x07, value])))
        self.wait()

    def launched(self):
        """
        Get how many balls have been launched

        Returns:
            int: balls launched
        """
        self.ser.write((bytes([self.adr << 1, 1, 0x08])))
        return self.ser.read(1)[0]

    def switch1(self):
        """
        Switch 1 status

        Returns:
            int: 1 if open, 0 if closed
        """
        self.ser.write((bytes([self.adr << 1, 1, 0x09])))
        return self.ser.read(1)[0]

    def switch2(self):
        """
        Switch 2 status

        Returns:
            int: 1 if open, 0 if closed
        """
        self.ser.write((bytes([self.adr << 1, 1, 0x10])))
        return self.ser.read(1)[0]
