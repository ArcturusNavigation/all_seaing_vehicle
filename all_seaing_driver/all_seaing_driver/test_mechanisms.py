#!/usr/bin/python

import ArcturusEE
import serial
import time

ser = serial.Serial("/dev/ttyACM0", 115200, timeout = 1)

mech_pwr_a = ArcturusEE.mech_power(ser, 0x05)
mech_pwr_b = ArcturusEE.mech_power(ser, 0x06)

def main():
    #if mech_pwr_a.estop():
    #    print("A: ESTOP ON")
    #else:
    #    print("A: ESTOP OFF")

    if mech_pwr_b.estop():
        print("B: ESTOP ON")
    else:
        print("B: ESTOP OFF")

    print("Setting Voltages to 7.4V and 12.0V")
    mech_pwr_a.set_voltage(7.4)
    mech_pwr_b.set_voltage(12.0)

    print("Turning on outputs")
    mech_pwr_a.output(1)
    mech_pwr_b.output(1)

    time.sleep(3)

    print("Converter A status: %s" % hex(mech_pwr_a.status()))
    print("Converter A current: %s A" % mech_pwr_a.current())

    print("Converter B status: %s" % hex(mech_pwr_b.status()))
    print("Converter B current: %s A" % mech_pwr_b.current())

    #print("Turning off outputs")
    #mech_pwr_a.output(0)
    #mech_pwr_b.output(0)

if __name__ == "__main__":
    main()