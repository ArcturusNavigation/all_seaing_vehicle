import Jetson.GPIO as GPIO
from colorama import Fore, Back, Style
import time
import os

# pin 13
"""
11, 13
"""
samples = {7, 11, 12, 13 ,15, 16, 18, 19, 21, 22, 23, 24, 26, 29, 31, 32, 33, 35, 36, 37, 38, 40}
GND = {6, 9, 14, 20, 25, 30, 34, 39}

def select_pin():
    os.system('clear')
    print()
    while True:
        pin = input("Enter pin number: ")
        try:
            pin = int(pin)
            if pin not in samples:
                raise ValueError("Pin is not a valid toggleable pin")
            return pin
        except:
            print("Invalid pin number or pin number is not a toggleable pin")

def main():
    GPIO.setmode(GPIO.BOARD)
    current_pin, high = None, False
    while True:
        os.system('clear')
        if not current_pin:
            current_pin = select_pin()
            GPIO.setup(current_pin, GPIO.OUT, initial=GPIO.LOW)
            GPIO.output(current_pin, GPIO.LOW)
            continue
        print_pin(current_pin)
        print(f"\r TESTING PIN {current_pin}")
        print("Arcturus GPIO Test - Hastily written by Perry Han on 10/13/24")
        print("Type h to set high, l to set low, or d to deselect. Type exit to quit program")
        print(f"State: " + ("high" if high else "low"))
        match input("Enter command: "):
            case "h":
                GPIO.output(current_pin, GPIO.HIGH)
                high = True
            case "l":
                GPIO.output(current_pin, GPIO.LOW)
                high = False
            case "d":
                GPIO.output(current_pin, GPIO.LOW)
                high = False
                # GPIO.cleanup()
                current_pin = None
            case "exit":

                break
            case "_":
                print("Unrecognized command")

    os.system('clear')
    GPIO.cleanup()

def print_pin(pin):
    for row in [(39, 0, -2), (40, 1, -2)]:
        for i in range(*row):
            if i in GND:
                print(Fore.YELLOW + "◎ ", end=" ")
            elif i == pin:
                print(Fore.CYAN + "◎ ", end=" ")
            else: print(Fore.RESET + "◎ ", end=" ")
        print("\n")
    print(Fore.RESET + "\n")


if __name__ == "__main__":
    main()