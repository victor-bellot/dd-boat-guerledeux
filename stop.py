from arduino_driver_v2 import ArduinoIO

if __name__ == "__main__":
    ard = ArduinoIO()
    ard.send_arduino_cmd_motor(0, 0)
