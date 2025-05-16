
import struct
from enum import Enum


class ReadParams(Enum):
    """ Enum for the read parameters. """
    ENCODER_VALUE = 0x30
    RECEIVED_PULSE_COUNT = 0x33
    MOTOR_ANGLE = 0x36
    # etc

class WriteParams(Enum):
    """ Enum for the write parameters. """
    CALIBRATION = 0x80
    # etc

class ZeroModeParams(Enum):
    """ Enum for the zero mode parameters. """
    ZERO_MODE = 0x90
    # etc

class PIDAccTorque(Enum):
    """ Enum for the PID, Acceleration, and Torque parameters. """
    PID_KP = 0xA0
    PID_KI = 0xA2
    PID_KD = 0xA3
    # etc

class Control(Enum):
    """ Enum for the control parameters. """
    EN_PIN_MODE = 0xF3
    CONSTANT_SPEED = 0xF6
    STOP = 0xF7
    SET_ANGLE = 0xFD

class Direction(Enum):
    """ Enum for the direction parameters. """
    CLOCKWISE = 0
    COUNTERCLOCKWISE = 1

def calculate_checksum(data: bytes) -> bytes:
    """
    Calculate the checksum for the given data.
    Checksum is the lowest byte of the sum of all bytes.
    """
    checksum = sum(data) & 0xFF
    return bytes([checksum])

class Servo42C:
    def __init__(self, address=0xe0):
        self.address = address


    def set_en_pin_mode_cmd(self, mode: int) -> bytes:
        """
            Returns the bytes to perform a set_en_pin_mode command.
            Bytes:
                0: address
                1: Control.EN_PIN_MODE
                2: Mode (0: Disable, 1: Enable)
                3: Checksum
        """
        # Create the command bytes
        data = bytearray(4)
        data[0] = self.address
        data[1] = Control.EN_PIN_MODE.value
        data[2] = mode

        # Calculate the checksum
        data[3] = calculate_checksum(data[:-1])[0]

        return bytes(data)

    def set_en_pin_mode_response(self, data: bytes) -> bool:
        """
            Parses the response from the servo.
            Returns True if the response is valid, False otherwise.
            Bytes:
                0: address
                1: Result (0x00: failure, 0x01: success)
                2: Checksum
        """
        # Check if the response is valid
        if len(data) != 3:
            return False

        # Check if the address matches
        if data[0] != self.address:
            return False

        # Check if the checksum is valid
        checksum = calculate_checksum(data[:-1])
        if data[2] != checksum[0]:
            return False

        # Check if the result is success
        return data[1] == 0x01

    def set_angle_cmd(self, direction: Direction, speed: int, pulseCount: int) -> bytes:
        """
            Returns the bytes to perform a set_angle command.
            Bytes:
                0: address
                1: Control.SET_ANGLE
                2: Bit 0: Direction, 1-7: Speed
                3-4: Pulse count
                5: Checksum
        """

        # Check if speed is in the range 0-127
        if not (0 <= speed <= 127):
            raise ValueError("Speed must be between 0 and 127")

        # Check if pulseCount is in the range 0-65535
        if not (0 <= pulseCount <= 65535):
            raise ValueError("Pulse count must be between 0 and 65535")

        # Create the command bytes
        data = bytearray(6)
        data[0] = self.address
        data[1] = Control.SET_ANGLE.value
        data[2] = (direction << 7) | speed
        data[3] = (pulseCount >> 8) & 0xFF
        data[4] = pulseCount & 0xFF

        # Calculate the checksum
        data[5] = calculate_checksum(data[:-1])[0]

        return bytes(data)

    def set_angle_response(self, data: bytes) -> bool:
        """
            Parses the response from the servo.
            Returns True if the response is valid, False otherwise.
            Bytes:
                0: address
                1: Result (0x00: failure, 0x01: success)
                2: Checksum (NB: No? I'm only getting 2 bytes back)
        """
        # Check if the response is valid
        # if len(data) != 3:
        if len(data) != 2:
            return False

        # Check if the address matches
        if data[0] != self.address:
            return False

        # Check if the checksum is valid
        # This is commented out because the motor only returns 2 bytes.

        # checksum = calculate_checksum(data[:-1])
        # if data[2] != checksum[0]:
        #     return False

        # Check if the result is success
        return data[1] == 0x01




