import socket
from enum import Enum

class Servo42C:
    class ReadParams(Enum):
        """ Enum for the read parameters. """
        ENCODER_VALUE = 0x30
        RECEIVED_PULSE_COUNT = 0x33
        MOTOR_ANGLE = 0x36
        MOTOR_ANGLE_ERROR = 0x39
        EN_PIN_STATUS = 0x3A
        MOTOR_STATUS = 0x3E

    class WriteParams(Enum):
        """ Enum for the write parameters. """
        CALIBRATION = 0x80
        SET_MOTOR_TYPE = 0x81
        SET_WORK_MODE = 0x82
        SET_CURRENT_GEAR = 0x83
        SET_SUBDIVISION = 0x84
        SET_EN_PIN_ACTIVE = 0x85
        SET_MOTOR_DIRECTION = 0x86
        SET_AUTO_SCREEN_OFF = 0x87
        SET_STALL_PROTECTION = 0x88
        SET_SUBDIVISION_INTERPOLATION = 0x89
        SET_BAUD_RATE = 0x8A
        SET_UART_ADDRESS = 0x8B

    class ZeroModeParams(Enum):
        """ Enum for the zero mode parameters. """
        ZERO_MODE = 0x90
        ZERO_POSITION = 0x91
        ZERO_SPEED = 0x92
        ZERO_DIRECTION = 0x93
        RETURN_TO_ZERO = 0x94

    class PIDAccTorque(Enum):
        """ Enum for the PID, Acceleration, and Torque parameters. """
        PID_KP = 0xA1
        PID_KI = 0xA2
        PID_KD = 0xA3
        ACCELERATION = 0xA4
        MAX_TORQUE = 0xA5

    class Control(Enum):
        """ Enum for the control parameters. """
        EN_PIN_MODE = 0xF3
        CONSTANT_SPEED = 0xF6
        STOP = 0xF7
        SAVE_OR_CLEAR_STATUS = 0xFF
        SET_ANGLE = 0xFD

    class Direction(Enum):
        """ Enum for the direction parameters. """
        CLOCKWISE = 0
        COUNTERCLOCKWISE = 1

    class Result(Enum):
        """ Enum for common result values. """
        SUCCESS = 0x01
        FAILURE = 0x00

    class MotorType(Enum):
        """ Enum for motor types. """
        DEGREE_0_9 = 0x00
        DEGREE_1_8 = 0x01

    class WorkMode(Enum):
        """ Enum for work modes. """
        CR_OPEN = 0x00
        CR_VFOC = 0x01
        CR_UART = 0x02

    class CurrentGear(Enum):
        """ Enum for current gear values. """
        MA_0 = 0x00
        MA_200 = 0x01
        MA_400 = 0x02
        MA_2400 = 0x0C

    class EnPinActive(Enum):
        """ Enum for En pin active states. """
        ACTIVE_LOW = 0x00
        ACTIVE_HIGH = 0x01
        ACTIVE_ALWAYS = 0x02

    class AutoScreenOff(Enum):
        """ Enum for auto screen off states. """
        DISABLE = 0x00
        ENABLE = 0x01

    class StallProtection(Enum):
        """ Enum for stall protection states. """
        DISABLE = 0x00
        ENABLE = 0x01

    class SubdivisionInterpolation(Enum):
        """ Enum for subdivision interpolation states. """
        DISABLE = 0x00
        ENABLE = 0x01

    class BaudRate(Enum):
        """ Enum for baud rate values. """
        BAUD_9600 = 0x01
        BAUD_19200 = 0x02
        BAUD_25000 = 0x03
        BAUD_38400 = 0x04
        BAUD_57600 = 0x05
        BAUD_115200 = 0x06

    class ZeroMode(Enum):
        """ Enum for zero mode states. """
        DISABLE = 0x00
        DIR_MODE = 0x01
        NEAR_MODE = 0x02

    class ZeroDirection(Enum):
        """ Enum for zero direction values. """
        CW = 0x00
        CCW = 0x01

    class SaveOrClearStatus(Enum):
        """ Enum for save or clear status values. """
        SAVE = 0xC8
        CLEAR = 0xCA

    def calculate_checksum(data: bytes) -> bytes:
        """
        Calculate the checksum for the given data.
        Checksum is the lowest byte of the sum of all bytes.
        """
        checksum = sum(data) & 0xFF
        return bytes([checksum])

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
        data[1] = Servo42C.Control.EN_PIN_MODE.value
        data[2] = mode

        # Calculate the checksum
        data[3] = Servo42C.calculate_checksum(data[:-1])[0]

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
        checksum = Servo42C.calculate_checksum(data[:-1])
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
        data[1] = Servo42C.Control.SET_ANGLE.value
        data[2] = (direction << 7) | speed
        data[3] = (pulseCount >> 8) & 0xFF
        data[4] = pulseCount & 0xFF

        # Calculate the checksum
        data[5] = Servo42C.calculate_checksum(data[:-1])[0]

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

    def set_subdivision_cmd(self, subdivision: int) -> bytes:
        """
        Returns the bytes to perform a set_subdivision command.
        Bytes:
            0: address
            1: WriteParams.SET_SUBDIVISION
            2: Subdivision (1-256)
            3: Checksum
        """
        if not (1 <= subdivision <= 256):
            raise ValueError("Subdivision must be between 1 and 256")

        data = bytearray(4)
        data[0] = self.address
        data[1] = Servo42C.WriteParams.SET_SUBDIVISION.value
        data[2] = subdivision if subdivision != 256 else 0x00  # 256 is represented as 0x00
        data[3] = Servo42C.calculate_checksum(data[:-1])[0]

        return bytes(data)

    def set_subdivision_response(self, data: bytes) -> bool:
        """
        Parses the response from the servo for the set_subdivision command.
        Returns True if the response is valid, False otherwise.
        Bytes:
            0: address
            1: Result (0x00: failure, 0x01: success)
            2: Checksum
        """
        if len(data) != 3:
            return False

        if data[0] != self.address:
            return False

        checksum = Servo42C.calculate_checksum(data[:-1])
        if data[2] != checksum[0]:
            return False

        return data[1] == Servo42C.Result.SUCCESS.value

    def set_constant_speed_cmd(self, direction: Direction, speed: int) -> bytes:
        """
        Returns the bytes to perform a set_constant_speed command.
        Bytes:
            0: address
            1: Control.CONSTANT_SPEED
            2: Bit 0: Direction, 1-7: Speed
            3: Checksum
        """
        if not (0 <= speed <= 127):
            raise ValueError("Speed must be between 0 and 127")

        data = bytearray(4)
        data[0] = self.address
        data[1] = Servo42C.Control.CONSTANT_SPEED.value
        data[2] = (direction.value << 7) | speed
        data[3] = Servo42C.calculate_checksum(data[:-1])[0]

        return bytes(data)

    def set_constant_speed_response(self, data: bytes) -> bool:
        """
        Parses the response from the servo for the set_constant_speed command.
        Returns True if the response is valid, False otherwise.
        Bytes:
            0: address
            1: Result (0x00: failure, 0x01: success)
            2: Checksum
        """
        if len(data) != 3:
            return False

        if data[0] != self.address:
            return False

        checksum = Servo42C.calculate_checksum(data[:-1])
        if data[2] != checksum[0]:
            return False

        return data[1] == Servo42C.Result.SUCCESS.value

    def stop_cmd(self) -> bytes:
        """
        Returns the bytes to perform a stop command.
        Bytes:
            0: address
            1: Control.STOP
            2: Checksum
        """
        data = bytearray(3)
        data[0] = self.address
        data[1] = Servo42C.Control.STOP.value
        data[2] = Servo42C.calculate_checksum(data[:-1])[0]

        return bytes(data)

    def stop_response(self, data: bytes) -> bool:
        """
        Parses the response from the servo for the stop command.
        Returns True if the response is valid, False otherwise.
        Bytes:
            0: address
            1: Result (0x00: failure, 0x01: success)
            2: Checksum
        """
        if len(data) != 3:
            return False

        if data[0] != self.address:
            return False

        checksum = Servo42C.calculate_checksum(data[:-1])
        if data[2] != checksum[0]:
            return False

        return data[1] == Servo42C.Result.SUCCESS.value

    def set_motor_type_cmd(self, motor_type: MotorType) -> bytes:
        """
        Returns the bytes to perform a set_motor_type command.
        Bytes:
            0: address
            1: WriteParams.SET_MOTOR_TYPE
            2: Motor type (0x00: 0.9°, 0x01: 1.8°)
            3: Checksum
        """
        data = bytearray(4)
        data[0] = self.address
        data[1] = Servo42C.WriteParams.SET_MOTOR_TYPE.value
        data[2] = motor_type.value
        data[3] = Servo42C.calculate_checksum(data[:-1])[0]

        return bytes(data)

    def set_motor_type_response(self, data: bytes) -> bool:
        """
        Parses the response from the servo for the set_motor_type command.
        Returns True if the response is valid, False otherwise.
        Bytes:
            0: address
            1: Result (0x00: failure, 0x01: success)
            2: Checksum
        """
        if len(data) != 3:
            return False

        if data[0] != self.address:
            return False

        checksum = Servo42C.calculate_checksum(data[:-1])
        if data[2] != checksum[0]:
            return False

        return data[1] == Servo42C.Result.SUCCESS.value


class Servo42CUartBridge(Servo42C):
    """
    This can be used to drive a servo42c over a UART bridge.
    Tested using github.com/oxan/esphome-stream-server on an ESP32.
    """
    def __init__(self, uart_bridge_ip: str, uart_bridge_port: int, address=0xe0):
        super().__init__(address)
        self.uart_bridge_ip = uart_bridge_ip
        self.uart_bridge_port = uart_bridge_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connected = False
        self.sock.settimeout(1)

    def connect(self):
        """
        Connect to the UART bridge.
        """
        if self.connected:
            return
        try:
            self.sock.connect((self.uart_bridge_ip, self.uart_bridge_port))
            self.connected = True
        except OSError as e:
            print(f"Error connecting to UART bridge: {e}")
            self.connected = False
            raise

    def set_angle(self, direction: Servo42C.Direction, speed: int, pulseCount: int) -> bool:
        """
        Set the angle of the servo.
        """
        self.connect()
        data = self.set_angle_cmd(direction, speed, pulseCount)
        self.sock.sendto(data, (self.uart_bridge_ip, self.uart_bridge_port))
        response = self.sock.recv(1024)
        return self.set_angle_response(response)
