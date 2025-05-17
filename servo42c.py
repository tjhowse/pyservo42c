from enum import Enum

class Servo42C:
    """
    This implement the Servo42C protocol.
    https://github.com/makerbase-mks/MKS-SERVO42C/wiki/Serial-communication-description
    """

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
        MA_600 = 0x03
        MA_800 = 0x04
        MA_1000 = 0x05
        MA_1200 = 0x06
        MA_1400 = 0x07
        MA_1600 = 0x08
        MA_1800 = 0x09
        MA_2000 = 0x0A
        MA_2200 = 0x0B
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

    def __init__(self, address=0xe0, expect_checksum=False):
        self.address = address

        # The spec indicates that the last byte of the response is a checksum,
        # but this does not seem to be the case in practice.
        self.expect_checksum = expect_checksum

    def verify_response(self, data: bytes, expected_length: int) -> bool:
        """
        Verify the response from the servo.
        Returns True if the response is valid, False otherwise.
        """

        # Check if the response length is correct
        if len(data) != (expected_length if self.expect_checksum else expected_length - 1):
            print("wrong length")
            return False

        # Check if the address matches
        if data[0] != self.address:
            print("wrong address")
            return False

        # Check if the checksum is valid
        if self.expect_checksum:
            checksum = Servo42C.calculate_checksum(data[:-1])
            if data[-1] != checksum[0]:
                print("wrong checksum")
                return False

        return True

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
        Parses the response from the servo for the set_en_pin_mode command.
        """
        if not self.verify_response(data, 3):
            return False
        return data[1] == Servo42C.Result.SUCCESS.value

    def set_angle_cmd(self, direction: Direction, speed: int, pulseCount: int) -> bytes:
        """
            Returns the bytes to perform a set_angle command.
            Bytes:
                0: address
                1: Control.SET_ANGLE
                2: Bit 0: Direction, 1-7: Speed
                3-4: Pulse count
                5: Checksum

            Note: The spec claims the pulse count field can be 4 bytes long, but
            this doesn't work in testing.
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
        data[2] = (direction.value << 7) | speed
        data[3] = (pulseCount >> 8) & 0xFF
        data[4] = pulseCount & 0xFF

        # Calculate the checksum
        data[5] = Servo42C.calculate_checksum(data[:-1])[0]

        return bytes(data)

    def set_angle_response(self, data: bytes) -> bool:
        """
        Parses the response from the servo for the set_angle command.
        """
        if not self.verify_response(data, 3):
            return False
        return data[1] == Servo42C.Result.SUCCESS.value

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
        """
        if not self.verify_response(data, 3):
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
        """
        if not self.verify_response(data, 3):
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
        """
        if not self.verify_response(data, 3):
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
        """
        if not self.verify_response(data, 3):
            return False
        return data[1] == Servo42C.Result.SUCCESS.value

    def set_work_mode_cmd(self, mode: WorkMode) -> bytes:
        """
        Returns the bytes to perform a set_work_mode command.
        Bytes:
            0: address
            1: WriteParams.SET_WORK_MODE
            2: Mode (0x00: CR_OPEN, 0x01: CR_VFOC, 0x02: CR_UART)
            3: Checksum
        """
        data = bytearray(4)
        data[0] = self.address
        data[1] = Servo42C.WriteParams.SET_WORK_MODE.value
        data[2] = mode.value
        data[3] = Servo42C.calculate_checksum(data[:-1])[0]

        return bytes(data)

    def set_work_mode_response(self, data: bytes) -> bool:
        """
        Parses the response from the servo for the set_work_mode command.
        """
        if not self.verify_response(data, 3):
            return False
        return data[1] == Servo42C.Result.SUCCESS.value

    def calibrate_cmd(self) -> bytes:
        """
        Returns the bytes to perform a calibration command.
        Bytes:
            0: address
            1: WriteParams.CALIBRATION
            2: 0x00
            3: Checksum
        """
        data = bytearray(4)
        data[0] = self.address
        data[1] = Servo42C.WriteParams.CALIBRATION.value
        data[2] = 0x00
        data[3] = Servo42C.calculate_checksum(data[:-1])[0]

        return bytes(data)

    def calibrate_response(self, data: bytes) -> bool:
        """
        Parses the response from the servo for the calibration command.
        """
        if not self.verify_response(data, 3):
            return False
        return data[1] == Servo42C.Result.SUCCESS.value

    def set_current_gear_cmd(self, gear: CurrentGear) -> bytes:
        """
        Returns the bytes to perform a set_current_gear command.
        Bytes:
            0: address
            1: WriteParams.SET_CURRENT_GEAR
            2: Gear (0x00: 0mA, 0x01: 200mA, ..., 0x0C: 2400mA)
            3: Checksum
        """
        data = bytearray(4)
        data[0] = self.address
        data[1] = Servo42C.WriteParams.SET_CURRENT_GEAR.value
        data[2] = gear.value
        data[3] = Servo42C.calculate_checksum(data[:-1])[0]

        return bytes(data)

    def set_current_gear_response(self, data: bytes) -> bool:
        """
        Parses the response from the servo for the set_current_gear command.
        """
        if not self.verify_response(data, 3):
            return False
        return data[1] == Servo42C.Result.SUCCESS.value

    def set_baud_rate_cmd(self, baud_rate: BaudRate) -> bytes:
        """
        Returns the bytes to perform a set_baud_rate command.
        Bytes:
            0: address
            1: WriteParams.SET_BAUD_RATE
            2: Baud rate (0x01: 9600, 0x02: 19200, ..., 0x06: 115200)
            3: Checksum
        """
        data = bytearray(4)
        data[0] = self.address
        data[1] = Servo42C.WriteParams.SET_BAUD_RATE.value
        data[2] = baud_rate.value
        data[3] = Servo42C.calculate_checksum(data[:-1])[0]

        return bytes(data)

    def set_baud_rate_response(self, data: bytes) -> bool:
        """
        Parses the response from the servo for the set_baud_rate command.
        """
        if not self.verify_response(data, 3):
            return False
        return data[1] == Servo42C.Result.SUCCESS.value

    def set_zero_mode_cmd(self, mode: ZeroMode) -> bytes:
        """
        Returns the bytes to perform a set_zero_mode command.
        Bytes:
            0: address
            1: ZeroModeParams.ZERO_MODE
            2: Mode (0x00: Disable, 0x01: DirMode, 0x02: NearMode)
            3: Checksum
        """
        data = bytearray(4)
        data[0] = self.address
        data[1] = Servo42C.ZeroModeParams.ZERO_MODE.value
        data[2] = mode.value
        data[3] = Servo42C.calculate_checksum(data[:-1])[0]

        return bytes(data)

    def set_zero_mode_response(self, data: bytes) -> bool:
        """
        Parses the response from the servo for the set_zero_mode command.
        """
        if not self.verify_response(data, 3):
            return False
        return data[1] == Servo42C.Result.SUCCESS.value

    def return_to_zero_cmd(self) -> bytes:
        """
        Returns the bytes to perform a return_to_zero command.
        Bytes:
            0: address
            1: ZeroModeParams.RETURN_TO_ZERO
            2: 0x00
            3: Checksum
        """
        data = bytearray(4)
        data[0] = self.address
        data[1] = Servo42C.ZeroModeParams.RETURN_TO_ZERO.value
        data[2] = 0x00
        data[3] = Servo42C.calculate_checksum(data[:-1])[0]

        return bytes(data)

    def return_to_zero_response(self, data: bytes) -> bool:
        """
        Parses the response from the servo for the return_to_zero command.
        """
        if not self.verify_response(data, 3):
            return False
        return data[1] == Servo42C.Result.SUCCESS.value

    def set_pid_kp_cmd(self, kp: int) -> bytes:
        """
        Returns the bytes to perform a set_pid_kp command.
        Bytes:
            0: address
            1: PIDAccTorque.PID_KP
            2-3: Kp value (uint16_t)
            4: Checksum
        """
        if not (0 <= kp <= 0xFFFF):
            raise ValueError("Kp must be between 0 and 65535")

        data = bytearray(5)
        data[0] = self.address
        data[1] = Servo42C.PIDAccTorque.PID_KP.value
        data[2] = (kp >> 8) & 0xFF
        data[3] = kp & 0xFF
        data[4] = Servo42C.calculate_checksum(data[:-1])[0]

        return bytes(data)

    def set_pid_kp_response(self, data: bytes) -> bool:
        """
        Parses the response from the servo for the set_pid_kp command.
        """
        if not self.verify_response(data, 3):
            return False
        return data[1] == Servo42C.Result.SUCCESS.value

    def set_pid_ki_cmd(self, ki: int) -> bytes:
        """
        Returns the bytes to perform a set_pid_ki command.
        Bytes:
            0: address
            1: PIDAccTorque.PID_KI
            2-3: Ki value (uint16_t)
            4: Checksum
        """
        if not (0 <= ki <= 0xFFFF):
            raise ValueError("Ki must be between 0 and 65535")

        data = bytearray(5)
        data[0] = self.address
        data[1] = Servo42C.PIDAccTorque.PID_KI.value
        data[2] = (ki >> 8) & 0xFF
        data[3] = ki & 0xFF
        data[4] = Servo42C.calculate_checksum(data[:-1])[0]

        return bytes(data)

    def set_pid_ki_response(self, data: bytes) -> bool:
        """
        Parses the response from the servo for the set_pid_ki command.
        """
        if not self.verify_response(data, 3):
            return False
        return data[1] == Servo42C.Result.SUCCESS.value

    def set_pid_kd_cmd(self, kd: int) -> bytes:
        """
        Returns the bytes to perform a set_pid_kd command.
        Bytes:
            0: address
            1: PIDAccTorque.PID_KD
            2-3: Kd value (uint16_t)
            4: Checksum
        """
        if not (0 <= kd <= 0xFFFF):
            raise ValueError("Kd must be between 0 and 65535")

        data = bytearray(5)
        data[0] = self.address
        data[1] = Servo42C.PIDAccTorque.PID_KD.value
        data[2] = (kd >> 8) & 0xFF
        data[3] = kd & 0xFF
        data[4] = Servo42C.calculate_checksum(data[:-1])[0]

        return bytes(data)

    def set_pid_kd_response(self, data: bytes) -> bool:
        """
        Parses the response from the servo for the set_pid_kd command.
        """
        if not self.verify_response(data, 3):
            return False
        return data[1] == Servo42C.Result.SUCCESS.value

    def set_acceleration_cmd(self, acceleration: int) -> bytes:
        """
        Returns the bytes to perform a set_acceleration command.
        Bytes:
            0: address
            1: PIDAccTorque.ACCELERATION
            2-3: Acceleration value (uint16_t)
            4: Checksum
        """
        if not (0 <= acceleration <= 0xFFFF):
            raise ValueError("Acceleration must be between 0 and 65535")

        data = bytearray(5)
        data[0] = self.address
        data[1] = Servo42C.PIDAccTorque.ACCELERATION.value
        data[2] = (acceleration >> 8) & 0xFF
        data[3] = acceleration & 0xFF
        data[4] = Servo42C.calculate_checksum(data[:-1])[0]

        return bytes(data)

    def set_acceleration_response(self, data: bytes) -> bool:
        """
        Parses the response from the servo for the set_acceleration command.
        """
        if not self.verify_response(data, 3):
            return False
        return data[1] == Servo42C.Result.SUCCESS.value

    def set_max_torque_cmd(self, max_torque: int) -> bytes:
        """
        Returns the bytes to perform a set_max_torque command.
        Bytes:
            0: address
            1: PIDAccTorque.MAX_TORQUE
            2-3: Max torque value (uint16_t)
            4: Checksum
        """
        if not (0 <= max_torque <= 0x4B0):
            raise ValueError("Max torque must be between 0 and 1200")

        data = bytearray(5)
        data[0] = self.address
        data[1] = Servo42C.PIDAccTorque.MAX_TORQUE.value
        data[2] = (max_torque >> 8) & 0xFF
        data[3] = max_torque & 0xFF
        data[4] = Servo42C.calculate_checksum(data[:-1])[0]

        return bytes(data)

    def set_max_torque_response(self, data: bytes) -> bool:
        """
        Parses the response from the servo for the set_max_torque command.
        """
        if not self.verify_response(data, 3):
            return False
        return data[1] == Servo42C.Result.SUCCESS.value

    def save_or_clear_status_cmd(self, action: SaveOrClearStatus) -> bytes:
        """
        Returns the bytes to perform a save_or_clear_status command.
        Bytes:
            0: address
            1: Control.SAVE_OR_CLEAR_STATUS
            2: Action (0xC8: Save, 0xCA: Clear)
            3: Checksum
        """
        data = bytearray(4)
        data[0] = self.address
        data[1] = Servo42C.Control.SAVE_OR_CLEAR_STATUS.value
        data[2] = action.value
        data[3] = Servo42C.calculate_checksum(data[:-1])[0]

        return bytes(data)

    def save_or_clear_status_response(self, data: bytes) -> bool:
        """
        Parses the response from the servo for the save_or_clear_status command.
        """
        if not self.verify_response(data, 3):
            return False
        return data[1] == Servo42C.Result.SUCCESS.value

