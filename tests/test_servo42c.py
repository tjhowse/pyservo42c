import unittest
from pyservo42c.servo42c import Servo42C


class TestServo42C(unittest.TestCase):
    def setUp(self):
        self.servo = Servo42C(address=0xE0)

    def test_set_constant_speed_cmd(self):
        # Example: set_constant_speed_cmd(0, 1) -> [e0 f6 01 d7]
        result = self.servo._set_constant_speed_cmd(Servo42C.Direction.CLOCKWISE, 1)
        self.assertEqual(result, bytes([0xE0, 0xF6, 0x01, 0xD7]))

        # Example: set_constant_speed_cmd(1, 1) -> [e0 f6 81 57]
        result = self.servo._set_constant_speed_cmd(
            Servo42C.Direction.COUNTERCLOCKWISE, 1
        )
        self.assertEqual(result, bytes([0xE0, 0xF6, 0x81, 0x57]))

    def test_set_angle_cmd(self):
        # Example: set_angle_cmd(0, 1, 3200) -> [e0 fd 01 00 0c 80 6a]
        result = self.servo._set_angle_cmd(Servo42C.Direction.CLOCKWISE, 1, 3200)
        self.assertEqual(result, bytes([0xE0, 0xFD, 0x01, 0x0C, 0x80, 0x6A]))

        # Example: set_angle_cmd(1, 6, 3200) -> [e0 fd 86 00 0c 80 ef]
        result = self.servo._set_angle_cmd(Servo42C.Direction.COUNTERCLOCKWISE, 6, 3200)
        self.assertEqual(result, bytes([0xE0, 0xFD, 0x86, 0x0C, 0x80, 0xEF]))

    def test_set_subdivision_cmd(self):
        # Example: set_subdivision_cmd(26) -> [e0 84 1a 7e]
        result = self.servo._set_subdivision_cmd(26)
        self.assertEqual(result, bytes([0xE0, 0x84, 0x1A, 0x7E]))

        # Example: set_subdivision_cmd(256) -> [e0 84 00 64]
        result = self.servo._set_subdivision_cmd(256)
        self.assertEqual(result, bytes([0xE0, 0x84, 0x00, 0x64]))

    def test_set_baud_rate_cmd(self):
        # Example: set_baud_rate_cmd(Servo42C.BaudRate.BAUD_38400) -> [e0 8a 04 6e]
        result = self.servo._set_baud_rate_cmd(Servo42C.BaudRate.BAUD_38400)
        self.assertEqual(result, bytes([0xE0, 0x8A, 0x04, 0x6E]))

    def test_calibrate_cmd(self):
        # Example: calibrate_cmd() -> [e0 80 00 60]
        result = self.servo._calibrate_cmd()
        self.assertEqual(result, bytes([0xE0, 0x80, 0x00, 0x60]))

    def test_set_work_mode_cmd(self):
        # Example: set_work_mode_cmd(Servo42C.WorkMode.CR_VFOC) -> [e0 82 01 63]
        result = self.servo._set_work_mode_cmd(Servo42C.WorkMode.CR_VFOC)
        self.assertEqual(result, bytes([0xE0, 0x82, 0x01, 0x63]))

    def test_set_current_gear_cmd(self):
        # Example: set_current_gear_cmd(Servo42C.CurrentGear.MA_1200) -> [e0 83 06 69]
        result = self.servo._set_current_gear_cmd(Servo42C.CurrentGear.MA_1200)
        self.assertEqual(result, bytes([0xE0, 0x83, 0x06, 0x69]))

    def test_set_zero_mode_cmd(self):
        # Example: set_zero_mode_cmd(Servo42C.ZeroMode.DIR_MODE) -> [e0 90 01 71]
        result = self.servo._set_zero_mode_cmd(Servo42C.ZeroMode.DIR_MODE)
        self.assertEqual(result, bytes([0xE0, 0x90, 0x01, 0x71]))

    def test_return_to_zero_cmd(self):
        # Example: return_to_zero_cmd() -> [e0 94 00 74]
        result = self.servo._return_to_zero_cmd()
        self.assertEqual(result, bytes([0xE0, 0x94, 0x00, 0x74]))

    def test_set_pid_kp_cmd(self):
        # Example: set_pid_kp_cmd(0x120) -> [e0 a1 01 20 a2]
        result = self.servo._set_pid_kp_cmd(0x120)
        self.assertEqual(result, bytes([0xE0, 0xA1, 0x01, 0x20, 0xA2]))

    def test_set_pid_ki_cmd(self):
        # Example: set_pid_ki_cmd(0x02) -> [e0 a2 00 02 84]
        result = self.servo._set_pid_ki_cmd(0x02)
        self.assertEqual(result, bytes([0xE0, 0xA2, 0x00, 0x02, 0x84]))

    def test_set_pid_kd_cmd(self):
        # Example: set_pid_kd_cmd(0x250) -> [e0 a3 02 50 d5]
        result = self.servo._set_pid_kd_cmd(0x250)
        self.assertEqual(result, bytes([0xE0, 0xA3, 0x02, 0x50, 0xD5]))

    def test_set_acceleration_cmd(self):
        # Example: set_acceleration_cmd(0x80) -> [e0 a4 00 80 04]
        result = self.servo._set_acceleration_cmd(0x80)
        self.assertEqual(result, bytes([0xE0, 0xA4, 0x00, 0x80, 0x04]))

    def test_set_max_torque_cmd(self):
        # Example: set_max_torque_cmd(0x258) -> [e0 a5 02 58 df]
        result = self.servo._set_max_torque_cmd(0x258)
        self.assertEqual(result, bytes([0xE0, 0xA5, 0x02, 0x58, 0xDF]))
    
    def test_read_encoder_value_response(self):
        # NB the spec indicates the last byte of the response should be
        # a checksum. This is not so in practice.
        data = [0xE0, 0x40, 0x11]
        result = self.servo._read_encoder_value_response(data)
        self.assertEqual(result, 16401)

    def test_read_param_response(self):
        # NB the spec indicates the last byte of the response should be
        # a checksum. This is not so in practice.
        # data = [0xE0, 0x01, 0xE1]
        data = [0xE0, 0x01]
        result = self.servo. _read_param_response(data, 3)
        self.assertEqual(result, [0x01])


if __name__ == "__main__":
    unittest.main()
