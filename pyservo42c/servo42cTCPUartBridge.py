import socket
from servo42c import Servo42C

class Servo42CTCPUartBridge(Servo42C):
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
        self.connecting = False
        self.sock.settimeout(1)

    def connect(self):
        """
        Connect to the UART bridge.
        """
        if self.connected or self.connecting:
            return
        try:
            self.connecting = True
            self.sock.connect((self.uart_bridge_ip, self.uart_bridge_port))
            self.connected = True
            self.connecting = False
        except OSError as e:
            print(f"Error connecting to UART bridge: {e}")
            self.connected = False
            self.connecting = False
            raise

    def disconnect(self):
        """
        Disconnect from the UART bridge.
        """
        if self.connected:
            self.sock.close()
            self.connected = False

    def send_command_get_response(self, command: bytes) -> bytes:
        """
        Send a command to the UART bridge and get the response.
        Always receive 1024 bytes.
        """
        self.connect()
        self.sock.sendall(command)
        return self.sock.recv(8)

    def set_angle(self, direction: Servo42C.Direction, speed: int, pulseCount: int) -> bool:
        """
        Set the angle of the servo.
        """
        response = self.send_command_get_response(self.set_angle_cmd(direction, speed, pulseCount))
        return self.set_angle_response(response)

    def set_en_pin_mode(self, mode: int) -> bool:
        """
        Set the EN pin mode.
        """
        response = self.send_command_get_response(self.set_en_pin_mode_cmd(mode))
        return self.set_en_pin_mode_response(response)

    def set_subdivision(self, subdivision: int) -> bool:
        """
        Set the subdivision value.
        """
        response = self.send_command_get_response(self.set_subdivision_cmd(subdivision))
        return self.set_subdivision_response(response)

    def set_constant_speed(self, direction: Servo42C.Direction, speed: int) -> bool:
        """
        Set the motor to run at a constant speed.
        """
        response = self.send_command_get_response(self.set_constant_speed_cmd(direction, speed))
        return self.set_constant_speed_response(response)

    def stop(self) -> bool:
        """
        Stop the motor.
        """
        response = self.send_command_get_response(self.stop_cmd())
        return self.stop_response(response)

    def set_motor_type(self, motor_type: Servo42C.MotorType) -> bool:
        """
        Set the motor type.
        """
        response = self.send_command_get_response(self.set_motor_type_cmd(motor_type))
        return self.set_motor_type_response(response)

    def set_work_mode(self, mode: Servo42C.WorkMode) -> bool:
        """
        Set the work mode of the motor.
        """
        response = self.send_command_get_response(self.set_work_mode_cmd(mode))
        return self.set_work_mode_response(response)

    def calibrate(self) -> bool:
        """
        Calibrate the motor.
        """
        response = self.send_command_get_response(self.calibrate_cmd())
        return self.calibrate_response(response)

    def set_current_gear(self, gear: Servo42C.CurrentGear) -> bool:
        """
        Set the current gear.
        """
        response = self.send_command_get_response(self.set_current_gear_cmd(gear))
        return self.set_current_gear_response(response)

    def set_baud_rate(self, baud_rate: Servo42C.BaudRate) -> bool:
        """
        Set the UART baud rate.
        """
        response = self.send_command_get_response(self.set_baud_rate_cmd(baud_rate))
        return self.set_baud_rate_response(response)

    def set_zero_mode(self, mode: Servo42C.ZeroMode) -> bool:
        """
        Set the zero mode.
        """
        response = self.send_command_get_response(self.set_zero_mode_cmd(mode))
        return self.set_zero_mode_response(response)

    def return_to_zero(self) -> bool:
        """
        Return the motor to zero position.
        """
        response = self.send_command_get_response(self.return_to_zero_cmd())
        return self.return_to_zero_response(response)

    def set_pid_kp(self, kp: int) -> bool:
        """
        Set the PID Kp parameter.
        """
        response = self.send_command_get_response(self.set_pid_kp_cmd(kp))
        return self.set_pid_kp_response(response)

    def set_pid_ki(self, ki: int) -> bool:
        """
        Set the PID Ki parameter.
        """
        response = self.send_command_get_response(self.set_pid_ki_cmd(ki))
        return self.set_pid_ki_response(response)

    def set_pid_kd(self, kd: int) -> bool:
        """
        Set the PID Kd parameter.
        """
        response = self.send_command_get_response(self.set_pid_kd_cmd(kd))
        return self.set_pid_kd_response(response)

    def set_acceleration(self, acceleration: int) -> bool:
        """
        Set the acceleration parameter.
        """
        response = self.send_command_get_response(self.set_acceleration_cmd(acceleration))
        return self.set_acceleration_response(response)

    def set_max_torque(self, max_torque: int) -> bool:
        """
        Set the maximum torque parameter.
        """
        response = self.send_command_get_response(self.set_max_torque_cmd(max_torque))
        return self.set_max_torque_response(response)

    def save_or_clear_status(self, action: Servo42C.SaveOrClearStatus) -> bool:
        """
        Save or clear the status.
        """
        response = self.send_command_get_response(self.save_or_clear_status_cmd(action))
        return self.save_or_clear_status_response(response)
