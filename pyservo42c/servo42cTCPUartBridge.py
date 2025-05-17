import socket
from servo42c import Servo42C

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

    def set_angle(self, direction: Servo42C.Direction, speed: int, pulseCount: int) -> bool:
        """
        Set the angle of the servo.
        """
        self.connect()
        data = self.set_angle_cmd(direction, speed, pulseCount)
        self.sock.sendto(data, (self.uart_bridge_ip, self.uart_bridge_port))
        response = self.sock.recv(1024)
        return self.set_angle_response(response)

    def set_en_pin_mode(self, mode: int) -> bool:
        """
        Set the EN pin mode.
        """
        self.connect()
        data = self.set_en_pin_mode_cmd(mode)
        self.sock.sendto(data, (self.uart_bridge_ip, self.uart_bridge_port))
        response = self.sock.recv(1024)
        return self.set_en_pin_mode_response(response)

    def set_subdivision(self, subdivision: int) -> bool:
        """
        Set the subdivision value.
        """
        self.connect()
        data = self.set_subdivision_cmd(subdivision)
        self.sock.sendto(data, (self.uart_bridge_ip, self.uart_bridge_port))
        response = self.sock.recv(1024)
        return self.set_subdivision_response(response)

    def set_constant_speed(self, direction: Servo42C.Direction, speed: int) -> bool:
        """
        Set the motor to run at a constant speed.
        """
        self.connect()
        data = self.set_constant_speed_cmd(direction, speed)
        self.sock.sendto(data, (self.uart_bridge_ip, self.uart_bridge_port))
        response = self.sock.recv(1024)
        return self.set_constant_speed_response(response)

    def stop(self) -> bool:
        """
        Stop the motor.
        """
        self.connect()
        data = self.stop_cmd()
        self.sock.sendto(data, (self.uart_bridge_ip, self.uart_bridge_port))
        response = self.sock.recv(1024)
        return self.stop_response(response)

    def set_motor_type(self, motor_type: Servo42C.MotorType) -> bool:
        """
        Set the motor type.
        """
        self.connect()
        data = self.set_motor_type_cmd(motor_type)
        self.sock.sendto(data, (self.uart_bridge_ip, self.uart_bridge_port))
        response = self.sock.recv(1024)
        return self.set_motor_type_response(response)

    def set_work_mode(self, mode: Servo42C.WorkMode) -> bool:
        """
        Set the work mode of the motor.
        """
        self.connect()
        data = self.set_work_mode_cmd(mode)
        self.sock.sendto(data, (self.uart_bridge_ip, self.uart_bridge_port))
        response = self.sock.recv(1024)
        return self.set_work_mode_response(response)

    def calibrate(self) -> bool:
        """
        Calibrate the motor.
        """
        self.connect()
        data = self.calibrate_cmd()
        self.sock.sendto(data, (self.uart_bridge_ip, self.uart_bridge_port))
        response = self.sock.recv(1024)
        return self.calibrate_response(response)

    def set_current_gear(self, gear: Servo42C.CurrentGear) -> bool:
        """
        Set the current gear.
        """
        self.connect()
        data = self.set_current_gear_cmd(gear)
        self.sock.sendto(data, (self.uart_bridge_ip, self.uart_bridge_port))
        response = self.sock.recv(1024)
        return self.set_current_gear_response(response)

    def set_baud_rate(self, baud_rate: Servo42C.BaudRate) -> bool:
        """
        Set the UART baud rate.
        """
        self.connect()
        data = self.set_baud_rate_cmd(baud_rate)
        self.sock.sendto(data, (self.uart_bridge_ip, self.uart_bridge_port))
        response = self.sock.recv(1024)
        return self.set_baud_rate_response(response)

    def set_zero_mode(self, mode: Servo42C.ZeroMode) -> bool:
        """
        Set the zero mode.
        """
        self.connect()
        data = self.set_zero_mode_cmd(mode)
        self.sock.sendto(data, (self.uart_bridge_ip, self.uart_bridge_port))
        response = self.sock.recv(1024)
        return self.set_zero_mode_response(response)

    def return_to_zero(self) -> bool:
        """
        Return the motor to zero position.
        """
        self.connect()
        data = self.return_to_zero_cmd()
        self.sock.sendto(data, (self.uart_bridge_ip, self.uart_bridge_port))
        response = self.sock.recv(1024)
        return self.return_to_zero_response(response)

    def set_pid_kp(self, kp: int) -> bool:
        """
        Set the PID Kp parameter.
        """
        self.connect()
        data = self.set_pid_kp_cmd(kp)
        self.sock.sendto(data, (self.uart_bridge_ip, self.uart_bridge_port))
        response = self.sock.recv(1024)
        return self.set_pid_kp_response(response)

    def set_pid_ki(self, ki: int) -> bool:
        """
        Set the PID Ki parameter.
        """
        self.connect()
        data = self.set_pid_ki_cmd(ki)
        self.sock.sendto(data, (self.uart_bridge_ip, self.uart_bridge_port))
        response = self.sock.recv(1024)
        return self.set_pid_ki_response(response)

    def set_pid_kd(self, kd: int) -> bool:
        """
        Set the PID Kd parameter.
        """
        self.connect()
        data = self.set_pid_kd_cmd(kd)
        self.sock.sendto(data, (self.uart_bridge_ip, self.uart_bridge_port))
        response = self.sock.recv(1024)
        return self.set_pid_kd_response(response)

    def set_acceleration(self, acceleration: int) -> bool:
        """
        Set the acceleration parameter.
        """
        self.connect()
        data = self.set_acceleration_cmd(acceleration)
        self.sock.sendto(data, (self.uart_bridge_ip, self.uart_bridge_port))
        response = self.sock.recv(1024)
        return self.set_acceleration_response(response)

    def set_max_torque(self, max_torque: int) -> bool:
        """
        Set the maximum torque parameter.
        """
        self.connect()
        data = self.set_max_torque_cmd(max_torque)
        self.sock.sendto(data, (self.uart_bridge_ip, self.uart_bridge_port))
        response = self.sock.recv(1024)
        return self.set_max_torque_response(response)
