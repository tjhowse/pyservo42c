import socket
from servo42c import Servo42C


class Servo42CTCPUartBridge(Servo42C):
    """
    This can be used to drive a servo42c over a UART bridge.
    Tested using github.com/oxan/esphome-stream-server on an ESP32.
    """

    def __init__(self, uart_bridge_ip: str, uart_bridge_port: int, address=0xE0):
        super().__init__(address)
        super().set_readwriter(self.tcp_uart_readwriter)
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

    def tcp_uart_readwriter(self, command: bytes) -> bytes:
        """
        Send a command to the UART bridge and get the response.
        """
        self.connect()
        self.sock.sendall(command)
        return self.sock.recv(8)
