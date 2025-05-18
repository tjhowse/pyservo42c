from servo42c import Servo42C
import serial


class Servo42CUart(Servo42C):
    """
    This can be used to drive a servo42c over a UART COM port.
    """

    def __init__(self, port: str, baud: int, address=0xE0):
        super().__init__(address)
        super().set_readwriter(self.uart_readwriter)
        self.com_port_identifier = port
        self.baud_rate = baud
        self.com_port = None
        self.connected = False

    def connect(self):
        """
        Connect to the COM port.
        """
        if self.connected:
            return
        try:
            self.com_port = serial.Serial(
                self.com_port_identifier, self.baud_rate, timeout=1
            )
            self.connected = True
        except serial.SerialException as e:
            print(f"Error connecting to COM port: {e}")
            self.connected = False
            raise

    def disconnect(self):
        """
        Disconnect from the COM port.
        """
        if self.connected:
            self.com_port.close()
            self.connected = False
            self.com_port = None

    def uart_readwriter(self, command: bytes) -> bytes:
        """
        Send a command to the COM port and get the response.
        """
        if not self.connected:
            self.connect()
        self.com_port.write(command)
        response = self.com_port.read(8)
        return response
