#!/usr/bin/env python3

from time import sleep
from servo42cTCPUartBridge import Servo42CTCPUartBridge
from servo42c import Servo42C
from servo42cUart import Servo42CUart

UART_BRIDGE_IP = "192.168.10.248"
UART_BRIDGE_PORT = 6638

def run_tcp_uart_test():
    s = Servo42CTCPUartBridge(UART_BRIDGE_IP, UART_BRIDGE_PORT)
    while True:
        try:
            run_test(s)
        except Exception as e:
            print(f"Error: Could not connect to the UART bridge: {e}")
            sleep(1)
            s.disconnect()
            s = Servo42CTCPUartBridge(UART_BRIDGE_IP, UART_BRIDGE_PORT)

def run_test(s: Servo42C):
    if not s._save_or_clear_status_cmd(Servo42CTCPUartBridge.SaveOrClearStatus.CLEAR):
        print("Failed to clear status")

    if not s.set_constant_speed(Servo42CTCPUartBridge.Direction.CLOCKWISE, 126):
        print("Failed to set constant speed")
    sleep(1)

    if not s.stop():
        print("Failed to stop")
    sleep(1)

    if not s.set_angle(Servo42CTCPUartBridge.Direction.CLOCKWISE, 127, 200*16):
        print("Failed to set angle")
    sleep(0.5)

    if not s.set_angle(Servo42CTCPUartBridge.Direction.COUNTERCLOCKWISE, 127, 100*16):
        print("Failed to set angle")
    sleep(0.5)

    print(s.read_encoder_value())



if __name__ == "__main__":
    run_tcp_uart_test()