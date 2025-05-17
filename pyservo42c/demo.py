#!/usr/bin/env python3

from time import sleep
from servo42cTCPUartBridge import Servo42CTCPUartBridge

UART_BRIDGE_IP = "uart-bridge-1.local"
UART_BRIDGE_PORT = 6638

def run_uart_test():
    s = Servo42CTCPUartBridge(UART_BRIDGE_IP, UART_BRIDGE_PORT)
    s.connect()
    while True:
        try:
            while True:
                if not s.save_or_clear_status_cmd(Servo42CTCPUartBridge.SaveOrClearStatus.CLEAR):
                    print("Failed to clear status")

                if not s.set_constant_speed(Servo42CTCPUartBridge.Direction.CLOCKWISE, 126):
                    print("Failed to set constant speed")
                sleep(1)

                if not s.stop():
                    print("Failed to stop")
                sleep(1)

                if not s.set_angle(Servo42CTCPUartBridge.Direction.CLOCKWISE, 100, 200*16):
                    print("Failed to set angle")
                sleep(1)

        except OSError as e:
            print(f"Error: Could not connect to the UART bridge: {e}")
            sleep(1)
            s.connect()

if __name__ == "__main__":
    run_uart_test()