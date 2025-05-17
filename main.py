#!/usr/bin/env python3

from time import sleep
from servo42cUartBridge import Servo42CUartBridge

UART_BRIDGE_IP = "uart-bridge-1.local"
UART_BRIDGE_PORT = 6638

def run_uart_test():
    s = Servo42CUartBridge(UART_BRIDGE_IP, UART_BRIDGE_PORT)
    s.connect()
    while True:
        try:
            s.save_or_clear_status_cmd(Servo42CUartBridge.SaveOrClearStatus.CLEAR)
            direction = 0
            while True:
                # s.set_angle(Servo42CUartBridge.Direction.CLOCKWISE, 100, 200*16)
                if not s.set_constant_speed(Servo42CUartBridge.Direction.CLOCKWISE, 126):
                    print("Failed to set constant speed")

                sleep(0.5)
                s.stop()
                sleep(5)
        except OSError as e:
            print(f"Error: Could not connect to the UART bridge: {e}")
            sleep(1)
            s.connect()

if __name__ == "__main__":
    run_uart_test()