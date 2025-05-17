# pyservo42c

This is a library for interfacing to the MKS Servo42C closed-loop stepper via UART.

See https://github.com/makerbase-mks/MKS-SERVO42C for details on the hardware.

## Modules

### servo42c

This provides methods that generate and consume byte arrays for commanding the
servo and interpreting results.

### servo42cTCPUartBridge

This wraps servo42c and provides an interface to a UART-over-TCP bridge, used for testing.

### servo42cUart

This wraps servo42c and provides a direct interface to a UART port. It doesn't exist yet.