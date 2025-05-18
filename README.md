# pyservo42c

This is a library for interfacing to the MKS Servo42C closed-loop steppe motor via UART.

See https://github.com/makerbase-mks/MKS-SERVO42C for details on the hardware.

Note the serial protocol spec claims to include a checksum at the end of response messages. This doesn't seem to be true in practice. Also the set-angle command only accepts two bytes in the "pulse number" field, not the four bytes the spec claims.

Many of the commands are implemented, but not all. Pull requests encouraged!

## Modules

### servo42c

This provides methods that generate and consume byte arrays for commanding the
servo and interpreting results.

### servo42cTCPUartBridge

This wraps servo42c and provides an interface to a UART-over-TCP bridge, used for testing.

### servo42cUart

This wraps servo42c and provides a direct interface to a UART port.