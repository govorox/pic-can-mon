# pic-CAN-mon

Firmware for a PIC32 based CAN bus monitor.

=============================

Hardware: chipKIT Pro MX7
* any other board based on PIC32 with CAN controller will work
provided MCP2551 or equivalent CAN driver is also attached

=============================

Implements CAN ASCII/SLCAN protocol compatible with Lawicel CAN232/CANUSB. 

See protocol definition here http://www.can232.com/docs/can232_v3.pdf and here http://www.can232.com/docs/canusb_manual.pdf

Commands not supported/not yet implemented:  
- s, W, M, m, U.

Commands modified:
-  Z - extra Z2 option enables 4 byte timestamp vs standard 2 byte (60000ms max)
  
```
CMD | IMPLEMENTED | SYNTAX               | DESCRIPTION
------------------------------------------------------------------------------------------------------------
'S' |   YES+      |   Sn[CR]               Setup with standard CAN bit-rates where n is 0-8.
    |             |                        S0 10Kbit          S4 125Kbit         S8 1Mbit
    |             |                        S1 20Kbit          S5 250Kbit         S9 N/A
    |             |                        S2 50Kbit          S6 500Kbit
    |             |                        S3 100Kbit         S7 800Kbit
's' |    -        |   sxxyy[CR]            Setup with BTR0/BTR1 CAN bit-rates where xx and yy is a hex value.
'O' |   YES       |   O[CR]                Open the CAN channel in normal mode (sending & receiving).
'L' |   YES       |   L[CR]                Open the CAN channel in listen only mode (receiving).
'C' |   YES       |   C[CR]                Close the CAN channel.
't' |   YES       |   tiiildd...[CR]       Transmit a standard (11bit) CAN frame.
'T' |   YES       |   Tiiiiiiiildd...[CR]  Transmit an extended (29bit) CAN frame
'r' |   YES       |   riiil[CR]            Transmit an standard RTR (11bit) CAN frame.
'R' |   YES       |   Riiiiiiiil[CR]       Transmit an extended RTR (29bit) CAN frame.
'P' |   YES       |   P[CR]                Poll incomming FIFO for CAN frames (single poll)
'A' |   YES       |   A[CR]                Polls incomming FIFO for CAN frames (all pending frames)
'F' |   YES+      |   F[CR]                Read Status Flags.
'X' |   YES       |   Xn[CR]               Sets Auto Poll/Send ON/OFF for received frames.
'W' |    -        |   Wn[CR]               Filter mode setting. By default CAN232 works in dual filter mode (0) and is backwards compatible with previous CAN232 versions.
'M' |    -        |   Mxxxxxxxx[CR]        Sets Acceptance Code Register (ACn Register of SJA1000). // we use MCP2515, not supported
'm' |    -        |   mxxxxxxxx[CR]        Sets Acceptance Mask Register (AMn Register of SJA1000). // we use MCP2515, not supported
'U' |   YES       |   Un[CR]               Setup UART with a new baud rate where n is 0-6.
'V' |   YES       |   v[CR]                Get Version number of both CAN232 hardware and software
'v' |   YES       |   V[CR]                Get Version number of both CAN232 hardware and software
'N' |   YES       |   N[CR]                Get Serial number of the CAN232.
'Z' |   YES+      |   Zn[CR]               Sets Time Stamp ON/OFF for received frames only. EXTENSION to LAWICEL: Z2 - millis() timestamp w/o standard 60000ms cycle
'Q' |   YES  todo |       Qn[CR]               Auto Startup feature (from power on). 

```

