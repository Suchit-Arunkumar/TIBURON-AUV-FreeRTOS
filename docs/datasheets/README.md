# Datasheets

The PDFs are not stored here: they are the manufacturers' copyrighted
documents (the Wayfinder guide also carries a US export-control notice), so
this page lists which document each driver follows and the numbers taken
from it. Get the PDFs from the manufacturers.

| Part | Document | Used for |
|---|---|---|
| VectorNav VN-200 | VN-200 User Manual, UM004 | binary output format (sec. 5.3), common-group fields (5.4.4, 5.4.6, 5.4.9), CRC-16 (4.4.3), output register 75, write-settings command (6.1.3) |
| Teledyne Wayfinder DVL | Wayfinder DVL Guide, P/N 95P-6001-00 (Sept 2023); [Wayfinder Binary Interface Packet Protocol](https://www.teledynemarine.com/en-us/support/Pages/Wayfinder-Binary-Interface-Packet-Protocol.aspx) (web page); [Wayfinder Python driver](https://teledynemarine.com/en-us/support/Pages/WAYFINDER-DVL-DRIVER.aspx) | NaN = no bottom lock, ping timing, default baud 115200; frame field list and sizes |
| TE MS5837-30BA (in the Blue Robotics Bar30) | MS5837-30BA datasheet, REV C2 12/2019 | commands (p. 7), conversion time (p. 2: 9.04 ms max at OSR 4096), CRC-4 (p. 10), compensation and worked example (p. 11-12), SCL max 400 kHz |
| CEVA / Hillcrest BNO085 | BNO085 datasheet; SH-2 reference manual | I2C addresses 0x4A / 0x4B; the protocol itself is CEVA's SH-2 library in `Middlewares/Third_Party/sh2` |
| DFRobot SEN0257 water pressure sensor | [product wiki](https://wiki.dfrobot.com/Gravity__Water_Pressure_Sensor_SKU__SEN0257) | 0-1 MPa, 0.5-4.5 V, 5 V supply, 0.5-1 % FS accuracy, example `P = (V - offset) x 250` |
| Blue Robotics T200 | T200 thruster datasheet | 1100-1900 us command range, 1500 us stop |
| STM32F446RE | RM0390 reference manual; DS10693 datasheet | DMA request mapping, I2C master receive sequences (RM0390 24.3.3), timer and UART registers, VREFINT calibration address |
