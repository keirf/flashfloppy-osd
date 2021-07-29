|Pin| FT | A Assignment  | FT | B Assignment     |
|---|----|---------------|----|------------------|
| 0 |    | Rotary CLK    |    |                  |
| 1 |    | Rotary DAT    |    |                  |
| 2 |    | Rotary SW     |  Y | **NA** (BOOT1)   |
| 3 |    |               |  Y | Amiga KB_DAT     |
| 4 |    |               |  Y | Amiga KB_CLK     |
| 5 |    | SPI1.SCK      |    |                  |
| 6 |    |               |  Y | I2C SCL          |
| 7 |    | Disp.Out.SPI1 |  Y | I2C SDA          |
| 8 |  Y | CSYNC/HSYNC   |  Y | User Out (U0)    |
| 9 |  Y | Serial Tx     |  Y | User Out (U1)    |
|10 |  Y | Serial Rx     |  Y | User Out (U2)    |
|11 |  Y |               |  Y | Atari KB         |
|12 |  Y |               |  Y |                  |
|13 |  Y | **NA** (SWDIO)|  Y | SPI2.SCK         |
|14 |  Y | **NA** (SWCLK)|  Y | VSYNC            |
|15 |  Y | Disp.Enable   |  Y | Disp.Out. (RGB)  |
