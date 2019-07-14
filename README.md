# Dynamixel AX-12A Robot Actuator

[![Datasheet](https://static.generation-robots.com/3053/dynamixel-ax-12a-servomotor.jpg)](http://www.trossenrobotics.com/images/productdownloads/AX-12(English).pdf)

[AX-12A Datasheet][Datasheet]

## Installing the library
- Download zip file
- Extract zip
- Rename the folder : AX-12A-servo-library-master --> AX-12A-servo-library
- Put the folder into your library folder (usually Documents/Arduino/libraries)

## Using the library
To use the library, include header file located in the src folder.

```
#include "AX12A.h"
```

The library is provided with 4 Arduino examples :
- **Blink** : simpliest example to blink the built-in LED
- **Move** : example showing how to set a goal position
- **EndlessTurn** : example of endlessTurn mode (wheel mode)
- **readRegister** : debug AX-12A by printing all its registers

You have 4 lines to configure :
```
#define DirectionPin  (10u)
#define BaudRate      (1000000ul)
#define ID            (1u)
...
AX12A.begin(BaudRate, DirectionPin, &Serial);
```

`DirectionPin` is the GPIO used to change communication direction.

`BaudRate` corresponds to the baud rate used to communicate with the servo.

`ID` corresponds to the ID of the servo (from 0 to 253, 254 is used for broadcast).

`&Serial` refers to the serial port you want to use. It may have to be changed to `&Serial1`, `&SerialUSB` or other depending on the board package you use (variant files).


Change AX-12A settings by connecting the servo to a PC via [USB2Dynamixel][USB2DXL].

To do so, install [Dynamixel Wizard][RoboPlus] included in the RoboPlus software suite.

Follow [these instructions][Dynamixel Wizard] to connect and access the AX-12A settings.

## Baud rate tests

We tested the library on differents boards at the most common baud rates.

Some boards cannot reach the desired baud rate :

| Address 0x04 | Baud rate | Feather M0 | Feather 32u4 | nRF52 | STM32L4 |
| ------------ | --------- | ---------- | ------------ | ----- | ------- | 
| 1	           | 1000000   | ok         | ok           | ok    | fail    |
| 3            | 500000    | ok         | ok           | fail  | ok      |
| 4            | 400000    | ok         | fail         | fail  | ok      |
| 7            | 250000    | ok         | ok           | ok    | ok      |
| 9            | 200000    | ok         | ok           | fail  | ok      |
| 16           | 115200    | ok         | ok           | ok    | ok      |
| 34           | 57600     | ok         | fail         | ok    | ok      |
| 103          | 19200     | ok         | ok           | ok    | ok      |
| 207          | 9600      | ok         | ok           | ok    | ok      |



[USB2DXL]: <http://support.robotis.com/en/product/auxdevice/interface/usb2dxl_manual.htm>
[RoboPlus]: <http://en.robotis.com/BlueAD/board.php?bbs_id=downloads&scate=SOFTWARE>
[Dynamixel Wizard]: <http://support.robotis.com/en/software/roboplus/dynamixel_monitor.htm>
[Datasheet]: <http://www.trossenrobotics.com/images/productdownloads/AX-12(English).pdf>
