# AVR-BME-DOGM
This project focuses on implementing real-time serial communication between a microcontroller, a temperature/gas pressure sensor, and a LCD display. The microcontroller reads and decodes data from the sensor, then generates a message to be displayed on the LCD. The message contains the temperature, gas pressure, and humidity readings. Program written in embedded C, interdevice communication done with SPI protocol.

## Wiring
![image](https://github.com/HaoRanYuan0/AVR-BME-DOGM/assets/121404407/a0a42441-230c-44cc-98ef-b563e72ca5ec)
('J1' the DOGM204 LCD display)


## Devices used + Datasheets
1. Microcontroller: [AVR128DB48](https://ww1.microchip.com/downloads/en/DeviceDoc/AVR128DB28-32-48-64-DataSheet-DS40002247A.pdf)
2. Sensor: [BME680](https://www.mouser.com/datasheet/2/308/1/BST_BME680_DS001-2488287.pdf)
3. LCD Display: [DOGM204](https://www.lcd-module.com/fileadmin/eng/pdf/doma/dogm204e.pdf)
