# SAM-BA-Monitor_SAMD21G18A

The SAM Boot Assistant (SAM-BA ® ) allows In-system Programming (ISP) using a USB or UART host without any external programming interface.
In general, SAM-BA Monitor is factory programmed to ROM, if it exists. If ROM does not exist, by default, SAM-BA is not supported.
To support SAM-BA in ROMless devices, the SAM-BA Monitor application can be loaded in the Flash memory.

For additional information read [this documentation](https://ww1.microchip.com/downloads/en/DeviceDoc/00002565A.pdf).

For this purpose i was modify the sam-ba_monitor_rom-less project to support ATSAMD21G18A chip.
This project supports only uart. Hope it will helps you and saves your time.

Follow this steps to get started:
1. Clone repository.
2. Open it and in Build menu click Clean solution and then build solution. 
3. Go to  Tools->Device Programming.
4. Click apply and then read device signature.
5. Click Memories then erase chip
6. Choose the brand new hex file in Debug folder and click programm. (path to .hex file must be in english language!!!)
7. Open sam-ba 2.18 and choose the com-port you are connected to.
8. Select SAMD21 Xplained Pro board and click connect.
9. You are welcome!

NOTE: Dont forget to change device in the project->properties->Device->Change device.

I will update the project during development process. Stay tuned. 

