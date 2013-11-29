uhs20_stm32
===========

USB Host library for STM32 Cortex M3 series MCU with on-chip USB PHY. 

This library has been tested on STM32F205, and should work on other ST's Cortex MCU with minor changing.
This library is a porting of the great USB Host Library founded by Circuits@Home (http://www.circuitsathome.com/). The original UHS_2.0 is running on AVR and using an external USB host chip - MAX3421.
This library also has intergrated some ST's USB BSP files, especially the part of initilization and interrupt handler.

To have a quick overview of the code, I suggest you start from app/testusbhostXXX.cpp
