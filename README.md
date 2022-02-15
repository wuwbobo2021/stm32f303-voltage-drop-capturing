# stm32f303-voltage-drop-capturing
A small attempt to evaluate AC internal resistance of the battery. Currently without current measuring.
## Description
Battery voltage input pin is currently configured as PA1, which is a 3.3 V pin. Minimum voltage drop that can be captured is defined by macro `VDecMin`. The macro `VRefInt` (other than the value stored in 0x1FFFF7BA) defines the internal reference voltage inside the chip, it might need to be measured in different seasons.
## Known Problem
The debug session might have effect on ADC readings.
