# stm32f303-voltage-drop-capturing
A small attempt to evaluate AC internal resistance of the battery. Currently without current measuring.

## Description
Tested on STM32F3Discovery development board.

Battery voltage input pin is currently configured as PA1, which is a 3.3 V pin, generally a voltage divider is needed. Minimum voltage drop that can be captured is defined by macro `VDecMin`.

The macro `VRefInt` (other than the factory calibration value stored in 0x1FFFF7BA) defines the internal reference voltage inside the chip, it might need to be measured in different seasons:
```
VRefInt = VRefInt_orginal * (VActual / VOutput)
```

To test the program, make sure that PB3 is connected to TRACESWO of the debug interface, choose SW port and enable Trace in debug adapter settings (Trace Clock Frequency might need to be slowed down to that of SW port) on PC with the debug adapter connected, enable ITM Port 0, then start debug session to receive printf data.

## Known Problems
1. Plugging in the PC's power adapter might have serious impact on ADC accuracy.
2. The debug session might have effect on ADC readings. it has no significant influence on the result, though.
