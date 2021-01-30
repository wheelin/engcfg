# EngCfg

## Introduction

This crate allows to generate 4-stroke engine waveforms for direct writing on GPIO as bit mask.
The goal is to generate a pulse train with the following information:
* crankshaft wheel signal
* camshaft wheel signal
* top-dead-center piston position for each cylinder.

## Technical information

The pulse train buffer is an array of [`EngBit`] (generally u8, u16 or u32, depending on the GPIO register width) with length 7200. An array has been used
in order to be compatible with DMA (Direct Memory Access) mechanisms. The goal
of this crate is really to create crank/cam/tdc generators for ECU development purpose.
Each array element is a bit field representing the current crank, cam, tdc state, at a precise
position (the array index) in the engine cycle:

Here is the implementation done for `u8` in this crate:

| Bit | Desc.     |
|:----|-----------|
|0    | Camshaft  |
|1    | Crankshaft|
|2    | TDC cyl. 1|
|3    | TDC cyl. 2|
|4    | TDC cyl. 3|
|5    | TDC cyl. 4|
|6    | TDC cyl. 5|
|7    | TDC cyl. 6|

When generated, the pulse train array shall be applied on output pins using a strict timing:
* either by using interrupts and an hardware timer, by writing each element of the array on the GPIO port one by one and restarting to 0 at the end.
* or by using a DMA with circular configuration and timer triggering. This way, the CPU has really nothing to do but reconfigure the timer in case of engine speed changes.

## Limitations

The following engines can not be generated at the moment:
* Engines with more than 6 cylinders
* Asymmetrical engines (TDCs are not spaced evenly)
* The concept uses a relatively high amount of RAM. But with the use of appropriate DMA and timers, the pulse train generation should not even require CPU processing.
