# rc2grbl

An arduino project that allows to control GRBL with an RC remote.

It's created for Arduino Leonardo or Pro Micro that have 2 UARTs. 
One UART is used to communicte with grbl, other (USB-UART) is used for PC control.

The sketch has ability to record positions in a queue and replay the positions so that grbl executes recorded trajectory.
