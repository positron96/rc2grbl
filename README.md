# rc2grbl

An arduino project that allows to control GRBL with an PS2 gamepad.

It's created for Arduino Leonardo or Pro Micro that have 2 UARTs. 
One UART is used to communicate with grbl, other (USB-UART) is used for PC control.

The sketch has ability to record positions into a queue and replay the positions so that grbl executes recorded trajectory. 
Recording and playback is controlled via buttons on gamepad
