<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

This project implements a finite state machine (FSM) using a combination of a Moore and Mealy model. The Moore FSM sets the state output based on C1 and C2 inputs, while the Mealy FSM generates the final output Ca based on the current state and the input I.

The outputs can be observed on the uo_out[1:0] pins. The FSM transitions and outputs are synchronized with the input clock.


## How to test

Explain how to use your project

## External hardware

List external hardware used in your project (e.g. PMOD, LED display, etc), if any
