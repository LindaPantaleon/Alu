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

1. Proporciona los operandos A y B de 8 bits por medio de las entradas `ui[0]` a `ui[7]` y `uio[0]` a `uio[7]`.
2. Selecciona la operación usando bits de control (por ejemplo, `uio[6:7]`).
3. El resultado se mostrará en `uo[0]` a `uo[7]`.
4. Cambia las entradas y verifica que las salidas corresponden a la operación esperada.

## External hardware

List external hardware used in your project (e.g. PMOD, LED display, etc), if any
