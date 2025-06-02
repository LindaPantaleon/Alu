/*
 * Copyright (c) 2024 Linda Pantaleón
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_tu_nombre (
    input  wire [7:0] ui_in,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    output wire [7:0] uo_out,
    input  wire rst_n,
    input  wire ena
);

    wire [7:0] A = ui_in[3:0];     // A: 4 bits
    wire [7:0] B = ui_in[7:4];     // B: 4 bits
    wire [3:0] s_amt = ui_in[3:0]; // Cantidad de desplazamiento
    wire [2:0] ALU_control = {uio_in[2], uio_in[1], uio_in[0]};

    wire [7:0] RESULT;
    wire ZERO, NEGATIVE, CARRY, OVERFLOW;

    ALU alu_inst (
        .A(A),
        .B(B),
        .s_amt(s_amt),
        .ALU_control(ALU_control),
        .RESULT(RESULT),
        .ZERO(ZERO),
        .NEGATIVE(NEGATIVE),
        .CARRY(CARRY),
        .OVERFLOW(OVERFLOW)
    );

    assign uo_out = RESULT;       // Mostrar el resultado en los LEDs
    assign uio_out = 8'b0;
    assign uio_oe = 8'b0;

endmodule

module ALU (
    input  wire [7:0] A,
    input  wire [7:0] B,
    input  wire [3:0] s_amt,
    input  wire [2:0] ALU_control,
    output reg  [7:0] RESULT,
    output wire ZERO,
    output wire NEGATIVE,
    output wire CARRY,
    output wire OVERFLOW
);

    wire [7:0] mux;
    wire       Cin;
    wire [7:0] S;
    wire       Cout;
    wire [7:0] shiftL_out;
    wire [7:0] shiftR_out;
    wire       C1;
    wire       X, Y;

    assign Cin = (ALU_control == 3'b001 || ALU_control == 3'b101 || ALU_control == 3'b111);
    assign mux = Cin ? ~B : B;

    Prefix_adder ADDER (
        .A(A),
        .B(mux),
        .Cin(Cin),
        .S(S),
        .Cout(Cout)
    );

    shift_left shL (
        .A(S),
        .s_amt(s_amt),
        .Y(shiftL_out)
    );

    shift_right shR (
        .A(S),
        .s_amt(s_amt),
        .Y(shiftR_out)
    );

    always @(*) begin
        case (ALU_control)
            3'b000: RESULT = S;
            3'b001: RESULT = S;
            3'b010: RESULT = A & B;
            3'b011: RESULT = A | B;
            3'b100: RESULT = shiftL_out;
            3'b101: RESULT = shiftL_out;
            3'b110: RESULT = shiftR_out;
            3'b111: RESULT = shiftR_out;
            default: RESULT = 8'b0;
        endcase
    end

    assign C1       = (ALU_control == 3'b010); 
    assign X        = A[7] ^ S[7];
    assign Y        = ~(A[7] ^ B[7] ^ Cin);
    assign ZERO     = (RESULT == 8'b0);
    assign NEGATIVE = RESULT[7];
    assign CARRY    = Cout & ~C1;
    assign OVERFLOW = X & Y & ~C1;
endmodule

module Prefix_adder (
    input  wire [7:0] A,
    input  wire [7:0] B,
    input  wire       Cin,
    output wire [7:0] S,
    output wire       Cout
);

    wire [7:0] G, P, X;
    wire [8:0] C;

    assign X = A ^ B;
    assign G = A & B;
    assign P = A | B;

    wire [7:0] G1, P1, G2, P2, G3, P3;

    // 1er nivel
    assign G1[0] = G[0]; assign P1[0] = P[0];
    assign G1[1] = G[1]; assign P1[1] = P[1];
    assign G1[2] = G[2] | (P[2] & G[1]);  assign P1[2] = P[2] & P[1];
    assign G1[3] = G[3]; assign P1[3] = P[3];
    assign G1[4] = G[4] | (P[4] & G[3]);  assign P1[4] = P[4] & P[3];
    assign G1[5] = G[5]; assign P1[5] = P[5];
    assign G1[6] = G[6] | (P[6] & G[5]);  assign P1[6] = P[6] & P[5];
    assign G1[7] = G[7]; assign P1[7] = P[7];

    // 2do nivel
    assign G2[0] = G1[0]; assign P2[0] = P1[0];
    assign G2[1] = G1[1]; assign P2[1] = P1[1];
    assign G2[2] = G1[2]; assign P2[2] = P1[2];
    assign G2[3] = G1[3] | (P1[3] & G1[2]); assign P2[3] = P1[3] & P1[2];
    assign G2[4] = G1[4]; assign P2[4] = P1[4];
    assign G2[5] = G1[5] | (P1[5] & G1[4]); assign P2[5] = P1[5] & P1[4];
    assign G2[6] = G1[6]; assign P2[6] = P1[6];
    assign G2[7] = G1[7] | (P1[7] & G1[6]); assign P2[7] = P1[7] & P1[6];

    // 3er nivel
    assign G3 = G2;
    assign P3 = P2;

    // Carries
    assign C[0] = Cin;
    assign C[1] = G[0] | (P[0] & C[0]);
    assign C[2] = G1[1] | (P1[1] & C[1]);
    assign C[3] = G1[2] | (P1[2] & C[2]);
    assign C[4] = G2[3] | (P2[3] & C[3]);
    assign C[5] = G2[4] | (P2[4] & C[4]);
    assign C[6] = G2[5] | (P2[5] & C[5]);
    assign C[7] = G2[6] | (P2[6] & C[6]);
    assign C[8] = G2[7] | (P2[7] & C[7]);

    assign S = X ^ C[7:0];
    assign Cout = C[8];

endmodule

module shift_left (
    input  wire [7:0] A,
    input  wire [3:0] s_amt,
    output wire [7:0] Y
);
    assign Y = A << s_amt;
endmodule

module shift_right (
    input  wire [7:0] A,
    input  wire [3:0] s_amt,
    output wire [7:0] Y
);
    assign Y = A >> s_amt;
endmodule
