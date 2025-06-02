/*
 * Copyright (c) 2024 Linda Pantaleón
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_top_alu(
    input  [7:0] io_in,
    output [7:0] io_out
);
    // Separar entradas
    wire [1:0] A       = io_in[1:0];
    wire [1:0] B       = io_in[3:2];
    wire [2:0] control = io_in[6:4];
    wire [0:0] s_amt   = io_in[7]; // un bit para shift amount

    // Expandimos a 8 bits para la ALU
    wire [7:0] A_ext = {6'b0, A};
    wire [7:0] B_ext = {6'b0, B};
    wire [3:0] s_amt_ext = {3'b0, s_amt};

    wire [7:0] RESULT;
    wire ZERO, NEGATIVE, CARRY, OVERFLOW;

    ALU alu (
        .A(A_ext),
        .B(B_ext),
        .s_amt(s_amt_ext),
        .ALU_control(control),
        .RESULT(RESULT),
        .ZERO(ZERO),
        .NEGATIVE(NEGATIVE),
        .CARRY(CARRY),
        .OVERFLOW(OVERFLOW)
    );

    assign io_out[3:0] = RESULT[3:0]; // solo 4 bits del resultado
    assign io_out[4]   = CARRY;
    assign io_out[5]   = ZERO;
    assign io_out[6]   = NEGATIVE;
    assign io_out[7]   = OVERFLOW;
endmodule

module ALU(
    input  [7:0] A,
    input  [7:0] B,
    input  [3:0] s_amt,
    input  [2:0] ALU_control,
    output [7:0] RESULT,
    output ZERO,
    output NEGATIVE,
    output CARRY,
    output OVERFLOW
);

    wire [7:0] mux, S, shiftL_out, shiftR_out;
    wire Cin, Cout, Ovf;
    wire C1, X, Y;
    reg [7:0] result_reg;

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
            3'b000: result_reg = S;
            3'b001: result_reg = S;
            3'b010: result_reg = A & B;
            3'b011: result_reg = A | B;
            3'b100: result_reg = shiftL_out;
            3'b101: result_reg = shiftL_out;
            3'b110: result_reg = shiftR_out;
            3'b111: result_reg = shiftR_out;
            default: result_reg = 8'b0;
        endcase
    end

    assign RESULT   = result_reg;
    assign C1       = (ALU_control == 3'b010);
    assign X        = A[7] ^ S[7];
    assign Y        = ~(A[7] ^ B[7] ^ Cin);
    assign ZERO     = (RESULT == 8'b0);
    assign NEGATIVE = RESULT[7];
    assign CARRY    = Cout & ~C1;
    assign OVERFLOW = X & Y & ~C1;

endmodule
module Prefix_adder(
    input  [7:0] A,
    input  [7:0] B,
    input        Cin,
    output [7:0] S,
    output       Cout
);

    wire [7:0] G, P, X;
    wire [8:0] C;

    wire [7:0] G1, P1, G2, P2, G3, P3;

    assign X = A ^ B;
    assign G = A & B;
    assign P = A | B;

    assign G1[0] = G[0]; assign P1[0] = P[0];
    assign G1[1] = G[1]; assign P1[1] = P[1];
    assign G1[2] = G[2] | (P[2] & G[1]); assign P1[2] = P[2] & P[1];
    assign G1[3] = G[3]; assign P1[3] = P[3];
    assign G1[4] = G[4] | (P[4] & G[3]); assign P1[4] = P[4] & P[3];
    assign G1[5] = G[5]; assign P1[5] = P[5];
    assign G1[6] = G[6] | (P[6] & G[5]); assign P1[6] = P[6] & P[5];
    assign G1[7] = G[7]; assign P1[7] = P[7];

    assign G2[0] = G1[0]; assign P2[0] = P1[0];
    assign G2[1] = G1[1]; assign P2[1] = P1[1];
    assign G2[2] = G1[2]; assign P2[2] = P1[2];
    assign G2[3] = G1[3] | (P1[3] & G1[2]); assign P2[3] = P1[3] & P1[2];
    assign G2[4] = G1[4]; assign P2[4] = P1[4];
    assign G2[5] = G1[5] | (P1[5] & G1[4]); assign P2[5] = P1[5] & P1[4];
    assign G2[6] = G1[6]; assign P2[6] = P1[6];
    assign G2[7] = G1[7] | (P1[7] & G1[6]); assign P2[7] = P1[7] & P1[6];

    assign G3 = G2;
    assign P3 = P2;

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
module shift_left(
    input  [7:0] A,
    input  [3:0] s_amt,
    output [7:0] Y
);
    assign Y = A << s_amt;
endmodule

module shift_right(
    input  [7:0] A,
    input  [3:0] s_amt,
    output [7:0] Y
);
    assign Y = A >> s_amt;
endmodule
