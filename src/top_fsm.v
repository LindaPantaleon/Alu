/*
 * Copyright (c) 2024 Linda Pantaleón
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_top_fsm (
    input  wire [7:0] ui_in,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    output wire [7:0] uo_out,
    input  wire clk,
    input  wire rst_n,
    input  wire ena
);

    assign uio_out = 8'b0;
    assign uio_oe  = 8'b0;
    
    wire C1 = ui_in[0];
    wire C2 = ui_in[1];
    wire I  = ui_in[2];

    wire [1:0] Ca;

    fsm my_fsm (
        .clk(clk),
        .reset(rst_n),
        .C1(C1),
        .C2(C2),
        .I(I),
        .Ca(Ca)
    );

    assign uo_out[1:0] = Ca;
    assign uo_out[7:2] = 6'b0;

endmodule


module fsm(
    input wire clk, reset,
    input wire C1, C2, I,
    output wire [1:0] Ca
);
    wire [1:0] P;
    wire A_internal;

    fsm_moore moore_fsm (.clk(clk), .reset(reset), .C1(C1), .C2(C2), .A(A_internal), .P(P));
    fsm_mealy mealy_fsm (.clk(clk), .reset(reset), .I(I), .P(P), .A(A_internal), .Ca(Ca));
endmodule


module fsm_moore(
    input wire clk, reset,
    input wire C1, C2, A,
    output reg [1:0] P
);

    reg [1:0] state, nextstate;

    always @(posedge clk or posedge reset)
        if (reset)
            state <= 2'b00;
        else
            state <= nextstate;

    always @(*) begin
        case (state)
            2'b00: if (C1) nextstate = 2'b01;
                   else if (C2) nextstate = 2'b10;
                   else nextstate = 2'b00;
            2'b01: if (C2) nextstate = 2'b11;
                   else nextstate = 2'b01;
            2'b10: if (C1) nextstate = 2'b11;
                   else nextstate = 2'b10;
            2'b11: nextstate = 2'b11;
            default: nextstate = 2'b00;
        endcase
    end

    always @(*) begin
        case (state)
            2'b00: P = 2'b00;
            2'b01: P = 2'b01;
            2'b10: P = 2'b10;
            2'b11: P = 2'b11;
            default: P = 2'b00;
        endcase
    end
endmodule


module fsm_mealy(
    input wire clk, reset,
    input wire I,
    input wire [1:0] P,
    output reg A,
    output reg [1:0] Ca
);

    reg [1:0] state, nextstate;

    always @(posedge clk or posedge reset)
        if (reset)
            state <= 2'b00;
        else
            state <= nextstate;

    always @(*) begin
        case (state)
            2'b00: if (I && P == 2'b01) nextstate = 2'b01;
                   else if (I && P == 2'b10) nextstate = 2'b10;
                   else nextstate = 2'b00;
            2'b01: if (P == 2'b10) nextstate = 2'b10;
                   else nextstate = 2'b01;
            2'b10: nextstate = 2'b11;
            2'b11: nextstate = 2'b11;
            default: nextstate = 2'b00;
        endcase
    end

    always @(*) begin
        case (state)
            2'b00: begin A = 0; Ca = 2'b00; end
            2'b01: begin A = 1; Ca = 2'b01; end
            2'b10: begin A = 1; Ca = 2'b10; end
            2'b11: begin A = 0; Ca = 2'b11; end
            default: begin A = 0; Ca = 2'b00; end
        endcase
    end
endmodule
