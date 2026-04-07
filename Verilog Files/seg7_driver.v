//============================================================================
// Module: seg7_driver
// Description: 7-Segment Display Driver for Nexys A7 (Common Anode)
//
// Displays an 8-bit value as two HEX digits on the rightmost two
// 7-segment displays (AN[0] and AN[1]).
// All other anodes (AN[2:7]) are kept off.
//
// Nexys A7 Display Characteristics:
//   - Common Anode: Anode HIGH = digit OFF, Anode LOW = digit ON
//   - Active-Low Cathodes: Cathode LOW = segment ON
//   - 8 digits total, time-multiplexed
//   - Refresh rate: ~1 kHz (100 MHz / 100000) to avoid flicker
//
// Segment Encoding (active-low):
//          a
//        -----
//   f   |     |  b
//        --g--
//   e   |     |  c
//        -----
//          d      . dp
//
//   seg[6:0] = {a, b, c, d, e, f, g}
//============================================================================

module seg7_driver (
    input  wire       clk,          // System clock (100 MHz)
    input  wire       rst_n,        // Active-low synchronous reset
    input  wire [7:0] display_data, // 8-bit value to display as hex
    output reg  [6:0] seg,          // Cathode outputs {a,b,c,d,e,f,g} (active low)
    output reg  [7:0] an,           // Anode outputs (active low)
    output wire       dp            // Decimal point (always off = HIGH)
);

    assign dp = 1'b1;  // Decimal point off (active low, so HIGH = off)

    //------------------------------------------------------------------------
    // Refresh Counter - generates ~1 kHz multiplex clock
    //------------------------------------------------------------------------
    localparam REFRESH_COUNT = 100_000;  // 100 MHz / 100000 = 1 kHz

    reg [16:0] refresh_cnt;
    reg        digit_sel;  // 0 = low nibble (AN[0]), 1 = high nibble (AN[1])

    always @(posedge clk) begin
        if (!rst_n) begin
            refresh_cnt <= 17'd0;
            digit_sel   <= 1'b0;
        end else if (refresh_cnt == REFRESH_COUNT - 1) begin
            refresh_cnt <= 17'd0;
            digit_sel   <= ~digit_sel;
        end else begin
            refresh_cnt <= refresh_cnt + 17'd1;
        end
    end

    //------------------------------------------------------------------------
    // Anode Control - activate one digit at a time
    //------------------------------------------------------------------------
    always @(*) begin
        case (digit_sel)
            1'b0: an = 8'b1111_1110;  // AN[0] active (low nibble)
            1'b1: an = 8'b1111_1101;  // AN[1] active (high nibble)
        endcase
    end

    //------------------------------------------------------------------------
    // Nibble Selection
    //------------------------------------------------------------------------
    reg [3:0] current_nibble;

    always @(*) begin
        case (digit_sel)
            1'b0: current_nibble = display_data[3:0];  // Low nibble
            1'b1: current_nibble = display_data[7:4];  // High nibble
        endcase
    end

    //------------------------------------------------------------------------
    // Hex to 7-Segment Decoder (active-low outputs)
    // seg = {a, b, c, d, e, f, g}
    //------------------------------------------------------------------------
    always @(*) begin
        case (current_nibble)
            //                       abcdefg
            4'h0: seg = 7'b000_0001;  // 0
            4'h1: seg = 7'b100_1111;  // 1
            4'h2: seg = 7'b001_0010;  // 2
            4'h3: seg = 7'b000_0110;  // 3
            4'h4: seg = 7'b100_1100;  // 4
            4'h5: seg = 7'b010_0100;  // 5
            4'h6: seg = 7'b010_0000;  // 6
            4'h7: seg = 7'b000_1111;  // 7
            4'h8: seg = 7'b000_0000;  // 8
            4'h9: seg = 7'b000_0100;  // 9
            4'hA: seg = 7'b000_1000;  // A
            4'hB: seg = 7'b110_0000;  // b
            4'hC: seg = 7'b011_0001;  // C
            4'hD: seg = 7'b100_0010;  // d
            4'hE: seg = 7'b011_0000;  // E
            4'hF: seg = 7'b011_1000;  // F
            default: seg = 7'b111_1111; // All off
        endcase
    end

endmodule
