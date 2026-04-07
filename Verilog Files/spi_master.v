//============================================================================
// Module: spi_master
// Description: Internal SPI Master (Mode 0: CPOL=0, CPHA=0)
//              Generates SPI transactions for write (3 bytes) and read
//              (2 bytes out + 1 byte in) operations.
//
// SPI Protocol:
//   WRITE: CS_N low → shift out CMD[7:0] + ADDR[7:0] + DATA[7:0] → CS_N high
//   READ:  CS_N low → shift out CMD[7:0] + ADDR[7:0] + shift in DATA[7:0] → CS_N high
//   CMD[7] = 1 → WRITE, CMD[7] = 0 → READ
//
// SPI Mode 0 Timing:
//   - SCLK idles LOW
//   - Data is shifted out on FALLING edge of SCLK
//   - Data is sampled on RISING edge of SCLK
//
// Parameters:
//   CLK_DIV - SCLK = clk / (2 * CLK_DIV).  Default 50 → 1 MHz @ 100 MHz clk
//============================================================================

module spi_master #(
    parameter CLK_DIV = 50   // SCLK frequency = clk / (2 * CLK_DIV)
)(
    input  wire       clk,       // System clock (100 MHz)
    input  wire       rst_n,     // Active-low synchronous reset
    // Command interface
    input  wire       start,     // Pulse high for 1 cycle to begin transaction
    input  wire       rw,        // 1 = write, 0 = read
    input  wire [7:0] addr,      // Target register address
    input  wire [7:0] wdata,     // Write data (ignored during read)
    output reg  [7:0] rdata,     // Read data (valid when done is asserted)
    output reg        done,      // Pulses high for 1 cycle when transaction completes
    // SPI signals
    output reg        sclk,      // SPI clock
    output reg        mosi,      // Master Out Slave In
    input  wire       miso,      // Master In Slave Out
    output reg        cs_n       // Chip select (active low)
);

    //------------------------------------------------------------------------
    // FSM State Encoding
    //------------------------------------------------------------------------
    localparam [2:0] S_IDLE  = 3'd0,  // Waiting for start
                     S_LOAD  = 3'd1,  // Load shift register
                     S_LOW   = 3'd2,  // SCLK low phase (shift data out)
                     S_HIGH  = 3'd3,  // SCLK high phase (sample data in)
                     S_DONE  = 3'd4;  // Transaction complete

    reg [2:0] state, next_state;

    //------------------------------------------------------------------------
    // Internal Registers
    //------------------------------------------------------------------------
    reg [23:0] shift_out;    // Outgoing data shift register (CMD + ADDR + DATA)
    reg  [7:0] shift_in;     // Incoming data shift register (read data)
    reg  [4:0] bit_cnt;      // Bit counter (0 to 23)
    reg  [4:0] total_bits;   // Total bits to transfer (24 for write, 24 for read)
    reg [15:0] clk_cnt;      // Clock divider counter
    reg        is_write;     // Latched write/read flag

    //------------------------------------------------------------------------
    // Clock Divider - generates half-period ticks
    //------------------------------------------------------------------------
    wire clk_tick = (clk_cnt == CLK_DIV - 1);

    always @(posedge clk) begin
        if (!rst_n || state == S_IDLE || state == S_LOAD)
            clk_cnt <= 16'd0;
        else if (clk_tick)
            clk_cnt <= 16'd0;
        else
            clk_cnt <= clk_cnt + 16'd1;
    end

    //------------------------------------------------------------------------
    // FSM - Sequential
    //------------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n)
            state <= S_IDLE;
        else
            state <= next_state;
    end

    //------------------------------------------------------------------------
    // FSM - Combinational Next-State Logic
    //------------------------------------------------------------------------
    always @(*) begin
        next_state = state;
        case (state)
            S_IDLE: begin
                if (start)
                    next_state = S_LOAD;
            end

            S_LOAD: begin
                next_state = S_LOW;
            end

            S_LOW: begin
                if (clk_tick)
                    next_state = S_HIGH;
            end

            S_HIGH: begin
                if (clk_tick) begin
                    if (bit_cnt == total_bits - 1)
                        next_state = S_DONE;
                    else
                        next_state = S_LOW;
                end
            end

            S_DONE: begin
                next_state = S_IDLE;
            end

            default: next_state = S_IDLE;
        endcase
    end

    //------------------------------------------------------------------------
    // Datapath - Sequential Logic
    //------------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            cs_n      <= 1'b1;
            sclk      <= 1'b0;
            mosi      <= 1'b0;
            done      <= 1'b0;
            rdata     <= 8'd0;
            shift_out <= 24'd0;
            shift_in  <= 8'd0;
            bit_cnt   <= 5'd0;
            total_bits<= 5'd0;
            is_write  <= 1'b0;
        end else begin
            done <= 1'b0;  // Default: de-assert done

            case (state)
                S_IDLE: begin
                    cs_n <= 1'b1;
                    sclk <= 1'b0;
                    mosi <= 1'b0;
                end

                S_LOAD: begin
                    // Assert chip select
                    cs_n <= 1'b0;
                    sclk <= 1'b0;
                    // Latch direction
                    is_write <= rw;
                    // Build outgoing shift register
                    if (rw) begin
                        // WRITE: CMD(0x80 | addr[6:0] bits) + ADDR + DATA
                        shift_out <= {1'b1, addr[6:0], addr, wdata};
                        total_bits <= 5'd24;
                    end else begin
                        // READ:  CMD(0x00 | addr[6:0] bits) + ADDR + 8'h00 (dummy)
                        shift_out <= {1'b0, addr[6:0], addr, 8'h00};
                        total_bits <= 5'd24;
                    end
                    shift_in  <= 8'd0;
                    bit_cnt   <= 5'd0;
                    // Drive first bit on MOSI immediately
                    if (rw)
                        mosi <= 1'b1;  // CMD[7] = 1 for write
                    else
                        mosi <= 1'b0;  // CMD[7] = 0 for read
                end

                S_LOW: begin
                    // SCLK goes low - shift out next bit
                    sclk <= 1'b0;
                    if (clk_tick) begin
                        // Drive MOSI with current MSB of shift register
                        mosi <= shift_out[23];
                    end
                end

                S_HIGH: begin
                    // SCLK goes high - sample MISO
                    if (clk_cnt == 16'd0) begin
                        sclk <= 1'b1;
                    end
                    if (clk_tick) begin
                        // Sample MISO - always capture (relevant for read during byte 3)
                        shift_in <= {shift_in[6:0], miso};
                        // Shift out register left
                        shift_out <= {shift_out[22:0], 1'b0};
                        // Increment bit counter
                        bit_cnt <= bit_cnt + 5'd1;
                    end
                end

                S_DONE: begin
                    cs_n <= 1'b1;
                    sclk <= 1'b0;
                    done <= 1'b1;
                    if (!is_write) begin
                        rdata <= shift_in;  // Capture read data
                    end
                end
            endcase
        end
    end

endmodule
