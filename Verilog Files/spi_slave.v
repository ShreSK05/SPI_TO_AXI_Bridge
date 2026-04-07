//============================================================================
// Module: spi_slave
// Description: SPI Slave (Mode 0: CPOL=0, CPHA=0)
//              Receives SPI data from the internal SPI Master.
//              Decodes CMD, ADDR, DATA bytes.
//              Sends read data back via MISO.
//
// Cross-Clock Domain:
//   SPI signals (sclk, mosi, cs_n) are in the SPI clock domain.
//   This module synchronizes them to the system clock domain using
//   double-flop synchronizers and edge detectors.
//
// Protocol:
//   Byte 0: CMD   - CMD[7] = 1 → Write, CMD[7] = 0 → Read
//   Byte 1: ADDR  - Register address (8-bit)
//   Byte 2: DATA  - Write data (for writes) / Read response (for reads)
//============================================================================

module spi_slave (
    input  wire       clk,        // System clock (100 MHz)
    input  wire       rst_n,      // Active-low synchronous reset
    // SPI signals
    input  wire       sclk,       // SPI clock from master
    input  wire       mosi,       // Master Out Slave In
    output reg        miso,       // Master In Slave Out
    input  wire       cs_n,       // Chip select (active low)
    // Decoded command interface
    output reg  [7:0] cmd,        // Decoded command byte
    output reg  [7:0] addr,       // Decoded address byte
    output reg  [7:0] wdata,      // Decoded write data byte
    input  wire [7:0] rdata,      // Read data to send back to master
    output reg        cmd_valid,  // Pulses when full command is received
    output reg        is_write    // 1 = write command, 0 = read command
);

    //------------------------------------------------------------------------
    // Double-Flop Synchronizers (SPI domain → System domain)
    //------------------------------------------------------------------------
    reg sclk_d1, sclk_d2, sclk_d3;
    reg mosi_d1, mosi_d2;
    reg cs_n_d1, cs_n_d2;

    always @(posedge clk) begin
        if (!rst_n) begin
            sclk_d1 <= 1'b0;  sclk_d2 <= 1'b0;  sclk_d3 <= 1'b0;
            mosi_d1 <= 1'b0;  mosi_d2 <= 1'b0;
            cs_n_d1 <= 1'b1;  cs_n_d2 <= 1'b1;
        end else begin
            sclk_d1 <= sclk;   sclk_d2 <= sclk_d1;  sclk_d3 <= sclk_d2;
            mosi_d1 <= mosi;   mosi_d2 <= mosi_d1;
            cs_n_d1 <= cs_n;   cs_n_d2 <= cs_n_d1;
        end
    end

    //------------------------------------------------------------------------
    // Edge Detectors
    //------------------------------------------------------------------------
    wire sclk_rising  = (sclk_d2 == 1'b1) && (sclk_d3 == 1'b0);
    wire sclk_falling = (sclk_d2 == 1'b0) && (sclk_d3 == 1'b1);
    wire cs_active    = ~cs_n_d2;

    //------------------------------------------------------------------------
    // Bit Counter and Shift Registers
    //------------------------------------------------------------------------
    reg [4:0] bit_cnt;         // Counts bits received (0 to 23)
    reg [7:0] shift_reg_in;    // Shift register for incoming data
    reg [7:0] shift_reg_out;   // Shift register for outgoing data (MISO)
    reg       cmd_latched;     // CMD[7] has been decoded
    reg       addr_latched;    // ADDR has been latched
    reg [7:0] cmd_reg;         // Internal CMD register
    reg [7:0] addr_reg;        // Internal ADDR register

    //------------------------------------------------------------------------
    // Main Logic
    //------------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            bit_cnt       <= 5'd0;
            shift_reg_in  <= 8'd0;
            shift_reg_out <= 8'd0;
            cmd           <= 8'd0;
            addr          <= 8'd0;
            wdata         <= 8'd0;
            cmd_valid     <= 1'b0;
            is_write      <= 1'b0;
            miso          <= 1'b0;
            cmd_latched   <= 1'b0;
            addr_latched  <= 1'b0;
            cmd_reg       <= 8'd0;
            addr_reg      <= 8'd0;
        end else begin
            cmd_valid <= 1'b0;  // Default: de-assert

            if (!cs_active) begin
                //------------------------------------------------------------
                // CS is inactive - reset state for next transaction
                //------------------------------------------------------------
                bit_cnt       <= 5'd0;
                shift_reg_in  <= 8'd0;
                shift_reg_out <= 8'd0;
                cmd_latched   <= 1'b0;
                addr_latched  <= 1'b0;
                miso          <= 1'b0;
            end else begin
                //------------------------------------------------------------
                // CS is active - process SPI data
                //------------------------------------------------------------

                // Rising edge of SCLK: sample MOSI
                if (sclk_rising) begin
                    shift_reg_in <= {shift_reg_in[6:0], mosi_d2};
                    bit_cnt <= bit_cnt + 5'd1;

                    // After 8 bits: CMD byte received
                    if (bit_cnt == 5'd7) begin
                        cmd_reg     <= {shift_reg_in[6:0], mosi_d2};
                        cmd_latched <= 1'b1;
                    end

                    // After 16 bits: ADDR byte received
                    if (bit_cnt == 5'd15) begin
                        addr_reg     <= {shift_reg_in[6:0], mosi_d2};
                        addr_latched <= 1'b1;
                        // For READ: load outgoing shift register with read data
                        if (!cmd_reg[7]) begin
                            shift_reg_out <= rdata;
                        end
                    end

                    // After 24 bits: DATA byte received (write) or transfer complete
                    if (bit_cnt == 5'd23) begin
                        cmd       <= cmd_reg;
                        addr      <= addr_reg;
                        is_write  <= cmd_reg[7];
                        if (cmd_reg[7]) begin
                            // Write: capture data byte
                            wdata <= {shift_reg_in[6:0], mosi_d2};
                        end
                        cmd_valid <= 1'b1;
                    end
                end

                // Falling edge of SCLK: shift out MISO (for read responses)
                if (sclk_falling) begin
                    if (addr_latched && !cmd_reg[7]) begin
                        // Read mode: drive MISO with MSB of outgoing shift register
                        miso          <= shift_reg_out[7];
                        shift_reg_out <= {shift_reg_out[6:0], 1'b0};
                    end
                end
            end
        end
    end

endmodule
