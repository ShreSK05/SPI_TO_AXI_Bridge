//============================================================================
// Module: top_spi_axi
// Description: Top-Level Module - SPI to AXI4-Lite Bridge System
//
// This module connects all sub-modules and implements a demo sequencer
// that automatically runs a series of SPI write and read transactions
// on power-up to demonstrate the full data path.
//
// Demo Sequence:
//   1. Write 0xA5 to register 0
//   2. Write 0x3C to register 1
//   3. Write 0x7E to register 2
//   4. Write 0xF0 to register 3
//   5. Read register 0 (expect 0xA5)
//   6. Read register 1 (expect 0x3C)
//   7. Read register 2 (expect 0x7E)
//   8. Read register 3 (expect 0xF0)
//   9. HALT - display results
//
// Board I/O (Nexys A7):
//   SW[0]  - Display mode: 0 = last write data, 1 = last read data
//   LED[0] - Read operation active
//   LED[1] - Write operation active
//   LED[2] - Transaction done (latched)
//   LED[3] - Error (latched)
//   LED[7:4] - Current register address
//   LED[15:8] - Current data byte
//============================================================================

module top_spi_axi (
    input  wire        CLK100MHZ,   // 100 MHz system clock
    input  wire        CPU_RESETN,  // Active-low reset (button)
    input  wire [15:0] SW,          // Switches
    output wire [15:0] LED,         // LEDs
    output wire  [6:0] SEG,         // 7-segment cathodes {a,b,c,d,e,f,g}
    output wire  [7:0] AN,          // 7-segment anodes
    output wire        DP           // 7-segment decimal point
);

    //------------------------------------------------------------------------
    // Internal Signals
    //------------------------------------------------------------------------
    wire clk   = CLK100MHZ;
    wire rst_n = CPU_RESETN;

    // SPI bus (internal wires between master and slave)
    wire spi_sclk;
    wire spi_mosi;
    wire spi_miso;
    wire spi_cs_n;

    // SPI Master control
    reg        spi_start;
    reg        spi_rw;
    reg  [7:0] spi_addr;
    reg  [7:0] spi_wdata;
    wire [7:0] spi_rdata;
    wire       spi_done;

    // SPI Slave decoded outputs
    wire [7:0] slave_cmd;
    wire [7:0] slave_addr;
    wire [7:0] slave_wdata;
    wire       slave_cmd_valid;
    wire       slave_is_write;

    // AXI Master ↔ Slave bus
    wire [31:0] axi_awaddr;
    wire        axi_awvalid;
    wire        axi_awready;
    wire [31:0] axi_wdata;
    wire  [3:0] axi_wstrb;
    wire        axi_wvalid;
    wire        axi_wready;
    wire  [1:0] axi_bresp;
    wire        axi_bvalid;
    wire        axi_bready;
    wire [31:0] axi_araddr;
    wire        axi_arvalid;
    wire        axi_arready;
    wire [31:0] axi_rdata;
    wire  [1:0] axi_rresp;
    wire        axi_rvalid;
    wire        axi_rready;

    // AXI Master command interface
    wire [7:0] axi_cmd_rdata;
    wire       axi_cmd_done;
    wire       axi_cmd_error;

    // Read data to send back to SPI Master via SPI Slave MISO
    wire [7:0] slave_rdata_to_send = axi_cmd_rdata;

    //------------------------------------------------------------------------
    // Module Instantiations
    //------------------------------------------------------------------------

    // SPI Master - generates SPI transactions
    spi_master #(
        .CLK_DIV(50)  // SCLK = 100 MHz / (2*50) = 1 MHz
    ) u_spi_master (
        .clk    (clk),
        .rst_n  (rst_n),
        .start  (spi_start),
        .rw     (spi_rw),
        .addr   (spi_addr),
        .wdata  (spi_wdata),
        .rdata  (spi_rdata),
        .done   (spi_done),
        .sclk   (spi_sclk),
        .mosi   (spi_mosi),
        .miso   (spi_miso),
        .cs_n   (spi_cs_n)
    );

    // SPI Slave - receives SPI data, decodes CMD/ADDR/DATA
    spi_slave u_spi_slave (
        .clk       (clk),
        .rst_n     (rst_n),
        .sclk      (spi_sclk),
        .mosi      (spi_mosi),
        .miso      (spi_miso),
        .cs_n      (spi_cs_n),
        .cmd       (slave_cmd),
        .addr      (slave_addr),
        .wdata     (slave_wdata),
        .rdata     (slave_rdata_to_send),
        .cmd_valid (slave_cmd_valid),
        .is_write  (slave_is_write)
    );

    // AXI4-Lite Master - converts SPI commands to AXI transactions
    axi4_lite_master u_axi_master (
        .clk            (clk),
        .rst_n          (rst_n),
        .cmd_valid      (slave_cmd_valid),
        .cmd_is_write   (slave_is_write),
        .cmd_addr       (slave_addr),
        .cmd_wdata      (slave_wdata),
        .cmd_rdata      (axi_cmd_rdata),
        .cmd_done       (axi_cmd_done),
        .cmd_error      (axi_cmd_error),
        .m_axi_awaddr   (axi_awaddr),
        .m_axi_awvalid  (axi_awvalid),
        .m_axi_awready  (axi_awready),
        .m_axi_wdata    (axi_wdata),
        .m_axi_wstrb    (axi_wstrb),
        .m_axi_wvalid   (axi_wvalid),
        .m_axi_wready   (axi_wready),
        .m_axi_bresp    (axi_bresp),
        .m_axi_bvalid   (axi_bvalid),
        .m_axi_bready   (axi_bready),
        .m_axi_araddr   (axi_araddr),
        .m_axi_arvalid  (axi_arvalid),
        .m_axi_arready  (axi_arready),
        .m_axi_rdata    (axi_rdata),
        .m_axi_rresp    (axi_rresp),
        .m_axi_rvalid   (axi_rvalid),
        .m_axi_rready   (axi_rready)
    );

    // AXI4-Lite Slave - register file (8 x 32-bit)
    axi4_lite_slave u_axi_slave (
        .clk            (clk),
        .rst_n          (rst_n),
        .s_axi_awaddr   (axi_awaddr),
        .s_axi_awvalid  (axi_awvalid),
        .s_axi_awready  (axi_awready),
        .s_axi_wdata    (axi_wdata),
        .s_axi_wstrb    (axi_wstrb),
        .s_axi_wvalid   (axi_wvalid),
        .s_axi_wready   (axi_wready),
        .s_axi_bresp    (axi_bresp),
        .s_axi_bvalid   (axi_bvalid),
        .s_axi_bready   (axi_bready),
        .s_axi_araddr   (axi_araddr),
        .s_axi_arvalid  (axi_arvalid),
        .s_axi_arready  (axi_arready),
        .s_axi_rdata    (axi_rdata),
        .s_axi_rresp    (axi_rresp),
        .s_axi_rvalid   (axi_rvalid),
        .s_axi_rready   (axi_rready)
    );

    // 7-Segment Display Driver
    reg [7:0] display_data;
    seg7_driver u_seg7 (
        .clk          (clk),
        .rst_n        (rst_n),
        .display_data (display_data),
        .seg          (SEG),
        .an           (AN),
        .dp           (DP)
    );

    //========================================================================
    // DEMO SEQUENCER FSM
    //========================================================================
    // Automatically runs 4 writes + 4 reads on power-up.
    //========================================================================

    localparam NUM_TRANSACTIONS = 8;

    // Transaction ROM - combinational lookup (synthesizable for FPGA)
    // Index 0-3: Writes;  Index 4-7: Reads
    reg        tx_rw_r;
    reg  [7:0] tx_addr_r;
    reg  [7:0] tx_wdata_r;

    always @(*) begin
        case (tx_index)
            4'd0: begin tx_rw_r = 1'b1; tx_addr_r = 8'h00; tx_wdata_r = 8'hA5; end
            4'd1: begin tx_rw_r = 1'b1; tx_addr_r = 8'h01; tx_wdata_r = 8'h3C; end
            4'd2: begin tx_rw_r = 1'b1; tx_addr_r = 8'h02; tx_wdata_r = 8'h7E; end
            4'd3: begin tx_rw_r = 1'b1; tx_addr_r = 8'h03; tx_wdata_r = 8'hF0; end
            4'd4: begin tx_rw_r = 1'b0; tx_addr_r = 8'h00; tx_wdata_r = 8'h00; end
            4'd5: begin tx_rw_r = 1'b0; tx_addr_r = 8'h01; tx_wdata_r = 8'h00; end
            4'd6: begin tx_rw_r = 1'b0; tx_addr_r = 8'h02; tx_wdata_r = 8'h00; end
            4'd7: begin tx_rw_r = 1'b0; tx_addr_r = 8'h03; tx_wdata_r = 8'h00; end
            default: begin tx_rw_r = 1'b0; tx_addr_r = 8'h00; tx_wdata_r = 8'h00; end
        endcase
    end

    // Sequencer FSM States
    localparam [2:0] SEQ_INIT  = 3'd0,
                     SEQ_START = 3'd1,
                     SEQ_WAIT  = 3'd2,  // Combined wait for both SPI + AXI
                     SEQ_NEXT  = 3'd3,
                     SEQ_HALT  = 3'd4;

    reg [2:0]  seq_state;
    reg [3:0]  tx_index;         // Transaction index (0 to 7)
    reg [7:0]  last_write_data;  // Last data written (for display)
    reg [7:0]  last_read_data;   // Last data read (for display)
    reg        led_done;         // Latched done indicator
    reg        led_error;        // Latched error indicator
    reg        led_reading;      // Currently doing a read
    reg        led_writing;      // Currently doing a write
    reg [7:0]  led_data;         // Current data value
    reg [3:0]  led_addr;         // Current address

    // Done flags - latch the 1-cycle pulses so they can't be missed.
    // The AXI transaction completes much faster (100 MHz) than SPI (1 MHz),
    // so without latching, the cmd_done pulse would be missed.
    reg        spi_done_flag;
    reg        axi_done_flag;
    reg        axi_error_flag;
    reg  [7:0] axi_rdata_captured;  // Captured read data from AXI

    // Latch done pulses into sticky flags
    always @(posedge clk) begin
        if (!rst_n) begin
            spi_done_flag    <= 1'b0;
            axi_done_flag    <= 1'b0;
            axi_error_flag   <= 1'b0;
            axi_rdata_captured <= 8'd0;
        end else begin
            // Clear flags when starting a new transaction
            if (seq_state == SEQ_START) begin
                spi_done_flag    <= 1'b0;
                axi_done_flag    <= 1'b0;
                axi_error_flag   <= 1'b0;
            end
            // Latch SPI done
            if (spi_done)
                spi_done_flag <= 1'b1;
            // Latch AXI done + error + read data
            if (axi_cmd_done) begin
                axi_done_flag    <= 1'b1;
                axi_error_flag   <= axi_cmd_error;
                axi_rdata_captured <= axi_cmd_rdata;
            end
        end
    end

    // Delay counter to space out transactions
    reg [19:0] delay_cnt;
    localparam DELAY_CYCLES = 20'd5000;  // Small delay between transactions

    always @(posedge clk) begin
        if (!rst_n) begin
            seq_state      <= SEQ_INIT;
            tx_index       <= 4'd0;
            spi_start      <= 1'b0;
            spi_rw         <= 1'b0;
            spi_addr       <= 8'd0;
            spi_wdata      <= 8'd0;
            last_write_data <= 8'd0;
            last_read_data  <= 8'd0;
            led_done       <= 1'b0;
            led_error      <= 1'b0;
            led_reading    <= 1'b0;
            led_writing    <= 1'b0;
            led_data       <= 8'd0;
            led_addr       <= 4'd0;
            delay_cnt      <= 20'd0;
        end else begin
            spi_start <= 1'b0;  // Default: de-assert start

            case (seq_state)
                SEQ_INIT: begin
                    // Wait a bit after reset before starting
                    delay_cnt <= delay_cnt + 20'd1;
                    if (delay_cnt == DELAY_CYCLES) begin
                        delay_cnt <= 20'd0;
                        seq_state <= SEQ_START;
                    end
                end

                SEQ_START: begin
                    // Load current transaction and start SPI
                    spi_rw    <= tx_rw_r;
                    spi_addr  <= tx_addr_r;
                    spi_wdata <= tx_wdata_r;
                    spi_start <= 1'b1;

                    // Update LED indicators
                    led_writing <= tx_rw_r;
                    led_reading <= ~tx_rw_r;
                    led_addr    <= tx_addr_r[3:0];
                    if (tx_rw_r)
                        led_data <= tx_wdata_r;

                    seq_state <= SEQ_WAIT;
                end

                SEQ_WAIT: begin
                    // Wait for BOTH SPI and AXI to complete.
                    // Flags latch the pulses so order doesn't matter.
                    if (spi_done_flag && axi_done_flag) begin
                        led_done <= 1'b1;
                        if (axi_error_flag)
                            led_error <= 1'b1;

                        // Capture results
                        if (tx_rw_r) begin
                            last_write_data <= tx_wdata_r;
                        end else begin
                            last_read_data <= axi_rdata_captured;
                            led_data       <= axi_rdata_captured;
                        end

                        seq_state <= SEQ_NEXT;
                        delay_cnt <= 20'd0;
                    end
                end

                SEQ_NEXT: begin
                    // Inter-transaction delay
                    delay_cnt <= delay_cnt + 20'd1;
                    if (delay_cnt == DELAY_CYCLES) begin
                        delay_cnt <= 20'd0;
                        if (tx_index == NUM_TRANSACTIONS - 1) begin
                            // All transactions complete
                            seq_state <= SEQ_HALT;
                        end else begin
                            tx_index  <= tx_index + 4'd1;
                            led_done  <= 1'b0;
                            seq_state <= SEQ_START;
                        end
                    end
                end

                SEQ_HALT: begin
                    // Demo complete - hold final state
                    led_writing <= 1'b0;
                    led_reading <= 1'b0;
                end
            endcase
        end
    end

    //------------------------------------------------------------------------
    // Display Data Selection (SW[0] toggles write/read display)
    //------------------------------------------------------------------------
    always @(*) begin
        if (SW[0])
            display_data = last_read_data;
        else
            display_data = last_write_data;
    end

    //------------------------------------------------------------------------
    // LED Assignments
    //------------------------------------------------------------------------
    assign LED[0]    = led_reading;     // Read operation
    assign LED[1]    = led_writing;     // Write operation
    assign LED[2]    = led_done;        // Transaction done
    assign LED[3]    = led_error;       // Error
    assign LED[7:4]  = led_addr;        // Current address
    assign LED[15:8] = led_data;        // Current data

endmodule
