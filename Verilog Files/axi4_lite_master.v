//============================================================================
// Module: axi4_lite_master
// Description: Converts decoded SPI commands into AXI4-Lite transactions.
//
// AXI4-Lite Write Flow:
//   1. Assert AWVALID with address → wait for AWREADY
//   2. Assert WVALID with data + strobe → wait for WREADY
//   3. Assert BREADY → wait for BVALID, check BRESP
//
// AXI4-Lite Read Flow:
//   1. Assert ARVALID with address → wait for ARREADY
//   2. Assert RREADY → wait for RVALID, capture RDATA, check RRESP
//
// Address Mapping:
//   8-bit SPI address → 32-bit AXI address (left-shifted by 2 for word alignment)
//   Example: SPI addr 0x02 → AXI addr 0x00000008
//
// Data Mapping:
//   8-bit SPI data → lower 8 bits of 32-bit AXI data, WSTRB = 4'b0001
//============================================================================

module axi4_lite_master (
    input  wire        clk,
    input  wire        rst_n,
    //-- Command Interface (from SPI Slave) --
    input  wire        cmd_valid,     // Pulse: new command available
    input  wire        cmd_is_write,  // 1 = write, 0 = read
    input  wire  [7:0] cmd_addr,      // Register address (8-bit)
    input  wire  [7:0] cmd_wdata,     // Write data (8-bit)
    output reg   [7:0] cmd_rdata,     // Read data result (8-bit)
    output reg         cmd_done,      // Pulse: transaction complete
    output reg         cmd_error,     // 1 = AXI error (non-OKAY response)
    //-- AXI4-Lite Master Interface --
    // Write Address Channel
    output reg  [31:0] m_axi_awaddr,
    output reg         m_axi_awvalid,
    input  wire        m_axi_awready,
    // Write Data Channel
    output reg  [31:0] m_axi_wdata,
    output reg   [3:0] m_axi_wstrb,
    output reg         m_axi_wvalid,
    input  wire        m_axi_wready,
    // Write Response Channel
    input  wire  [1:0] m_axi_bresp,
    input  wire        m_axi_bvalid,
    output reg         m_axi_bready,
    // Read Address Channel
    output reg  [31:0] m_axi_araddr,
    output reg         m_axi_arvalid,
    input  wire        m_axi_arready,
    // Read Data Channel
    input  wire [31:0] m_axi_rdata,
    input  wire  [1:0] m_axi_rresp,
    input  wire        m_axi_rvalid,
    output reg         m_axi_rready
);

    //------------------------------------------------------------------------
    // FSM State Encoding
    //------------------------------------------------------------------------
    localparam [2:0] S_IDLE    = 3'd0,
                     S_WR_ADDR = 3'd1,
                     S_WR_DATA = 3'd2,
                     S_WR_RESP = 3'd3,
                     S_RD_ADDR = 3'd4,
                     S_RD_DATA = 3'd5,
                     S_DONE    = 3'd6;

    reg [2:0] state, next_state;

    //------------------------------------------------------------------------
    // Internal Registers
    //------------------------------------------------------------------------
    reg  [7:0] latched_addr;
    reg  [7:0] latched_wdata;
    reg        latched_is_write;
    reg        error_flag;

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
                if (cmd_valid) begin
                    if (cmd_is_write)
                        next_state = S_WR_ADDR;
                    else
                        next_state = S_RD_ADDR;
                end
            end

            S_WR_ADDR: begin
                if (m_axi_awready)
                    next_state = S_WR_DATA;
            end

            S_WR_DATA: begin
                if (m_axi_wready)
                    next_state = S_WR_RESP;
            end

            S_WR_RESP: begin
                if (m_axi_bvalid)
                    next_state = S_DONE;
            end

            S_RD_ADDR: begin
                if (m_axi_arready)
                    next_state = S_RD_DATA;
            end

            S_RD_DATA: begin
                if (m_axi_rvalid)
                    next_state = S_DONE;
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
            m_axi_awaddr  <= 32'd0;
            m_axi_awvalid <= 1'b0;
            m_axi_wdata   <= 32'd0;
            m_axi_wstrb   <= 4'b0000;
            m_axi_wvalid  <= 1'b0;
            m_axi_bready  <= 1'b0;
            m_axi_araddr  <= 32'd0;
            m_axi_arvalid <= 1'b0;
            m_axi_rready  <= 1'b0;
            cmd_rdata     <= 8'd0;
            cmd_done      <= 1'b0;
            cmd_error     <= 1'b0;
            latched_addr  <= 8'd0;
            latched_wdata <= 8'd0;
            latched_is_write <= 1'b0;
            error_flag    <= 1'b0;
        end else begin
            cmd_done  <= 1'b0;  // Default: de-assert
            cmd_error <= 1'b0;

            case (state)
                S_IDLE: begin
                    // De-assert all AXI signals
                    m_axi_awvalid <= 1'b0;
                    m_axi_wvalid  <= 1'b0;
                    m_axi_bready  <= 1'b0;
                    m_axi_arvalid <= 1'b0;
                    m_axi_rready  <= 1'b0;
                    error_flag    <= 1'b0;
                    if (cmd_valid) begin
                        // Latch command details
                        latched_addr     <= cmd_addr;
                        latched_wdata    <= cmd_wdata;
                        latched_is_write <= cmd_is_write;
                    end
                end

                S_WR_ADDR: begin
                    // Drive write address channel
                    m_axi_awaddr  <= {24'd0, latched_addr, 2'b00};  // Word-aligned
                    m_axi_awvalid <= 1'b1;
                end

                S_WR_DATA: begin
                    m_axi_awvalid <= 1'b0;
                    // Drive write data channel
                    m_axi_wdata  <= {24'd0, latched_wdata};  // Lower 8 bits
                    m_axi_wstrb  <= 4'b0001;                 // Only byte 0
                    m_axi_wvalid <= 1'b1;
                end

                S_WR_RESP: begin
                    m_axi_wvalid <= 1'b0;
                    m_axi_bready <= 1'b1;
                    if (m_axi_bvalid) begin
                        if (m_axi_bresp != 2'b00)
                            error_flag <= 1'b1;
                    end
                end

                S_RD_ADDR: begin
                    // Drive read address channel
                    m_axi_araddr  <= {24'd0, latched_addr, 2'b00};  // Word-aligned
                    m_axi_arvalid <= 1'b1;
                end

                S_RD_DATA: begin
                    m_axi_arvalid <= 1'b0;
                    m_axi_rready  <= 1'b1;
                    if (m_axi_rvalid) begin
                        cmd_rdata <= m_axi_rdata[7:0];  // Capture lower 8 bits
                        if (m_axi_rresp != 2'b00)
                            error_flag <= 1'b1;
                    end
                end

                S_DONE: begin
                    cmd_done  <= 1'b1;
                    cmd_error <= error_flag;
                end
            endcase
        end
    end

endmodule
