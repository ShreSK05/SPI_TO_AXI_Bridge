//============================================================================
// Module: axi4_lite_slave
// Description: Simple AXI4-Lite Slave with 8 x 32-bit registers.
//
// Address Map:
//   0x00 → reg_file[0]
//   0x04 → reg_file[1]
//   0x08 → reg_file[2]
//   0x0C → reg_file[3]
//   0x10 → reg_file[4]
//   0x14 → reg_file[5]
//   0x18 → reg_file[6]
//   0x1C → reg_file[7]
//
// Out-of-range addresses return SLVERR (BRESP/RRESP = 2'b10).
//
// AXI4-Lite Protocol:
//   - Write: AWVALID/AWREADY → WVALID/WREADY → BVALID/BREADY
//   - Read:  ARVALID/ARREADY → RVALID/RREADY
//   - All channels use valid/ready handshaking
//============================================================================

module axi4_lite_slave (
    input  wire        clk,
    input  wire        rst_n,
    //-- Write Address Channel --
    input  wire [31:0] s_axi_awaddr,
    input  wire        s_axi_awvalid,
    output reg         s_axi_awready,
    //-- Write Data Channel --
    input  wire [31:0] s_axi_wdata,
    input  wire  [3:0] s_axi_wstrb,
    input  wire        s_axi_wvalid,
    output reg         s_axi_wready,
    //-- Write Response Channel --
    output reg   [1:0] s_axi_bresp,
    output reg         s_axi_bvalid,
    input  wire        s_axi_bready,
    //-- Read Address Channel --
    input  wire [31:0] s_axi_araddr,
    input  wire        s_axi_arvalid,
    output reg         s_axi_arready,
    //-- Read Data Channel --
    output reg  [31:0] s_axi_rdata,
    output reg   [1:0] s_axi_rresp,
    output reg         s_axi_rvalid,
    input  wire        s_axi_rready
);

    //------------------------------------------------------------------------
    // Constants
    //------------------------------------------------------------------------
    localparam NUM_REGS   = 8;
    localparam ADDR_WIDTH = 5;  // Need bits [4:0] for 8 words × 4 bytes = 32 bytes
    localparam RESP_OKAY  = 2'b00;
    localparam RESP_SLVERR = 2'b10;

    //------------------------------------------------------------------------
    // FSM States
    //------------------------------------------------------------------------
    localparam [2:0] S_IDLE    = 3'd0,
                     S_WR_DATA = 3'd1,
                     S_WR_RESP = 3'd2,
                     S_RD_DATA = 3'd3;

    reg [2:0] state, next_state;

    //------------------------------------------------------------------------
    // Register File
    //------------------------------------------------------------------------
    reg [31:0] reg_file [0:NUM_REGS-1];

    //------------------------------------------------------------------------
    // Latched Address
    //------------------------------------------------------------------------
    reg [31:0] wr_addr_latched;
    reg [31:0] rd_addr_latched;

    //------------------------------------------------------------------------
    // Address Decode - extract register index from word-aligned address
    //------------------------------------------------------------------------
    wire [2:0] wr_reg_idx = wr_addr_latched[4:2];
    wire [2:0] rd_reg_idx = rd_addr_latched[4:2];
    wire       wr_addr_valid = (wr_addr_latched[31:5] == 27'd0);  // Check in range
    wire       rd_addr_valid = (rd_addr_latched[31:5] == 27'd0);

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
                if (s_axi_awvalid)
                    next_state = S_WR_DATA;
                else if (s_axi_arvalid)
                    next_state = S_RD_DATA;
            end

            S_WR_DATA: begin
                if (s_axi_wvalid)
                    next_state = S_WR_RESP;
            end

            S_WR_RESP: begin
                if (s_axi_bready)
                    next_state = S_IDLE;
            end

            S_RD_DATA: begin
                if (s_axi_rready)
                    next_state = S_IDLE;
            end

            default: next_state = S_IDLE;
        endcase
    end

    //------------------------------------------------------------------------
    // Datapath - Sequential Logic
    //------------------------------------------------------------------------
    integer i = 0;

    always @(posedge clk) begin
        if (!rst_n) begin
            s_axi_awready  <= 1'b0;
            s_axi_wready   <= 1'b0;
            s_axi_bresp    <= 2'b00;
            s_axi_bvalid   <= 1'b0;
            s_axi_arready  <= 1'b0;
            s_axi_rdata    <= 32'd0;
            s_axi_rresp    <= 2'b00;
            s_axi_rvalid   <= 1'b0;
            wr_addr_latched <= 32'd0;
            rd_addr_latched <= 32'd0;
            for (i = 0; i < NUM_REGS; i = i + 1)
                reg_file[i] <= 32'd0;
        end else begin
            // Defaults - de-assert handshake signals
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            s_axi_arready <= 1'b0;

            case (state)
                S_IDLE: begin
                    s_axi_bvalid <= 1'b0;
                    s_axi_rvalid <= 1'b0;
                    if (s_axi_awvalid) begin
                        // Accept write address
                        s_axi_awready   <= 1'b1;
                        wr_addr_latched <= s_axi_awaddr;
                    end else if (s_axi_arvalid) begin
                        // Accept read address
                        s_axi_arready   <= 1'b1;
                        rd_addr_latched <= s_axi_araddr;
                    end
                end

                S_WR_DATA: begin
                    s_axi_bvalid <= 1'b0;
                    if (s_axi_wvalid) begin
                        s_axi_wready <= 1'b1;
                        // Write data to register (with byte strobes)
                        if (wr_addr_valid) begin
                            if (s_axi_wstrb[0]) reg_file[wr_reg_idx][ 7: 0] <= s_axi_wdata[ 7: 0];
                            if (s_axi_wstrb[1]) reg_file[wr_reg_idx][15: 8] <= s_axi_wdata[15: 8];
                            if (s_axi_wstrb[2]) reg_file[wr_reg_idx][23:16] <= s_axi_wdata[23:16];
                            if (s_axi_wstrb[3]) reg_file[wr_reg_idx][31:24] <= s_axi_wdata[31:24];
                        end
                    end
                end

                S_WR_RESP: begin
                    s_axi_bvalid <= 1'b1;
                    s_axi_bresp  <= wr_addr_valid ? RESP_OKAY : RESP_SLVERR;
                end

                S_RD_DATA: begin
                    s_axi_rvalid <= 1'b1;
                    if (rd_addr_valid) begin
                        s_axi_rdata <= reg_file[rd_reg_idx];
                        s_axi_rresp <= RESP_OKAY;
                    end else begin
                        s_axi_rdata <= 32'hDEAD_BEEF;
                        s_axi_rresp <= RESP_SLVERR;
                    end
                end
            endcase
        end
    end

endmodule
