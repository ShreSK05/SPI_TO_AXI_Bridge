//============================================================================
// Testbench: tb_top_spi_axi
// Description: Full-system testbench for SPI to AXI4-Lite Bridge
//
// Test Strategy:
//   1. Apply reset
//   2. Let the demo sequencer run automatically (4 writes + 4 reads)
//   3. Monitor AXI slave register file to verify writes
//   4. Monitor SPI master read data to verify reads
//   5. Check LED outputs for expected status
//   6. Report PASS/FAIL
//
// Runtime: ~5 ms simulated time (at 1 MHz SCLK, each 24-bit SPI
//          transaction takes ~24 μs, 8 transactions + gaps ≈ 300 μs,
//          plus delays)
//============================================================================

`timescale 1ns / 1ps

module tb_top_spi_axi;

    //------------------------------------------------------------------------
    // DUT Signals
    //------------------------------------------------------------------------
    reg         CLK100MHZ;
    reg         CPU_RESETN;
    reg  [15:0] SW;
    wire [15:0] LED;
    wire  [6:0] SEG;
    wire  [7:0] AN;
    wire        DP;

    //------------------------------------------------------------------------
    // DUT Instantiation
    //------------------------------------------------------------------------
    top_spi_axi u_dut (
        .CLK100MHZ  (CLK100MHZ),
        .CPU_RESETN (CPU_RESETN),
        .SW         (SW),
        .LED        (LED),
        .SEG        (SEG),
        .AN         (AN),
        .DP         (DP)
    );

    //------------------------------------------------------------------------
    // Clock Generation - 100 MHz (period = 10 ns)
    //------------------------------------------------------------------------
    initial CLK100MHZ = 0;
    always #5 CLK100MHZ = ~CLK100MHZ;

    //------------------------------------------------------------------------
    // Test Variables
    //------------------------------------------------------------------------
    integer test_pass = 0;
    integer test_fail = 0;
    integer i = 0;
    reg [7:0] expected_data [0:3];

    initial begin
        expected_data[0] = 8'hA5;
        expected_data[1] = 8'h3C;
        expected_data[2] = 8'h7E;
        expected_data[3] = 8'hF0;
    end

    //------------------------------------------------------------------------
    // Waveform Dump
    //------------------------------------------------------------------------
    initial begin
        $dumpfile("spi_axi_bridge.vcd");
        $dumpvars(0, tb_top_spi_axi);
    end

    //------------------------------------------------------------------------
    // Main Test Sequence
    //------------------------------------------------------------------------
    initial begin
        test_pass = 0;
        test_fail = 0;

        //--------------------------------------------------------------------
        // Display header
        //--------------------------------------------------------------------
        $display("==========================================================");
        $display("  SPI to AXI4-Lite Bridge - Full System Testbench");
        $display("==========================================================");
        $display("  Time: %0t", $time);
        $display("");

        //--------------------------------------------------------------------
        // 1. Apply Reset
        //--------------------------------------------------------------------
        $display("[%0t] Applying reset...", $time);
        CPU_RESETN = 0;
        SW         = 16'h0000;
        #200;  // Hold reset for 200 ns
        CPU_RESETN = 1;
        $display("[%0t] Reset released.", $time);
        $display("");

        //--------------------------------------------------------------------
        // 2. Wait for demo sequencer to complete all 8 transactions
        //    The sequencer writes to 4 registers, then reads them back.
        //    At 1 MHz SCLK with 24-bit frames + inter-transaction delays,
        //    this should complete within ~5 ms of simulated time.
        //--------------------------------------------------------------------
        $display("[%0t] Waiting for demo sequencer to complete...", $time);
        $display("       (This may take a few seconds of simulation time)");
        $display("");

        // Wait for the sequencer to reach HALT state
        // The sequencer FSM is at seq_state = SEQ_HALT (3'd4) when done
        wait (u_dut.seq_state == 3'd4);
        #1000;  // A little margin
        $display("[%0t] Demo sequencer reached HALT state.", $time);
        $display("");

        //--------------------------------------------------------------------
        // 3. Verify AXI Slave Register Contents
        //--------------------------------------------------------------------
        $display("--- Verifying AXI Slave Register Contents ---");

        for (i = 0; i < 4; i = i + 1) begin
            if (u_dut.u_axi_slave.reg_file[i][7:0] == expected_data[i]) begin
                $display("  [PASS] reg_file[%0d] = 0x%02H (expected 0x%02H)",
                         i, u_dut.u_axi_slave.reg_file[i][7:0], expected_data[i]);
                test_pass = test_pass + 1;
            end else begin
                $display("  [FAIL] reg_file[%0d] = 0x%02H (expected 0x%02H)",
                         i, u_dut.u_axi_slave.reg_file[i][7:0], expected_data[i]);
                test_fail = test_fail + 1;
            end
        end
        $display("");

        //--------------------------------------------------------------------
        // 4. Verify LED Status
        //--------------------------------------------------------------------
        $display("--- Verifying LED Status ---");

        // LED[2] = done should be high
        if (LED[2] == 1'b1) begin
            $display("  [PASS] LED[2] (Done) = 1");
            test_pass = test_pass + 1;
        end else begin
            $display("  [FAIL] LED[2] (Done) = %b (expected 1)", LED[2]);
            test_fail = test_fail + 1;
        end

        // LED[3] = error should be low
        if (LED[3] == 1'b0) begin
            $display("  [PASS] LED[3] (Error) = 0");
            test_pass = test_pass + 1;
        end else begin
            $display("  [FAIL] LED[3] (Error) = %b (expected 0)", LED[3]);
            test_fail = test_fail + 1;
        end
        $display("");

        //--------------------------------------------------------------------
        // 5. Verify Read-Back Data
        //    The last read was from register 3 → expect 0xF0
        //--------------------------------------------------------------------
        $display("--- Verifying Read-Back Data ---");

        if (u_dut.last_read_data == 8'hF0) begin
            $display("  [PASS] last_read_data = 0x%02H (expected 0xF0)",
                     u_dut.last_read_data);
            test_pass = test_pass + 1;
        end else begin
            $display("  [FAIL] last_read_data = 0x%02H (expected 0xF0)",
                     u_dut.last_read_data);
            test_fail = test_fail + 1;
        end

        if (u_dut.last_write_data == 8'hF0) begin
            $display("  [PASS] last_write_data = 0x%02H (expected 0xF0)",
                     u_dut.last_write_data);
            test_pass = test_pass + 1;
        end else begin
            $display("  [FAIL] last_write_data = 0x%02H (expected 0xF0)",
                     u_dut.last_write_data);
            test_fail = test_fail + 1;
        end
        $display("");

        //--------------------------------------------------------------------
        // 6. Test Display Mode Switch
        //--------------------------------------------------------------------
        $display("--- Testing Display Mode Switch ---");

        // Switch to read data display
        SW[0] = 1'b1;
        #100;
        $display("  SW[0] = 1 → Display mode: READ DATA");
        $display("  display_data = 0x%02H", u_dut.display_data);

        // Switch to write data display
        SW[0] = 1'b0;
        #100;
        $display("  SW[0] = 0 → Display mode: WRITE DATA");
        $display("  display_data = 0x%02H", u_dut.display_data);
        $display("");

        //--------------------------------------------------------------------
        // 7. Summary
        //--------------------------------------------------------------------
        $display("==========================================================");
        $display("  TEST SUMMARY");
        $display("==========================================================");
        $display("  Total Passed: %0d", test_pass);
        $display("  Total Failed: %0d", test_fail);
        $display("");

        if (test_fail == 0) begin
            $display("  ╔═══════════════════════════════════╗");
            $display("  ║       *** ALL TESTS PASSED ***    ║");
            $display("  ╚═══════════════════════════════════╝");
        end else begin
            $display("  ╔═══════════════════════════════════╗");
            $display("  ║       *** SOME TESTS FAILED ***   ║");
            $display("  ╚═══════════════════════════════════╝");
        end

        $display("");
        $display("[%0t] Simulation complete.", $time);
        $display("==========================================================");
        $finish;
    end

    //------------------------------------------------------------------------
    // Transaction Monitor - observe SPI Master done pulses
    //------------------------------------------------------------------------
    always @(posedge CLK100MHZ) begin
        if (u_dut.spi_done) begin
            $display("[%0t] SPI Transaction Complete - RW=%b ADDR=0x%02H WDATA=0x%02H RDATA=0x%02H",
                     $time,
                     u_dut.spi_rw,
                     u_dut.spi_addr,
                     u_dut.spi_wdata,
                     u_dut.spi_rdata);
        end
    end

    always @(posedge CLK100MHZ) begin
        if (u_dut.axi_cmd_done) begin
            $display("[%0t] AXI Transaction Complete - Write=%b Error=%b RDATA=0x%02H",
                     $time,
                     u_dut.u_axi_master.latched_is_write,
                     u_dut.axi_cmd_error,
                     u_dut.axi_cmd_rdata);
        end
    end

    //------------------------------------------------------------------------
    // Timeout watchdog - prevent infinite simulation
    //------------------------------------------------------------------------
    initial begin
        #50_000_000;  // 50 ms timeout
        $display("");
        $display("[%0t] ERROR: Simulation timeout! Possible deadlock.", $time);
        $display("");
        $finish;
    end

endmodule
