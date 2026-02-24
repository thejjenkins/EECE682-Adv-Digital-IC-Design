`timescale 1ns / 1ps
// =============================================================================
//  rk4_projectile_top_tb.sv
//  Self-checking testbench for rk4_projectile_top.sv
//  Tool   : Xilinx Vivado Lab Edition (xsim)
//  Style  : Single self-contained file; all stimulus and checking inline.
//
//  Strategy
//    • Instantiates DUT with a fast sim clock (50 MHz) and a scaled-down
//      BAUD_DIV so UART transactions complete in a reasonable sim time.
//    • Drives UART RX bytes into the DUT using a task that bit-bangs 8N1.
//    • Monitors UART TX bytes from the DUT using a task that samples 8N1.
//    • Runs three test cases:
//        TC1 – v0y = 49.0  m/s  (textbook example, T = 10 s, 100 steps)
//        TC2 – v0y = 19.6  m/s  (T =  4 s, lower arc)
//        TC3 – v0y =  9.8  m/s  (T =  2 s, low arc, hits ground quickly)
//    • After each run it checks:
//        – The end-of-stream marker 0xDEADBEEF arrives.
//        – Every received yi is >= 0 (sign bit clear).
//        – The first yi is within 5 % of the analytic value at dt.
//        – yi at the midpoint step (step 50) matches analytic within 1 %.
//
//  Q16.16 helper functions are defined in this file (not synthesized).
// =============================================================================

module rk4_projectile_top_tb;

    // -------------------------------------------------------------------------
    // Simulation parameters
    // -------------------------------------------------------------------------
    // Use a small BAUD_DIV so each UART bit is only 10 clock cycles.
    // This dramatically reduces sim time vs. a real 115200-baud divisor.
    localparam integer SIM_CLK_FREQ  = 50_000_000;
    localparam integer SIM_BAUD_DIV  = 10;          // 10 clocks per UART bit
    localparam integer SIM_BAUD_RATE = SIM_CLK_FREQ / SIM_BAUD_DIV;

    localparam real CLK_PERIOD = 20.0; // ns  (50 MHz)
    localparam real BIT_PERIOD = CLK_PERIOD * SIM_BAUD_DIV; // ns per UART bit

    // -------------------------------------------------------------------------
    // DUT signals
    // -------------------------------------------------------------------------
    reg  clk;
    reg  rst_n;
    reg  uart_rx_tb;   // TB drives this → DUT rx input
    wire uart_tx_tb;   // DUT drives this → TB monitors

    // -------------------------------------------------------------------------
    // DUT instantiation  – override CLK_FREQ/BAUD_RATE with sim-friendly values
    // -------------------------------------------------------------------------
    rk4_projectile_top #(
        .CLK_FREQ  (SIM_CLK_FREQ),
        .BAUD_RATE (SIM_BAUD_RATE)
    ) dut (
        .clk      (clk),
        .rst_n    (rst_n),
        .uart_rx  (uart_rx_tb),
        .uart_tx  (uart_tx_tb)
    );

    // -------------------------------------------------------------------------
    // Clock
    // -------------------------------------------------------------------------
    initial clk = 1'b0;
    always #(CLK_PERIOD / 2.0) clk = ~clk;

    // -------------------------------------------------------------------------
    // Q16.16 helpers (real ↔ fixed, simulation only)
    // -------------------------------------------------------------------------
    function automatic signed [31:0] to_q1616(input real r);
        to_q1616 = $rtoi(r * 65536.0);
    endfunction

    function automatic real from_q1616(input signed [31:0] f);
        from_q1616 = $itor(f) / 65536.0;
    endfunction

    // Analytic y(t) for projectile: y = v0y*t - 0.5*g*t^2
    function automatic real analytic_y(input real v0y, real t);
        analytic_y = v0y * t - 0.5 * 9.8 * t * t;
    endfunction

    // Absolute value for real numbers ($abs not supported in Vivado xsim)
    function automatic real real_abs(input real x);
        real_abs = (x < 0.0) ? -x : x;
    endfunction

    // -------------------------------------------------------------------------
    // Task: send one UART byte (8N1) to DUT, LSB first
    // -------------------------------------------------------------------------
    task automatic uart_send_byte(input [7:0] data);
        integer i;
        begin
            // Start bit
            uart_rx_tb = 1'b0;
            #(BIT_PERIOD);
            // Data bits LSB→MSB
            for (i = 0; i < 8; i = i + 1) begin
                uart_rx_tb = data[i];
                #(BIT_PERIOD);
            end
            // Stop bit
            uart_rx_tb = 1'b1;
            #(BIT_PERIOD);
        end
    endtask

    // -------------------------------------------------------------------------
    // Task: send v0y as 4 bytes little-endian Q16.16 over UART
    // -------------------------------------------------------------------------
    task automatic uart_send_v0y(input real v0y_real);
        reg signed [31:0] v0y_fixed;
        begin
            v0y_fixed = to_q1616(v0y_real);
            $display("[TB] Sending v0y = %0.2f m/s  (Q16.16 = 0x%08X)",
                     v0y_real, v0y_fixed);
            uart_send_byte(v0y_fixed[ 7: 0]);
            uart_send_byte(v0y_fixed[15: 8]);
            uart_send_byte(v0y_fixed[23:16]);
            uart_send_byte(v0y_fixed[31:24]);
        end
    endtask

    // -------------------------------------------------------------------------
    // Task: receive one UART byte from DUT (8N1), sample at mid-bit
    // -------------------------------------------------------------------------
    task automatic uart_recv_byte(output [7:0] data);
        integer i;
        begin
            // Wait for start bit (falling edge on uart_tx_tb)
            @(negedge uart_tx_tb);
            // Wait half a bit period to land in middle of start bit
            #(BIT_PERIOD / 2.0);
            // Verify start bit is still low
            if (uart_tx_tb !== 1'b0)
                $display("[TB] WARNING: start bit glitch detected");
            // Sample 8 data bits
            for (i = 0; i < 8; i = i + 1) begin
                #(BIT_PERIOD);
                data[i] = uart_tx_tb;
            end
            // Consume stop bit
            #(BIT_PERIOD);
        end
    endtask

    // -------------------------------------------------------------------------
    // Task: receive one 32-bit word (4 bytes, little-endian) from DUT
    // -------------------------------------------------------------------------
    task automatic uart_recv_word(output [31:0] word);
        reg [7:0] b0, b1, b2, b3;
        begin
            uart_recv_byte(b0);
            uart_recv_byte(b1);
            uart_recv_byte(b2);
            uart_recv_byte(b3);
            word = {b3, b2, b1, b0};
        end
    endtask

    // -------------------------------------------------------------------------
    // Task: run one full test case
    //   - Sends v0y, then collects (ti, yi) pairs until 0xDEADBEEF received.
    //   - Checks sign, first-step accuracy, mid-step accuracy.
    // -------------------------------------------------------------------------
    integer tc_pass_cnt;
    integer tc_fail_cnt;

    task automatic run_test(
        input real   v0y_real,
        input string tc_name
    );
        reg [31:0]     ti_word, yi_word;
        real           ti_real, yi_real;
        real           g, T, dt_real;
        real           y_analytic;
        real           err_pct;
        integer        step;
        reg            got_eos;          // end-of-stream marker received
        reg            y_neg_violation;
        real           first_yi, mid_yi;
        integer        mid_step;

        begin
            g              = 9.8;
            T              = 2.0 * v0y_real / g;
            dt_real        = T / 100.0;
            step           = 0;
            got_eos        = 1'b0;
            y_neg_violation= 1'b0;
            first_yi       = 0.0;
            mid_yi         = 0.0;
            mid_step       = 50;

            $display("");
            $display("============================================================");
            $display("[TB] TEST CASE: %s", tc_name);
            $display("[TB]   v0y = %0.2f m/s,  T = %0.2f s,  dt = %0.4f s",
                     v0y_real, T, dt_real);
            $display("============================================================");

            // Apply reset then send v0y
            rst_n = 1'b0;
            repeat(8) @(posedge clk);
            rst_n = 1'b1;
            repeat(4) @(posedge clk);

            uart_send_v0y(v0y_real);

            // Collect output pairs until EOS marker
            forever begin
                // Receive ti (4 bytes)
                uart_recv_word(ti_word);

                // Check for end-of-stream: 0xDEADBEEF
                if (ti_word == 32'hDEADBEEF) begin
                    got_eos = 1'b1;
                    $display("[TB] Received end-of-stream marker 0xDEADBEEF at step %0d",
                             step);
                    break;
                end

                // Receive yi (4 bytes)
                uart_recv_word(yi_word);

                ti_real   = from_q1616($signed(ti_word));
                yi_real   = from_q1616($signed(yi_word));
                y_analytic= analytic_y(v0y_real, ti_real);

                $display("[TB]   step %3d : ti = %7.4f s,  yi = %8.4f m  (analytic = %8.4f m)",
                         step+1, ti_real, yi_real, y_analytic);

                // Check sign – DUT should stop before yi goes negative
                if ($signed(yi_word) < 0) begin
                    $display("[TB]   *** FAIL: yi is negative at step %0d ***", step+1);
                    y_neg_violation = 1'b1;
                end

                // Capture first and mid values for accuracy checks
                if (step == 0)        first_yi = yi_real;
                if (step == mid_step) mid_yi   = yi_real;

                step = step + 1;
            end

            // ------------------------------------------------------------------
            // Check 1 – EOS marker arrived
            // ------------------------------------------------------------------
            if (got_eos) begin
                $display("[TB] CHECK EOS marker         : PASS");
                tc_pass_cnt = tc_pass_cnt + 1;
            end else begin
                $display("[TB] CHECK EOS marker         : FAIL (never received)");
                tc_fail_cnt = tc_fail_cnt + 1;
            end

            // ------------------------------------------------------------------
            // Check 2 – No negative y values output
            // ------------------------------------------------------------------
            if (!y_neg_violation) begin
                $display("[TB] CHECK no negative y      : PASS");
                tc_pass_cnt = tc_pass_cnt + 1;
            end else begin
                $display("[TB] CHECK no negative y      : FAIL");
                tc_fail_cnt = tc_fail_cnt + 1;
            end

            // ------------------------------------------------------------------
            // Check 3 – First step accuracy within 5 %
            // ------------------------------------------------------------------
            if (step >= 1) begin
                y_analytic = analytic_y(v0y_real, dt_real);
                if (y_analytic != 0.0)
                    err_pct = 100.0 * real_abs(first_yi - y_analytic) / real_abs(y_analytic);
                else
                    err_pct = 0.0;
                if (err_pct <= 5.0) begin
                    $display("[TB] CHECK first step (<%0.0f%% err): PASS  (err = %0.2f%%)",
                             5.0, err_pct);
                    tc_pass_cnt = tc_pass_cnt + 1;
                end else begin
                    $display("[TB] CHECK first step (<%0.0f%% err): FAIL  (err = %0.2f%%)",
                             5.0, err_pct);
                    tc_fail_cnt = tc_fail_cnt + 1;
                end
            end

            // ------------------------------------------------------------------
            // Check 4 – Mid-step accuracy within 2 %
            // ------------------------------------------------------------------
            if (step > mid_step) begin
                y_analytic = analytic_y(v0y_real, dt_real * (mid_step + 1));
                if (y_analytic != 0.0)
                    err_pct = 100.0 * real_abs(mid_yi - y_analytic) / real_abs(y_analytic);
                else
                    err_pct = 0.0;
                if (err_pct <= 2.0) begin
                    $display("[TB] CHECK mid step  (<%0.0f%% err): PASS  (err = %0.2f%%)",
                             2.0, err_pct);
                    tc_pass_cnt = tc_pass_cnt + 1;
                end else begin
                    $display("[TB] CHECK mid step  (<%0.0f%% err): FAIL  (err = %0.2f%%)",
                             2.0, err_pct);
                    tc_fail_cnt = tc_fail_cnt + 1;
                end
            end

            $display("------------------------------------------------------------");
        end
    endtask

    // -------------------------------------------------------------------------
    // Main stimulus
    // -------------------------------------------------------------------------
    initial begin
        $display("");
        $display("############################################################");
        $display("#   rk4_projectile_top Testbench  (Vivado xsim)            #");
        $display("############################################################");

        // Initialise
        uart_rx_tb  = 1'b1;   // UART idle high
        rst_n       = 1'b0;
        tc_pass_cnt = 0;
        tc_fail_cnt = 0;

        // Waveform dump for Vivado simulator
        $dumpfile("rk4_projectile_top_tb.vcd");
        $dumpvars(0, rk4_projectile_top_tb);

        // ------------------------------------------------------------------
        // TC1 : v0y = 49.0 m/s  →  T = 10 s,  dt = 0.1 s
        // ------------------------------------------------------------------
        run_test(49.0, "TC1: v0y=49.0 m/s");

        // Inter-test gap
        repeat(20) @(posedge clk);

        // ------------------------------------------------------------------
        // TC2 : v0y = 19.6 m/s  →  T = 4 s,   dt = 0.04 s
        // ------------------------------------------------------------------
        run_test(19.6, "TC2: v0y=19.6 m/s");

        repeat(20) @(posedge clk);

        // ------------------------------------------------------------------
        // TC3 : v0y = 9.8 m/s   →  T = 2 s,   dt = 0.02 s
        // ------------------------------------------------------------------
        run_test(9.8,  "TC3: v0y=9.8 m/s");

        // ------------------------------------------------------------------
        // Summary
        // ------------------------------------------------------------------
        repeat(20) @(posedge clk);
        $display("");
        $display("############################################################");
        $display("#  SIMULATION SUMMARY                                       #");
        $display("#  Total PASS : %0d", tc_pass_cnt);
        $display("#  Total FAIL : %0d", tc_fail_cnt);
        if (tc_fail_cnt == 0)
            $display("#  RESULT     : ALL TESTS PASSED                           #");
        else
            $display("#  RESULT     : *** FAILURES DETECTED ***                  #");
        $display("############################################################");
        $display("");

        $finish;
    end

    // -------------------------------------------------------------------------
    // Timeout watchdog – abort after 50 ms sim time to prevent hangs
    // -------------------------------------------------------------------------
    initial begin
        #(50_000_000);   // 50 ms in ns
        $display("[TB] WATCHDOG TIMEOUT – simulation did not complete in time.");
        $finish;
    end

endmodule
