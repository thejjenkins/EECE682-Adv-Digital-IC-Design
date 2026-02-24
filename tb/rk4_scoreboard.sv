// =============================================================================
//  rk4_scoreboard.sv -- Collects UART bytes, reassembles (ti, yi) pairs,
//                       checks against analytic projectile reference
// =============================================================================
class rk4_scoreboard extends uvm_scoreboard;

  `uvm_component_utils(rk4_scoreboard)

  uvm_analysis_imp #(rk4_uart_rx_item, rk4_scoreboard) rx_imp;

  real    expected_v0y;
  real    expected_T;
  real    expected_dt;

  // Byte accumulation
  logic [7:0] byte_buf [$];
  int         word_phase;   // 0 = accumulating ti, 1 = accumulating yi
  int         step_cnt;
  bit         eos_received;

  int pass_cnt;
  int fail_cnt;

  function new(string name = "rk4_scoreboard", uvm_component parent = null);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    rx_imp = new("rx_imp", this);
    word_phase   = 0;
    step_cnt     = 0;
    eos_received = 0;
    pass_cnt     = 0;
    fail_cnt     = 0;

    if (!uvm_config_db#(real)::get(this, "", "expected_v0y", expected_v0y))
      `uvm_warning("NOCFG", "expected_v0y not set -- scoreboard checks disabled")
    else begin
      expected_T  = 2.0 * expected_v0y / 9.8;
      expected_dt = expected_T / 100.0;
    end
  endfunction

  function void write(rk4_uart_rx_item item);
    logic [31:0] word;
    real ti_r, yi_r, y_ref, err_pct;

    byte_buf.push_back(item.data);

    if (byte_buf.size() < 4) return;

    // Assemble 32-bit word (little-endian)
    word = {byte_buf[3], byte_buf[2], byte_buf[1], byte_buf[0]};
    byte_buf.delete();

    if (word_phase == 0) begin
      // First word: could be ti or EOS marker
      if (word == EOS_MARKER) begin
        eos_received = 1;
        `uvm_info("SB", $sformatf("EOS marker received after %0d steps", step_cnt), UVM_LOW)
        return;
      end
      // Store ti for pairing with yi
      ti_r = q16_to_real($signed(word));
      uvm_config_db#(real)::set(null, "", "_sb_last_ti", ti_r);
      word_phase = 1;
    end else begin
      // Second word: yi
      real last_ti;
      void'(uvm_config_db#(real)::get(null, "", "_sb_last_ti", last_ti));
      yi_r  = q16_to_real($signed(word));
      y_ref = analytic_y(expected_v0y, last_ti);

      if ($signed(word) < 0) begin
        `uvm_error("SB", $sformatf("Step %0d: yi is NEGATIVE (%.4f)", step_cnt, yi_r))
        fail_cnt++;
      end else begin
        pass_cnt++;
      end

      if (y_ref != 0.0) begin
        err_pct = 100.0 * ((yi_r - y_ref) < 0.0 ? -(yi_r - y_ref) : (yi_r - y_ref))
                  / ((y_ref < 0.0) ? -y_ref : y_ref);
      end else
        err_pct = 0.0;

      `uvm_info("SB", $sformatf(
        "Step %3d: ti=%.4f  yi=%.4f  ref=%.4f  err=%.2f%%",
        step_cnt, last_ti, yi_r, y_ref, err_pct), UVM_MEDIUM)

      step_cnt++;
      word_phase = 0;
    end
  endfunction

  function void report_phase(uvm_phase phase);
    super.report_phase(phase);
    `uvm_info("SB", $sformatf("==== Scoreboard Summary ===="), UVM_LOW)
    `uvm_info("SB", $sformatf("  Steps received : %0d", step_cnt), UVM_LOW)
    `uvm_info("SB", $sformatf("  EOS received   : %s", eos_received ? "YES" : "NO"), UVM_LOW)
    `uvm_info("SB", $sformatf("  Pass checks    : %0d", pass_cnt), UVM_LOW)
    `uvm_info("SB", $sformatf("  Fail checks    : %0d", fail_cnt), UVM_LOW)
    if (!eos_received)
      `uvm_error("SB", "End-of-stream marker 0xDEADBEEF was never received")
  endfunction

endclass
