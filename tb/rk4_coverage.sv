// =============================================================================
//  rk4_coverage.sv -- Functional coverage collector
// =============================================================================
class rk4_coverage extends uvm_subscriber #(rk4_uart_rx_item);

  `uvm_component_utils(rk4_coverage)

  logic [7:0] rx_byte;
  int         byte_cnt;
  logic [31:0] word_buf;
  int         word_byte_idx;
  int         word_phase;    // 0=ti, 1=yi
  int         step_cnt;
  real        last_yi;

  covergroup cg_uart_byte;
    cp_rx_data : coverpoint rx_byte {
      bins low    = {[0:63]};
      bins mid    = {[64:191]};
      bins high   = {[192:255]};
    }
  endgroup

  covergroup cg_rk4_step;
    cp_step_cnt : coverpoint step_cnt {
      bins early  = {[0:24]};
      bins mid    = {[25:74]};
      bins late   = {[75:100]};
    }
  endgroup

  covergroup cg_yi_range;
    cp_yi_value : coverpoint last_yi {
      bins zero_region = {[0:0.5]};
      bins low         = {[0.5:50.0]};
      bins high        = {[50.0:200.0]};
    }
  endgroup

  function new(string name = "rk4_coverage", uvm_component parent = null);
    super.new(name, parent);
    cg_uart_byte = new();
    cg_rk4_step  = new();
    cg_yi_range  = new();
    byte_cnt       = 0;
    word_byte_idx  = 0;
    word_phase     = 0;
    step_cnt       = 0;
    last_yi        = 0.0;
  endfunction

  function void write(rk4_uart_rx_item t);
    rx_byte = t.data;
    cg_uart_byte.sample();
    byte_cnt++;

    word_buf = {t.data, word_buf[31:8]};
    word_byte_idx++;

    if (word_byte_idx == 4) begin
      word_byte_idx = 0;
      if (word_buf == EOS_MARKER) begin
        word_phase = 0;
        return;
      end

      if (word_phase == 1) begin
        last_yi = q16_to_real($signed(word_buf));
        step_cnt++;
        cg_rk4_step.sample();
        cg_yi_range.sample();
      end
      word_phase = word_phase ^ 1;
    end
  endfunction

  function void report_phase(uvm_phase phase);
    super.report_phase(phase);
    `uvm_info("COV", $sformatf("UART byte coverage : %.1f%%", cg_uart_byte.get_coverage()), UVM_LOW)
    `uvm_info("COV", $sformatf("RK4 step coverage  : %.1f%%", cg_rk4_step.get_coverage()), UVM_LOW)
    `uvm_info("COV", $sformatf("Yi range coverage   : %.1f%%", cg_yi_range.get_coverage()), UVM_LOW)
  endfunction

endclass
