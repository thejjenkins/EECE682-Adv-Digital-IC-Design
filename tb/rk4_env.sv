// =============================================================================
//  rk4_env.sv -- UVM environment: agent + scoreboard + coverage
// =============================================================================
class rk4_env extends uvm_env;

  `uvm_component_utils(rk4_env)

  rk4_uart_agent  agent;
  rk4_scoreboard  sb;
  rk4_coverage    cov;

  function new(string name = "rk4_env", uvm_component parent = null);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    agent = rk4_uart_agent::type_id::create("agent", this);
    sb    = rk4_scoreboard::type_id::create("sb", this);
    cov   = rk4_coverage::type_id::create("cov", this);
  endfunction

  function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);
    agent.ap.connect(sb.rx_imp);
    agent.ap.connect(cov.analysis_export);
  endfunction

endclass
