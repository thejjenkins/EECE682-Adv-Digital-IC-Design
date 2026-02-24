// =============================================================================
//  rk4_types.sv -- Shared typedefs, constants, and Q16.16 helpers
// =============================================================================

typedef logic signed [31:0] q16_t;

localparam q16_t G_FIXED     = 32'sd642252;   // 9.8  in Q16.16
localparam q16_t INV6_FIXED  = 32'sd10922;    // 1/6  in Q16.16
localparam q16_t INV_N_FIXED = 32'sd655;      // 1/100 in Q16.16
localparam q16_t INV_G_FIXED = 32'sd6694;     // 1/9.8 in Q16.16

localparam int unsigned NUM_DIV = 100;

localparam logic [31:0] EOS_MARKER = 32'hDEADBEEF;

function automatic real q16_to_real(input q16_t val);
  return $itor(val) / 65536.0;
endfunction

function automatic q16_t real_to_q16(input real val);
  return $rtoi(val * 65536.0);
endfunction

function automatic real analytic_y(input real v0y, input real t);
  return v0y * t - 0.5 * 9.8 * t * t;
endfunction
