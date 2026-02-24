`timescale 1ns / 1ps
// =============================================================================
//  rk4_projectile_top.sv
//  Full synthesizable top-level: UART RX → RK4 Projectile Solver → UART TX
//
//  Target : TSMC 180 nm, Cadence Genus/Innovus
//  Format : Q16.16 signed fixed-point throughout
//
//  Physics model
//    f(ti, yi) = v0y - g*ti          (dy/dt for vertical projectile)
//    yi+1      = yi + h/6*(k1+2k2+2k3+k4)
//    T         = 2*v0y / g           (total flight time)
//    ti        = (T / N) * step_i    (N = NUM_DIVISIONS = 100)
//    dt        = T / N               (fixed step size h)
//

module rk4_projectile_top #(
    parameter integer CLK_FREQ     = 50_000_000, // Hz  – set to your chip clock
    parameter integer BAUD_RATE    = 115_200,
    parameter integer NUM_DIV      = 100,        // number of RK4 steps
    // Q16.16 constants ----------------------------------------------------------
    // g = 9.8  → round(9.8 * 65536) = 642,252
    parameter signed [31:0] G_FIXED    = 32'sd642252,
    // 1/6 in Q16.16 → round((1/6) * 65536) = 10922
    parameter signed [31:0] INV6_FIXED = 32'sd10922,
    // 1/NUM_DIV in Q16.16 → round((1/100) * 65536) = 655
    parameter signed [31:0] INV_N_FIXED = 32'sd655
) (
    input  wire clk,
    input  wire rst_n,       // active-low async reset (tie to pad)
    input  wire uart_rx,
    output wire uart_tx
);

// ---------------------------------------------------------------------------
// 0.  Shared clock divider for UART
// ---------------------------------------------------------------------------
localparam integer BAUD_DIV = CLK_FREQ / BAUD_RATE;  // clocks per bit

// ---------------------------------------------------------------------------
// 1.  UART RX  – 8N1, receives one byte at a time
// ---------------------------------------------------------------------------
wire       rx_valid;   // pulses 1 clk when rx_data is ready
wire [7:0] rx_data;

uart_rx #(.BAUD_DIV(BAUD_DIV)) u_rx (
    .clk      (clk),
    .rst_n    (rst_n),
    .rx       (uart_rx),
    .rx_valid (rx_valid),
    .rx_data  (rx_data)
);

// ---------------------------------------------------------------------------
// 2.  UART TX  – 8N1, accepts one byte at a time
// ---------------------------------------------------------------------------
wire       tx_ready;   // high when TX is idle and can accept a new byte
reg        tx_valid;   // pulse to send tx_data
reg  [7:0] tx_data;

uart_tx #(.BAUD_DIV(BAUD_DIV)) u_tx (
    .clk      (clk),
    .rst_n    (rst_n),
    .tx_valid (tx_valid),
    .tx_data  (tx_data),
    .tx_ready (tx_ready),
    .tx       (uart_tx)
);

// ---------------------------------------------------------------------------
// 3.  Receive v0y (4 bytes, little-endian Q16.16)
// ---------------------------------------------------------------------------
reg signed [31:0] v0y_reg;
reg  [1:0]        rx_byte_cnt;   // counts 0..3
reg               v0y_ready;     // goes high for 1 cycle when all 4 bytes in

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        v0y_reg     <= 32'sd0;
        rx_byte_cnt <= 2'd0;
        v0y_ready   <= 1'b0;
    end else begin
        v0y_ready <= 1'b0;
        if (rx_valid) begin
            case (rx_byte_cnt)
                2'd0: v0y_reg[ 7: 0] <= rx_data;
                2'd1: v0y_reg[15: 8] <= rx_data;
                2'd2: v0y_reg[23:16] <= rx_data;
                2'd3: begin
                    v0y_reg[31:24] <= rx_data;
                    v0y_ready      <= 1'b1;
                end
            endcase
            rx_byte_cnt <= (rx_byte_cnt == 2'd3) ? 2'd0 : rx_byte_cnt + 2'd1;
        end
    end
end

// ---------------------------------------------------------------------------
// 5.  Pre-compute dt = T/N = (2*v0y/g) / N  in Q16.16
//
//     We avoid a true divider by using the pre-computed constants:
//       T     = 2 * v0y * inv_g        (inv_g = 1/9.8 in Q16.16 = 6694)
//       dt    = T * inv_N              (inv_N = 1/100 in Q16.16 = 655)
//     Both multiplications are done with qmul.
//
//     inv_g = round(1/9.8 * 65536) = 6694
// ---------------------------------------------------------------------------
localparam signed [31:0] INV_G_FIXED = 32'sd6694;

// ---------------------------------------------------------------------------
// 6.  RK4 core FSM
// ---------------------------------------------------------------------------
// States
localparam [3:0]
    S_IDLE      = 4'd0,   // waiting for v0y
    S_INIT      = 4'd1,   // compute dt, reset counters
    S_K1        = 4'd2,   // compute k1 = f(ti, yi)
    S_K2        = 4'd3,   // compute k2 = f(ti+dt/2, yi+dt/2*k1)
    S_K3        = 4'd4,   // compute k3 = f(ti+dt/2, yi+dt/2*k2)
    S_K4        = 4'd5,   // compute k4 = f(ti+dt,   yi+dt*k3)
    S_UPDATE    = 4'd6,   // yi+1 = yi + dt/6*(k1+2k2+2k3+k4)
    S_CHECK     = 4'd7,   // check y >= 0
    S_TX_PREP   = 4'd8,   // load transmit shift register
    S_TX_SEND   = 4'd9,   // clock bytes out over UART
    S_DONE_MARK = 4'd10,  // send 0xDEADBEEF marker
    S_DONE      = 4'd11;  // halt until reset

reg [3:0] state;

// RK4 working registers
reg signed [31:0] yi;         // current y
reg signed [31:0] ti;         // current time  (Q16.16)
reg signed [31:0] dt;         // step size  T/N  (Q16.16)
reg signed [31:0] dt_half;    // dt/2
reg signed [31:0] k1, k2, k3, k4;
reg signed [31:0] v0y;        // latched v0y

reg [6:0] step_cnt;           // 0 .. NUM_DIV-1  (7 bits covers 0-127)

// TX shift register: holds up to 8 bytes (ti + yi) to transmit
reg [63:0] tx_shift;          // shift register, LSB first
reg [3:0]  tx_bytes_left;     // how many bytes remain to send
reg [3:0]  tx_total;          // total bytes for current transmission

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state        <= S_IDLE;
        yi           <= 32'sd0;
        ti           <= 32'sd0;
        dt           <= 32'sd0;
        dt_half      <= 32'sd0;
        k1           <= 32'sd0;
        k2           <= 32'sd0;
        k3           <= 32'sd0;
        k4           <= 32'sd0;
        v0y          <= 32'sd0;
        step_cnt     <= 7'd0;
        tx_shift     <= 64'd0;
        tx_bytes_left<= 4'd0;
        tx_total     <= 4'd0;
        tx_valid     <= 1'b0;
        tx_data      <= 8'd0;
    end else begin
        // Default: de-assert tx_valid each cycle (re-asserted below if needed)
        tx_valid <= 1'b0;

        case (state)

            // -----------------------------------------------------------------
            S_IDLE: begin
                if (v0y_ready) begin
                    v0y   <= v0y_reg;
                    state <= S_INIT;
                end
            end

            // -----------------------------------------------------------------
            // Compute dt = 2 * v0y * inv_g * inv_N
            //            = qmul(qmul(v0y << 1, INV_G_FIXED), INV_N_FIXED)
            // yi = 0, ti = 0
            // -----------------------------------------------------------------
            S_INIT: begin
                begin : compute_dt
                    reg signed [31:0] two_v0y;
                    reg signed [31:0] T_total;
                    two_v0y = v0y_reg <<< 1;              // 2*v0y  (Q16.16)
                    T_total = qmul(two_v0y, INV_G_FIXED); // T = 2*v0y/g
                    dt      <= qmul(T_total, INV_N_FIXED);// dt = T/N
                    dt_half <= qmul(T_total, INV_N_FIXED) >>> 1; // dt/2
                end
                yi       <= 32'sd0;
                ti       <= 32'sd0;
                step_cnt <= 7'd0;
                state    <= S_K1;
            end

            // -----------------------------------------------------------------
            // k1 = f(ti, yi) = v0y - g*ti
            // -----------------------------------------------------------------
            S_K1: begin
                k1    <= f_proj(ti, v0y);
                state <= S_K2;
            end

            // -----------------------------------------------------------------
            // k2 = f(ti + dt/2,  yi + dt/2 * k1)
            // -----------------------------------------------------------------
            S_K2: begin
                k2    <= f_proj(ti + dt_half, v0y);
                // Note: yi argument is unused by f(t) since f depends only on t
                // f(ti, yi) = v0y - g*ti  → only ti matters
                state <= S_K3;
            end

            // -----------------------------------------------------------------
            // k3 = f(ti + dt/2,  yi + dt/2 * k2)
            // -----------------------------------------------------------------
            S_K3: begin
                k3    <= f_proj(ti + dt_half, v0y);
                state <= S_K4;
            end

            // -----------------------------------------------------------------
            // k4 = f(ti + dt,  yi + dt * k3)
            // -----------------------------------------------------------------
            S_K4: begin
                k4    <= f_proj(ti + dt, v0y);
                state <= S_UPDATE;
            end

            // -----------------------------------------------------------------
            // yi+1 = yi + qmul(dt, (k1 + 2*k2 + 2*k3 + k4)) * inv6
            //      = yi + qmul(qmul(dt, k_sum), INV6_FIXED)
            // ti+1 = ti + dt
            // -----------------------------------------------------------------
            S_UPDATE: begin
                begin : update_y
                    reg signed [31:0] k_sum;
                    reg signed [31:0] weighted;
                    k_sum    = k1 + (k2 <<< 1) + (k3 <<< 1) + k4;
                    weighted = qmul(qmul(dt, k_sum), INV6_FIXED);
                    yi      <= yi + weighted;
                end
                ti       <= ti + dt;
                step_cnt <= step_cnt + 7'd1;
                state    <= S_CHECK;
            end

            // -----------------------------------------------------------------
            // Halt if y < 0 or all steps done
            // -----------------------------------------------------------------
            S_CHECK: begin
                if (yi[31]) begin          // MSB = sign bit; 1 → negative
                    state <= S_DONE_MARK;
                end else if (step_cnt == NUM_DIV[6:0]) begin
                    state <= S_DONE_MARK;
                end else begin
                    state <= S_TX_PREP;    // send (ti, yi) then continue
                end
            end

            // -----------------------------------------------------------------
            // Load 8-byte shift register: {yi[31:0], ti[31:0]} little-endian
            // tx_shift[7:0] = first byte to send = ti[7:0]
            // -----------------------------------------------------------------
            S_TX_PREP: begin
                // Pack: bytes 0-3 = ti (LE), bytes 4-7 = yi (LE)
                tx_shift      <= {yi[31:24], yi[23:16], yi[15:8], yi[7:0],
                                  ti[31:24], ti[23:16], ti[15:8], ti[7:0]};
                tx_bytes_left <= 4'd8;
                tx_total      <= 4'd8;
                state         <= S_TX_SEND;
            end

            // -----------------------------------------------------------------
            // Shift out bytes LSB-first; wait for tx_ready between each byte
            // -----------------------------------------------------------------
            S_TX_SEND: begin
                if (tx_bytes_left == 4'd0) begin
                    // All bytes sent; proceed to next RK4 step
                    state <= S_K1;
                end else if (tx_ready) begin
                    // Send the next byte (currently in bits [7:0] after shift)
                    tx_data       <= tx_shift[7:0];
                    tx_valid      <= 1'b1;
                    tx_shift      <= {8'd0, tx_shift[63:8]};  // shift right 8
                    tx_bytes_left <= tx_bytes_left - 4'd1;
                end
            end

            // -----------------------------------------------------------------
            // Send 0xDEADBEEF (4 bytes, little-endian: EF BE AD DE)
            // -----------------------------------------------------------------
            S_DONE_MARK: begin
                tx_shift      <= {32'd0, 8'hDE, 8'hAD, 8'hBE, 8'hEF};
                tx_bytes_left <= 4'd4;
                tx_total      <= 4'd4;
                state         <= S_DONE;
                // Reuse S_TX_SEND for sending but land in S_DONE after
                // (handled by adding a flag below — simplified: drain inline)
            end

            // -----------------------------------------------------------------
            S_DONE: begin
                // Drain the end-of-stream marker then halt
                if (tx_bytes_left == 4'd0) begin
                    state <= S_IDLE; // ready for next v0y from UART
                end else if (tx_ready) begin
                    tx_data       <= tx_shift[7:0];
                    tx_valid      <= 1'b1;
                    tx_shift      <= {8'd0, tx_shift[63:8]};
                    tx_bytes_left <= tx_bytes_left - 4'd1;
                end
            end

        endcase
    end
end

endmodule
