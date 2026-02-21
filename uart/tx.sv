// =============================================================================
//  uart_tx.sv  –  8N1 UART transmitter, fully synthesizable
// =============================================================================
module uart_tx #(
    parameter integer BAUD_DIV = 434
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       tx_valid,
    input  wire [7:0] tx_data,
    output reg        tx_ready,
    output reg        tx
);

reg [$clog2(BAUD_DIV+1)-1:0] baud_cnt;
reg [9:0]  shift_reg;   // {stop, data[7:0], start}
reg [3:0]  bit_cnt;     // 10 bits total (start + 8 data + stop)
reg        active;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        baud_cnt <= '0;
        shift_reg<= 10'h3FF;   // idle = all 1s (stop/mark)
        bit_cnt  <= 4'd0;
        active   <= 1'b0;
        tx_ready <= 1'b1;
        tx       <= 1'b1;      // UART idle line high
    end else begin
        if (!active) begin
            tx_ready <= 1'b1;
            tx       <= 1'b1;
            if (tx_valid) begin
                // Load: start(0) + data[7:0] + stop(1)
                shift_reg <= {1'b1, tx_data, 1'b0};
                bit_cnt   <= 4'd0;
                baud_cnt  <= BAUD_DIV[$clog2(BAUD_DIV+1)-1:0] - 1;
                active    <= 1'b1;
                tx_ready  <= 1'b0;
            end
        end else begin
            if (baud_cnt == '0) begin
                baud_cnt  <= BAUD_DIV[$clog2(BAUD_DIV+1)-1:0] - 1;
                tx        <= shift_reg[0];
                shift_reg <= {1'b1, shift_reg[9:1]};  // shift LSB first
                bit_cnt   <= bit_cnt + 4'd1;
                if (bit_cnt == 4'd9) begin             // all 10 bits sent
                    active <= 1'b0;
                end
            end else begin
                baud_cnt <= baud_cnt - 1;
                tx       <= shift_reg[0];  // hold current bit
            end
        end
    end
end

endmodule
