// ---------------------------------------------------------------------------
// 4.  Q16.16 fixed-point multiply helper (function → combinational logic)
//     result = (a * b) >> 16    (signed)
// ---------------------------------------------------------------------------
function automatic signed [31:0] qmul;
    input signed [31:0] a, b;
    reg signed [63:0] tmp;
    begin
        tmp  = a * b;           // 32×32 → 64-bit signed product
        qmul = tmp[47:16];      // shift right 16 = divide by 2^16
    end
endfunction