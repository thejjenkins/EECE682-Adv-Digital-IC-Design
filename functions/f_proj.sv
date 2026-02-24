// f(t, y) = v0y - g*t   (all Q16.16)
// Since g*t = qmul(G_FIXED, t_val), this is purely combinational.
function automatic signed [31:0] f_proj;
    input signed [31:0] t_val;
    // v0y captured in register 'v0y' — passed as extra input for synthesis
    input signed [31:0] v0y_in;
    begin
        f_proj = v0y_in - qmul(G_FIXED, t_val);
    end
endfunction