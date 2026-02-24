# RK4 Projectile Solver -- EECE 682 Advanced Digital IC Design

Synthesizable 4th-order Runge-Kutta (RK4) projectile trajectory solver implemented in SystemVerilog. The design receives an initial vertical velocity over UART, computes the full projectile arc using fixed-point arithmetic, and streams (t, y) data points back over UART.

## Physics Model

The solver integrates the vertical projectile equation:

```
dy/dt = f(t) = v0y - g * t
y(t)  = v0y * t - 0.5 * g * t^2   (analytic solution)
```

- **Total flight time:** `T = 2 * v0y / g`
- **Step size:** `dt = T / N`, where `N = 100` divisions
- **RK4 update:** `y_{i+1} = y_i + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4)`

All arithmetic uses **Q16.16 signed fixed-point** (16 integer bits, 16 fractional bits). Division is avoided by pre-computing reciprocal constants (`1/g`, `1/N`, `1/6`) at elaboration time.

## Project Structure

```
EECE682-Adv-Digital-IC-Design/
|-- rk4_projectile.sv          # Top-level module: FSM + UART RX -> RK4 -> UART TX
|-- tb_rk4_projectile.sv       # Self-checking testbench (Vivado xsim)
|-- functions/
|   |-- qmul.sv                # Q16.16 fixed-point multiply: (a * b) >> 16
|   |-- f_proj.sv              # Projectile derivative: f(t) = v0y - g*t
|-- uart/
|   |-- rx.sv                  # 8N1 UART receiver with mid-bit sampling
|   |-- tx.sv                  # 8N1 UART transmitter
|-- readme.md
```

## Module Descriptions

### `rk4_projectile_top` -- Top Level

12-state FSM that orchestrates the full pipeline:

| State | Description |
|---|---|
| `S_IDLE` | Wait for 4-byte v0y from UART RX |
| `S_INIT` | Compute `dt` and `dt/2` from v0y |
| `S_K1`-`S_K4` | Evaluate RK4 slopes |
| `S_UPDATE` | Compute `y_{i+1}` and advance `t` |
| `S_CHECK` | Halt if `y < 0` or all steps done |
| `S_TX_PREP` / `S_TX_SEND` | Stream `(ti, yi)` over UART (8 bytes, little-endian) |
| `S_DONE_MARK` / `S_DONE` | Send `0xDEADBEEF` end-of-stream marker, return to idle |

### `qmul` -- Fixed-Point Multiply

Combinational Q16.16 multiply: performs a 32x32 signed multiply, then extracts bits [47:16] as the 32-bit result.

### `f_proj` -- Derivative Function

Evaluates `f(t) = v0y - g * t` using `qmul` for the `g * t` term.

### `uart_rx` / `uart_tx` -- UART Peripherals

Standard 8N1 UART with parameterized baud divisor. The receiver includes a double-flop synchronizer for metastability hardening on the async RX pad.

## UART Protocol

**Input:** 4 bytes (little-endian Q16.16) representing `v0y` in m/s.

**Output:** Repeating 8-byte frames `[ti(4B) | yi(4B)]` (little-endian Q16.16), terminated by a 4-byte marker `0xDEADBEEF`.

Default configuration: 115200 baud, 50 MHz clock.

## Testbench

`tb_rk4_projectile.sv` runs three test cases with known analytic solutions:

| Test | v0y (m/s) | Flight Time (s) | dt (s) |
|---|---|---|---|
| TC1 | 49.0 | 10.0 | 0.10 |
| TC2 | 19.6 | 4.0 | 0.04 |
| TC3 | 9.8 | 2.0 | 0.02 |

Checks performed per test case:
- End-of-stream marker `0xDEADBEEF` is received
- All output `yi` values are non-negative
- First-step `yi` is within 5% of analytic value
- Mid-step `yi` (step 50) is within 2% of analytic value

## Target Technology

TSMC 180 nm -- synthesis with Cadence Genus, place-and-route with Cadence Innovus.

## Reference

This project is based on the following paper:

- **"Design and Analysis of a Hardware Accelerator with FPU-Based Runge-Kutta Solvers"** -- [IEEE Xplore](https://ieeexplore.ieee.org/document/10442325)
