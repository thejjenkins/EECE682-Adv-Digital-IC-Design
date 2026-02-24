# ==============================================================================
#  Makefile -- Run UVM testbench with Cadence xrun
# ==============================================================================

# Default test
TEST ?= rk4_base_test

# Verbosity: UVM_LOW, UVM_MEDIUM, UVM_HIGH, UVM_DEBUG
VERBOSITY ?= UVM_MEDIUM

# Project root (one level up from tb/)
PROJ := ..

# RTL sources
RTL_SRCS := \
	$(PROJ)/functions/qmul.sv \
	$(PROJ)/functions/f_proj.sv \
	$(PROJ)/uart/rx.sv \
	$(PROJ)/uart/tx.sv \
	$(PROJ)/rk4_ring_osc.sv \
	$(PROJ)/rk4_clock_divider.sv \
	$(PROJ)/rk4_clk_gen.sv \
	$(PROJ)/rk4_projectile.sv \
	$(PROJ)/rk4_top.sv

# TB sources (interface compiled separately, then package, then top)
TB_SRCS := \
	rk4_if.sv \
	rk4_pkg.sv \
	rk4_tb_top.sv

# xrun flags
XRUN_FLAGS := \
	-uvm \
	-access +rwc \
	-timescale 1ns/1ps \
	-incdir . \
	+UVM_TESTNAME=$(TEST) \
	+UVM_VERBOSITY=$(VERBOSITY) \
	-sv

.PHONY: run clean help

run:
	xrun $(XRUN_FLAGS) $(RTL_SRCS) $(TB_SRCS)

clean:
	rm -rf xcelium.d INCA_libs xrun.log xrun.history waves.shm *.dsn *.trn *.err

help:
	@echo "Usage:"
	@echo "  make run                          # Run with default test (rk4_base_test)"
	@echo "  make run TEST=rk4_base_test       # Specify test by name"
	@echo "  make run VERBOSITY=UVM_HIGH       # Increase UVM verbosity"
	@echo "  make clean                        # Remove build artifacts"
