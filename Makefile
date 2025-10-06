# Tìm tất cả testbench
TB_FILE := $(wildcard *_tb.sv)
# Chuyển sang file .vvp output
OUTPUT  := $(TB_FILE:.sv=.vvp)

# Rule mặc định
build: $(OUTPUT)

# Rule: cách build 1 file .vvp từ .sv
%.vvp: %.sv
	iverilog -g2012 -o $@ $^
