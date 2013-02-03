/*
 * Milkymist SoC - Digilent Nexys 3 port
 * Copyright (C) 2013 Fernand Lone-Sang
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

module psram #(
	parameter adr_width = 22,
	parameter init_timing = 16'h2ee0
) (
	input sys_clk,
	input sys_rst,

	input [31:0] wb_adr_i,
	output reg [31:0] wb_dat_o,
	input [31:0] wb_dat_i,
	input [3:0] wb_sel_i,
	input wb_stb_i,
	input wb_cyc_i,
	output reg wb_ack_o,
	input wb_we_i,

	output [adr_width-1:0] psram_adr,
	inout [15:0] psram_dq,
	output reg psram_oe_n,
	output reg psram_we_n,
	output reg psram_ce_n,
    output reg psram_cre,
    input psram_wait,
    output reg psram_adv_n,
	output psram_clk,
    output psram_lb,
    output psram_ub
);

reg [adr_width-1:0] psram_adr_r;
reg [15:0] psram_do;
reg lsb;
reg clk_en;

/* Clock buffering, needed by Xilinx to drive PSRAM */
ODDR2 #(
	.DDR_ALIGNMENT("NONE"),
	.INIT(1'b0),
	.SRTYPE("ASYNC")
) ODDR2_INST (
	.C0(sys_clk),
	.C1(~sys_clk),   
	.CE(clk_en),
	.D0(1'b1),
	.D1(1'b0),
	.R(sys_rst),
	.S(1'b0),
	.Q(psram_clk)
);

/* Bus Configuration Register (BCR) default values 
 * (see page 24 for register values and meaning)
 */
wire [1:0] reg_sel 		= 2'b10;	// Select Bus Configuration Register
wire op_mode 			= 1'b0;		// Configure synchronous burst access mode
wire acc_lat 			= 1'b0;		// Set variable access latency
wire [2:0] lat_cnt 		= 3'b011;	// Set code 3 latency counter
wire wait_polarity 		= 1'b1;		// Set wait signal as active high
wire wait_config 		= 1'b1;		// Asserts wait 1 cycle before before delay
wire [1:0] drv_strength	= 2'b01;	// Set 1/2 strength
wire burst_wrap 		= 1'b1;		// Force not to wrap
wire [2:0] burst_len	= 3'b111;	// Configure for continous burst

assign two_cycle_transfer = (wb_sel_i == 4'b1111);
assign psram_adr = {psram_adr_r[adr_width-1:1], (two_cycle_transfer) ? lsb : psram_adr_r[0]};
assign psram_dq = psram_oe_n ? psram_do : 16'bz;
assign psram_lb = 1'b1;
assign psram_ub = 1'b1;

reg configure;
reg is_configured;
reg load;
reg store;

/* Count how many cycles psram_ce is asserted low */
reg [7:0] ce_n_counter;
always @(posedge sys_clk) begin
	if(~psram_ce_n)
		ce_n_counter <= ce_n_counter + 1'b1;
	else
		ce_n_counter <= 1'b0;
end

always @(posedge sys_clk) begin
	psram_oe_n <= 1'b1;
	psram_we_n <= 1'b1;
	psram_ce_n <= 1'b1;
	psram_adv_n <= 1'b1;
	psram_cre <= 1'b0;
	clk_en <= 0;

	if(configure) begin
		clk_en <= 1'b1;
		psram_ce_n <= 1'b0;
		psram_do <= {3'b0, reg_sel, 2'b0, op_mode, acc_lat, lat_cnt, 
		             wait_polarity, 1'b0, wait_config, 2'b0, drv_strength, 
					 burst_wrap, burst_len};
		if (~ce_n_counter) begin
			psram_we_n <= 1'b0;
			psram_adv_n <= 1'b0;
			psram_cre <= 1'b1;
		end
	end

	/* Use IOB registers to prevent glitches on address lines */
	/* register only when needed to reduce EMI */
	if(wb_cyc_i & wb_stb_i & is_configured) begin
        clk_en <= 1'b1;
        psram_ce_n <= 1'b0;
		psram_adr_r <= wb_adr_i[adr_width:1];
        if (~ce_n_counter) begin
            psram_adv_n <= 1'b0;
            psram_we_n <= wb_we_i ? 1'b0 : 1'b1;
        end
		if(wb_we_i)
			case(wb_sel_i)
				4'b0011: psram_do <= wb_dat_i[15:0];
				4'b1100: psram_do <= wb_dat_i[31:16];
				default: psram_do <= 16'hxxxx;
			endcase
		else
			psram_oe_n <= 1'b0;
	end

	if(load) begin
		casex({wb_sel_i, lsb})
			5'b0001x: wb_dat_o <= {4{psram_dq[7:0]}};
			5'b0010x: wb_dat_o <= {4{psram_dq[15:8]}};
			5'b0100x: wb_dat_o <= {4{psram_dq[7:0]}};
			5'b1000x: wb_dat_o <= {4{psram_dq[15:8]}};
			5'b0011x: wb_dat_o <= {2{psram_dq}};
			5'b1100x: wb_dat_o <= {2{psram_dq}};
			5'b11110: begin wb_dat_o[31:16] <= psram_dq; lsb <= ~lsb; end
			5'b11111: begin wb_dat_o[15:0]  <= psram_dq; lsb <= ~lsb; end
			default:  wb_dat_o <= 32'hxxxxxxxx;
		endcase
	end

// 	if(store)
// 		psram_we_n <= 1'b0;

	if(sys_rst)
		lsb <= 1'b0;
end

/* Power-Up Initialization of the PSRAM takes at least 150 us to complete
 * (see page 10, "Power-Up Initialization" section) 
 */
reg [15:0] init_counter;
wire init_done = (init_counter == init_timing);

always @(posedge sys_clk) begin
	init_counter <= 16'h0;
	if(~init_done)
		init_counter <= init_counter + 4'd1;
end

parameter POWERUP	= 3'd0;
parameter CONFIGURE	= 3'd1;
parameter IDLE		= 3'd2;
parameter READ		= 3'd3;
parameter WRITE		= 3'd4;
parameter ACK		= 3'd5;

reg [2:0] state;
reg [2:0] next_state;

always @(posedge sys_clk) begin
	if(~init_done)
		state <= POWERUP;	
	else if(sys_rst & is_configured)
		state <= IDLE;
	else
		state <= next_state;
end

always @(*) begin
	next_state = state;
	configure = 1'b0;
	load = 1'b0;
	store = 1'b0;
	wb_ack_o = 1'b0;
	is_configured = (state >= IDLE);

	case(state)
		POWERUP: begin
			if(init_done)
				next_state = CONFIGURE;
		end

		CONFIGURE: begin
			configure = 1'b1;
			if(psram_wait)
				next_state = IDLE;
		end

		IDLE: begin
			if(wb_cyc_i & wb_stb_i) begin
				if(wb_we_i)
					next_state = WRITE;
				else
					next_state = READ;
			end
		end

		READ: begin
			if(psram_wait) begin
				load = 1'b1;
				if(~two_cycle_transfer | lsb)
					next_state = ACK;
			end
		end

		WRITE: begin
			store = 1'b1;
			if(psram_wait)
				next_state = ACK;
		end

		ACK: begin
			wb_ack_o = 1'b1;
			next_state = IDLE;
		end
	endcase
end

endmodule
