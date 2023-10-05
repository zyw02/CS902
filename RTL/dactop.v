`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/09/05 21:07:01
// Design Name: 
// Module Name: drv_dac
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module dactop(clk, start, channel, dac, sclk_div, set_channel, set_wtm, SYNC, SCLK, DIN);

input clk;
// input rst_n;
input start;    
input [31:0] sclk_div;
// input [31:0] sample_delay;
input [2:0]  channel;
input [11:0] dac;
output set_channel;
output reg set_wtm;
output wire SYNC;		// SYNC signal going to DAC IC
output wire SCLK;		// SCLK signal going to DAC IC
output wire DIN;		// DIN  signal going to DAC IC


wire start;
reg  start_0;
reg	 start_1;
// 锁存通道值
reg [2:0] r_channel = 0;
// 寄存使能信号
reg en;
reg conv_done ;
// spi通信模块驱动信号
reg serial_start;
wire serial_done;
reg [15:0] serial_data_out;

// spi通信模块(16位移位)
dacspi dacspi_inst(
	.clk(clk),
	.data(serial_data_out),
	.start(serial_start),
	.done(serial_done),
	.sclk_div(sclk_div),
	.SYNC(SYNC),
	.SCLK(SCLK),
	.DIN(DIN)
);


// always @(posedge clk or negedge rst_n) begin
//     if(!rst_n) begin
//         start_0 <= 1'b0;   
//         start_1 <= 1'b0; 
//     end
//     else begin
//         start_0 <= start;   
//         start_1 <= start_0;     
//     end
// end
// assign start_en = ~start_1 & start_0;

always@(posedge clk)
	// if(!rst_n)
	// 	r_channel <= 3'd0;
	// else 
	if(start)
		r_channel <= channel;
	else
		r_channel <= r_channel;
		
	//产生使能转换信号
// reg flag_set_wtm; // flag用于区分是WTM下的spi通信还是进行输出电压设置时的spi通信
// reg flag_ch_set;
// assign set = conv_done & flag;
assign set_channel = conv_done;
always@(posedge clk) begin
	// if(!rst_n) begin
	// 	en <= 1'b0;
	// 	// r_channel <= 3'd0;
	// end
	// else 
		if(start) begin
		en <= 1'b1;
		// r_channel <= channel;
	end
	else if(conv_done)
		en <= 1'b0;
	else begin
		en <= en;
		// r_channel <= r_channel;
	end
end
reg load_serial_data_out;
reg [15:0] data_out;

always @(posedge clk) begin
	// if (!rst_n)
	// 	serial_data_out <= 16'b0;
	// else 
	if (load_serial_data_out)
		serial_data_out <= data_out;
end


// 状态机信号
localparam 	S_SET_WTM 			= 4'b0001;
localparam 	S_CH_SET 			= 4'b0010;		// 通道寄存
localparam 	S_SER_START 		= 4'b0100;		// SPI开始
localparam 	S_SER_WAIT 			= 4'b1000;		// 等待SPI结束

reg 		S_SET_WTM_DONE = 1'b0;
reg 		S_CH_SET_DONE = 1'b0;
reg 		S_SER_START_DONE = 1'b0;
reg 		S_SER_WAIT_DONE = 1'b0;

reg [3:0] state = S_SET_WTM;
reg [3:0] next_state = S_SET_WTM;



always @(posedge clk) begin
	// if (!rst_n) begin
	// 	state				<= S_SET_WTM;
	// 	// set_wtm             <= 1'b0;
		
	// end
	// else
		state <= next_state;
end

always @(*) begin
	case(state)
        S_SET_WTM: begin
			if (S_SET_WTM_DONE) 
				next_state = S_SER_START;
			else
				next_state = state;
        end
        S_CH_SET: begin 
            if (S_CH_SET_DONE) begin
                next_state = S_SER_START;
            end
            else next_state = state;
        end
        S_SER_START : begin
            if (S_SER_START_DONE) begin
                next_state = S_SER_WAIT;
            end
            else next_state = state;
        end
        S_SER_WAIT : begin
            if (S_SER_WAIT_DONE) begin
                next_state = S_CH_SET;
            end
            else next_state = state;
        end
        default : begin 
            next_state = S_SET_WTM;
        end
    endcase
	// serial_done => conv_done                 => conv_done,      => !conv_done
    //                next_state = set_ch       => state = set_ch  => 
    //                flow_vout += 1            => start           => en
end
always @(posedge clk) begin
	// if (!rst_n) begin
	// 	conv_done <= 1'b0;
	// 	serial_start <= 1'b0;
	// 	set_wtm <= 1'b0;
	// 	S_SET_WTM_DONE		<= 1'b0; 
	// 	S_CH_SET_DONE 		<= 1'b0;
	// 	S_SER_START_DONE 	<= 1'b0;
	// 	S_SER_WAIT_DONE 	<= 1'b0;
	// end
	// else begin
		case(state)
		S_SET_WTM : begin
			S_CH_SET_DONE               <= 1'b0;
			S_SER_START_DONE            <= 1'b0;
			S_SER_WAIT_DONE             <= 1'b0;
			load_serial_data_out        <= 1;
			data_out                    <= {4'b1001, 12'b0};
			S_SET_WTM_DONE              <= 1'b1;
			set_wtm                     <= 1'b0;
			conv_done                   <= 1'b0;
		end
		S_CH_SET : begin
			conv_done <= 1'b0;
			set_wtm <= 1'b1;
			if (en) begin
				S_SER_START_DONE        <= 1'b0;
				S_SER_WAIT_DONE         <= 1'b0;
				load_serial_data_out    <= 1;
				data_out                <= {1'b0, r_channel, dac};
				S_CH_SET_DONE           <= 1'b1;
			end
			else begin
				S_CH_SET_DONE <= 1'b0;
			end
		end
		S_SER_START : begin
			S_CH_SET_DONE <= 1'b0;
			serial_start <= 1'b1;
			S_SER_START_DONE <= 1'b1;
		end
		S_SER_WAIT : begin
			serial_start <= 1'b0;
			if (serial_done) begin
				S_SER_WAIT_DONE <= 1'b1;
				conv_done    <= 1'b1;
				
			end
			else begin
				S_SER_WAIT_DONE <= 1'b0;
			end
		end
		default : begin 
            S_CH_SET_DONE               <= 1'b0;
			S_SER_START_DONE            <= 1'b0;
			S_SER_WAIT_DONE             <= 1'b0;
			load_serial_data_out        <= 1;
			data_out                    <= {4'b1001, 12'b0};
			set_wtm                     <= 1'b0;
			conv_done                   <= 1'b0;
        end
		endcase
	// end
end
endmodule



