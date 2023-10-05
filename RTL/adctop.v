`timescale 1ns/1ps
module adctop 
(
	input clk,//系统时钟50M
    input [31:0] user_freq,
	// input ext_Rst_n,
	input start_en,//使能单次转换，该信号为单周期有效，高脉冲使能一次转换
	input [2:0]channel,//ADC转换通道选择
	input ADC_DOUT,//ADC转换结果，由ADC输给FPGA
    // input DCAC,       // 区分直流交流，采用不同的SCLK时钟
	output ADC_sclk,//输出sclk时钟，ADC串行数据接口时钟信号
	output ADC_csn,//ADC串行数据接口使能信号,低电平有效
	output ADC_din,//ADC控制信号，由FPGA发送通道控制字给ADC
	output ADC_state,//ADC工作状态，ADC处于转换时为低电平，空闲时为高电平
	output ADC_conv_done,//转换完成信号，完成转换后产生一个时钟周期的高脉冲
	output [11:0] ADC_Datain//缓存ADC_DOUT的数据，ADC转换结果
 
);
reg conv_done = 1'b0;
reg sclk = 1'b1;
reg cs_n = 1'b1;
reg din = 1'b1;
reg [11:0] ADC_Data = 12'd0;
assign ADC_conv_done = conv_done;
assign ADC_Datain = ADC_Data;
assign ADC_din = din;
assign ADC_sclk = sclk;
assign ADC_csn = cs_n;
// parameter AC_FREQ = 160_000;
// // 系统时钟50M,要100K的直流采集速率, 也就是说SCLK频率为1.6M, 50/1.6是一个周期的计数 25/1.6就是要的计数值
// parameter DC_FREQ = 32_000;
localparam SYS_FREQ = 50_000_000;

reg en = 1'b0;//转换使能信号
reg [2:0] r_channel = 3'd0; //通道选择内部寄存器
//在每个使能转换的时候，寄存Channel的值，防止在转换过程中该值发生变化
always@(posedge clk)
	if(start_en)
		r_channel <= channel;
	else
		r_channel <= r_channel;
		
	//产生使能转换信号
always@(posedge clk)
	if(start_en)
		en <= 1'b1;
	else if(conv_done)
		en <= 1'b0;
	else
		en <= en;
// 10K就可以
// localparam total_num = 500/16;
reg [31:0] div_cnt = 32'd0;//slck2x时钟分频器
// assign total_num = SYS_FREQ / DC_FREQ;
//生成2倍SCLK使能时钟计数器
// wire [31:0] total_num;
// always @ (posedge clk) begin
//     total_num = SYS_FREQ / (user_freq * 32);
// endSYS_FREQ
// assign total_num =  user_freq;
always@(posedge clk)
	if(en)begin
        if(div_cnt < user_freq - 1)
            div_cnt <= div_cnt + 32'd1;
        else 
            div_cnt <= 32'd0;
    end
	else
		div_cnt <= 32'd0;
		
reg sclk2x = 1'b0;//slck2x时钟
//生成2倍SCLK使能时钟
always@(posedge clk)
	if(en && div_cnt == user_freq - 1)
		sclk2x <= 1'b1;
	else
		sclk2x <= 1'b0;
		
reg [5:0]sclk2x_num = 6'd0;
//生成序列计数器
always@(posedge clk)
	if(en && sclk2x) begin
		if(sclk2x_num <= 6'd32)
			sclk2x_num <= sclk2x_num + 1'b1;
		else 
			sclk2x_num <= 6'd0;
    end
	else sclk2x_num <= sclk2x_num;
	
reg [11:0] ADC_Data_r = 12'd0;
//序列机实现ADC串行数据接口的数据发送和接收	
always@(posedge clk)
	if(en) begin
		if(sclk2x)
			case(sclk2x_num)
			6'd0: begin 
                cs_n <= 1'b0;
                sclk <= 1'b1;
                din <= 1'b1;
            end
			6'd1: begin 
                sclk <= 1'b0;
            end
			6'd2: begin 
                sclk <= 1'b1;
            end
			6'd3: begin 
                sclk <= 1'b0;
            end
			6'd4: begin 
                sclk <= 1'b1;
            end
	        6'd5: begin 
                sclk <= 1'b0;
                din <= r_channel[2];
            end
	        6'd6: begin 
                sclk <= 1'b1;
            end
			6'd7: begin 
                sclk <= 1'b0;
                din <= r_channel[1];
            end
			6'd8: begin 
                sclk <= 1'b1;
            end
			6'd9: begin 
                sclk <= 1'b0;
                din <= r_channel[0];
            end
			6'd10:begin 
                sclk <= 1'b1;
                ADC_Data_r <= {ADC_Data_r[10:0],ADC_DOUT}; 
            end
			6'd11:begin 
                sclk <= 1'b0;
            end
			6'd12:begin 
                sclk <= 1'b1;
                ADC_Data_r <= {ADC_Data_r[10:0],ADC_DOUT}; 
            end
			6'd13:begin 
                sclk <= 1'b0; 
            end
			6'd14:begin 
                sclk <= 1'b1;
                ADC_Data_r <= {ADC_Data_r[10:0],ADC_DOUT}; 
            end
			6'd15:begin 
                sclk <= 1'b0; 
            end
			6'd16:begin 
                sclk <= 1'b1;
                ADC_Data_r <= {ADC_Data_r[10:0],ADC_DOUT}; 
            end
			6'd17:begin 
                sclk <= 1'b0; 
            end
			6'd18:begin 
                sclk <= 1'b1;
                ADC_Data_r <= {ADC_Data_r[10:0],ADC_DOUT}; 
            end
			6'd19:begin 
                sclk <= 1'b0; 
            end
			6'd20:begin 
                sclk <= 1'b1;
                ADC_Data_r <= {ADC_Data_r[10:0],ADC_DOUT}; 
            end
			6'd21:begin 
                sclk <= 1'b0; 
            end
			6'd22:begin 
                sclk <= 1'b1;
                ADC_Data_r <= {ADC_Data_r[10:0],ADC_DOUT}; 
            end
			6'd23:begin 
                sclk <= 1'b0; 
            end
			6'd24:begin 
                sclk <= 1'b1;
                ADC_Data_r <= {ADC_Data_r[10:0],ADC_DOUT}; 
            end
			6'd25:begin 
                sclk <= 1'b0;
            end
			6'd26:begin 
                sclk <= 1'b1;
                ADC_Data_r <= {ADC_Data_r[10:0],ADC_DOUT}; 
            end
			6'd27:begin 
                sclk <= 1'b0; 
            end
			6'd28:begin 
                sclk <= 1'b1;
                ADC_Data_r <= {ADC_Data_r[10:0],ADC_DOUT}; 
            end
			6'd29:begin 
                sclk <= 1'b0; 
            end
			6'd30:begin 
                sclk <= 1'b1;
                ADC_Data_r <= {ADC_Data_r[10:0],ADC_DOUT}; 
            end
			6'd31:begin 
                sclk <= 1'b0; 
            end
			6'd32:begin 
                sclk <= 1'b1;
                ADC_Data_r <= {ADC_Data_r[10:0],ADC_DOUT}; 
            end
			6'd33:begin 
                cs_n <= 1'b1; 
            end	//每个上升沿，寄存ADC串行数据输出线上的转换结果
			default:begin 
                cs_n <= 1'b1; 
            end //将转换结果输出
			endcase
	end     
	        
	else    
		cs_n <= 1'b1; 
	//转换完成时，将转换结果输出到Data端口，同时产生一个时钟周期的高脉冲信号
	always@(posedge clk)
	if(en && sclk2x && (sclk2x_num == 6'd33))begin
		ADC_Data <= ADC_Data_r; 
		conv_done <= 1'b1;
	end else begin
		ADC_Data <= ADC_Data; 
		conv_done <= 1'b0;
	end
	
	//产生ADC工作状态指示信号
	assign ADC_state= cs_n;
endmodule
