module myadctop(
	input sys_clk,//系统时钟200M
    input sys_rst_n,
    input start_en,
    // 200 / 12
	input [2:0]channel,//ADC转换通道选择
	input ADC_DOUT,//ADC转换结果，由ADC输给FPGA
 
	output reg sclk,//输出sclk时钟，ADC串行数据接口时钟信号
	output reg cs_n,//ADC串行数据接口使能信号,低电平有效
	output reg din,//ADC控制信号，由FPGA发送通道控制字给ADC
	output reg conv_done,//转换完成信号，完成转换后产生一个时钟周期的高脉冲
	output reg[11:0]ADC_Data//缓存ADC_DOUT的数据，ADC转换结果
 
);



reg en;//转换使能信号

reg [2:0]r_channel; //通道选择内部寄存器
//在每个使能转换的时候，寄存Channel的值，防止在转换过程中该值发生变化
always@(posedge sys_clk) begin
	if(!sys_rst_n)
		r_channel <= 3'd0;
	else if(start_en)
		r_channel <= channel;
	else
		r_channel <= r_channel;
end
	//产生使能转换信号
always@(posedge sys_clk) begin
	if(!sys_rst_n)
		en <= 1'b0;
	else if(start_en)
		en <= 1'b1;
	else if(conv_done)
		en <= 1'b0;
	else
		en <= en;
end
/*sclk 推荐范围为0.8~3.2Mhz，拟采用1.92M
设置2倍采样频率3.84M，采样13分频*/
reg [3:0]div_cnt;//slck2x时钟分频器  
//生成2倍SCLK使能时钟计数器
always@(posedge sys_clk) begin
	if(!sys_rst_n)
		div_cnt <= 4'd0;
	else if(en)begin
		if(div_cnt < 4'd2)
			div_cnt <= div_cnt + 1'b1;
		else 
			div_cnt <= 4'd0;
	end
	else
		div_cnt <= 4'd0;
end	
reg sclk2x;//slck2x时钟
//生成2倍SCLK使能时钟
always@(posedge sys_clk) begin
	if(!sys_rst_n)
		sclk2x <= 1'b0;
	else if(en && div_cnt == 4'd2)
		sclk2x <= 1'b1;
	else
		sclk2x <= 1'b0;
end
reg [5:0]sclk2x_num;
//生成序列计数器
always@(posedge sys_clk) begin
	if(!sys_rst_n)
		sclk2x_num <= 6'd0;
	else if(en && sclk2x)
	begin
		if(sclk2x_num <= 6'd32)
			sclk2x_num <= sclk2x_num + 1'b1;
		else 
			sclk2x_num <= 6'd0;
    end
	else sclk2x_num <= sclk2x_num;
end
reg [11:0]ADC_Data_r;
//序列机实现ADC串行数据接口的数据发送和接收	
always@(posedge sys_clk) begin
	if(!sys_rst_n)
		begin
			sclk <= 1'b1;
			din <= 1'b1;
			cs_n <= 1'b1;
		end
	else if(en)
	begin
		if(sclk2x)
			case(sclk2x_num)
			6'd0: begin cs_n <= 1'b0;end
			6'd1: begin sclk <= 1'b0;end
			6'd2: begin sclk <= 1'b1;end
			6'd3: begin sclk <= 1'b0;end
			6'd4: begin sclk <= 1'b1;end
	        6'd5: begin sclk <= 1'b0;din <= r_channel[2];end
	        6'd6: begin sclk <= 1'b1;end
			6'd7: begin sclk <= 1'b0;din <= r_channel[1];end
			6'd8: begin sclk <= 1'b1;end
			6'd9: begin sclk <= 1'b0;din <= r_channel[0];end
			6'd10:begin sclk <= 1'b1;ADC_Data_r <= {ADC_Data_r[10:0],ADC_DOUT}; end
			6'd11:begin sclk <= 1'b0;end
			6'd12:begin sclk <= 1'b1;ADC_Data_r <= {ADC_Data_r[10:0],ADC_DOUT}; end
			6'd13:begin sclk <= 1'b0; end
			6'd14:begin sclk <= 1'b1;ADC_Data_r <= {ADC_Data_r[10:0],ADC_DOUT}; end
			6'd15:begin sclk <= 1'b0; end
			6'd16:begin sclk <= 1'b1;ADC_Data_r <= {ADC_Data_r[10:0],ADC_DOUT}; end
			6'd17:begin sclk <= 1'b0; end
			6'd18:begin sclk <= 1'b1;ADC_Data_r <= {ADC_Data_r[10:0],ADC_DOUT}; end
			6'd19:begin sclk <= 1'b0; end
			6'd20:begin sclk <= 1'b1;ADC_Data_r <= {ADC_Data_r[10:0],ADC_DOUT}; end
			6'd21:begin sclk <= 1'b0; end
			6'd22:begin sclk <= 1'b1;ADC_Data_r <= {ADC_Data_r[10:0],ADC_DOUT}; end
			6'd23:begin sclk <= 1'b0; end
			6'd24:begin sclk <= 1'b1;ADC_Data_r <= {ADC_Data_r[10:0],ADC_DOUT}; end
			6'd25:begin sclk <= 1'b0;end
			6'd26:begin sclk <= 1'b1;ADC_Data_r <= {ADC_Data_r[10:0],ADC_DOUT}; end
			6'd27:begin sclk <= 1'b0; end
			6'd28:begin sclk <= 1'b1;ADC_Data_r <= {ADC_Data_r[10:0],ADC_DOUT}; end
			6'd29:begin sclk <= 1'b0; end
			6'd30:begin sclk <= 1'b1;ADC_Data_r <= {ADC_Data_r[10:0],ADC_DOUT}; end
			6'd31:begin sclk <= 1'b0; end
			6'd32:begin sclk <= 1'b1;ADC_Data_r <= {ADC_Data_r[10:0],ADC_DOUT}; end
			6'd33:begin cs_n <= 1'b1; end	//每个上升沿，寄存ADC串行数据输出线上的转换结果
			default:begin cs_n <= 1'b1; end //将转换结果输出
			endcase
	end     
	        
	else    
		cs_n <= 1'b1; 
	//转换完成时，将转换结果输出到Data端口，同时产生一个时钟周期的高脉冲信号
end
always@(posedge sys_clk) begin
	if(!sys_rst_n)begin
		ADC_Data <= 12'd0; 
		conv_done <= 1'b0;
	end else if(en && sclk2x && (sclk2x_num == 6'd33))begin
		ADC_Data <= ADC_Data_r; 
		conv_done <= 1'b1;
	end else begin
		ADC_Data <= ADC_Data; 
		conv_done <= 1'b0;
	end
end
endmodule 