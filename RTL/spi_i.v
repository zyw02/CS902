module spi_i(
	input sys_clk,
	input n_rst,                   //reset (0 - enable reset)
	
	input SCK,
	input SDI,
	input SCS,
	
	output [31:0] revData_o,       // output received data
	output reg revDone_o,          // enable when received done(1 - enable)
	input readDone_i               // read done to start next spi receive(1-enable)
    );
	
	(*KEEP="TRUE"*)reg sck_pre, sck_cur;
	(*KEEP="TRUE"*)reg scs_pre, scs_cur;
	(*KEEP="TRUE"*)reg sdo_buf;
	always @(posedge sys_clk)begin
			sck_cur <= SCK;
			sck_pre <= sck_cur;
			scs_pre <= scs_cur;
			scs_cur <= SCS;
			sdo_buf <= SDI;
	end
	
	(*KEEP="TRUE"*)reg [31:0]	revData;
	(*KEEP="TRUE"*)reg [3:0]	cstate;
	parameter [3:0]	S_IDLE		=	4'b0001,
                    S_READ		=	4'b0010,
                    S_READ_DONE	=	4'b0100,
                    S_WAIT		=	4'b1000;	
	
	assign revData_o = revData;	
	
	always @(posedge sys_clk)begin
		if(!n_rst)begin
			cstate <= S_IDLE;
		end else begin
			case(cstate)
				S_IDLE:begin
						if((scs_cur == 0) && (scs_pre == 1))begin
							cstate <= S_READ;
						end else begin
							cstate <= S_IDLE;
						end
					end
					
				S_READ:begin
						if(scs_cur == 1 && scs_pre == 0) begin
							cstate <= S_READ_DONE;
						end else begin
							cstate <= S_READ;
						end
					end
					
				S_READ_DONE:begin
						cstate <= S_WAIT;
					end
					
				S_WAIT:begin
						if(readDone_i == 1) begin
							cstate <= S_IDLE;
						end else begin
							cstate <= S_WAIT;
						end
					end
					
				default:begin
						cstate <= S_IDLE;
					end
			endcase
		end
	end
	
	always@(posedge sys_clk)begin
		if(!n_rst)begin
			revData <= 0;
			revDone_o <= 0;
		end else begin
			case(cstate)
				S_IDLE:begin
						revDone_o <= 0;
						revData <= 0;
					end
					
				S_READ:begin
						revDone_o <= 0;
						if(sck_cur == 1 && sck_pre == 0)begin	//sck positive edge:RECEIVE DATA
							// 右移
							revData <= {sdo_buf,revData[31:1]};
						end else begin
							revData <= revData;
						end
					end
					
				S_READ_DONE:begin
						revDone_o <= 1;
						revData <= revData;
					end
					
				S_WAIT:begin
						revDone_o <= 1;
						revData <= revData;						
					end
					
				default:begin
						revData <= 0;
						revDone_o <= 0;
					end
			endcase
		end
	end

endmodule