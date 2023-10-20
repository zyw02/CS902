// // 向FREQ0中写0xFFFC000
// // step 1: 控制字D15 D14 = 00, B28(D13) = 1, HLB(D12) = X SDATA写入0010 0000 0000 0000
// // step 2: FREQ0写入低14位 D15,14 = 01 SDATA写入0100 0000 0000 0000
// // step 3: FREQ0写入高14位 D15,14 = 10 SDATA写入0111 1111 1111 1111
// // 输出频率: 25MHz / 2^28 * FREQREG
// //
// module dds_16b
// #(parameter CLKS_PER_BIT = 10)
// (
// 	input clk,
// 	input rst_n,
// 	input en,
// 	// input[15:0] control,
// 	input[27:0] freq,
// 	// output reg good_to_reset_go = 0,
// 	output reg send_complete = 0,
// 	output reg fsync,
// 	output reg sclk,
// 	output reg sdata
// );

// // 定义状态机节点
// localparam IDLE = 4'b0000;
// localparam START_SCLK = 4'b0001;
// localparam START_FSYNC = 4'b0010;
// localparam WORD_TRANSFER_1 = 4'b0011;
// localparam FSYNC_WAIT_HIGH_1 = 4'b0100;
// localparam FSYNC_WAIT_LOW_1 = 4'b0101;
// localparam SEND_COMPLETE = 4'b0110;
// localparam CLEANUP = 4'b0111;

// reg[3:0] state = 0;
// reg[15:0] clk_ctr = 0;
// reg[5:0] bit_ctr = 0;
// reg[2:0] word_ctr = 0;
// wire[15:0] adreg0;
// wire[15:0] adreg1;
// reg [15:0] control = 16'h2100;
// reg [15:0] control_2 = 16'h2000;
// reg [15:0] freq_lsb = 16'h50c7;
// reg [15:0] freq_msb = 16'h4000;
  
//   //reg[15:0] adreg0 = 16'b0100000000001000;
//   //reg[15:0] adreg1 = 16'b0100000000000000;
  
// assign adreg0 = 16'h4000 | freq[13:0];
// assign adreg1 = 16'h4000 | freq[27:14];
  
// always @(posedge clk or negedge rst_n) begin
//     case(state)  
//     IDLE : begin
// 		fsync <= 1;
//         sclk <= 0;
//         sdata <= 0;
//         if (en)
// 			state <= START_SCLK;
//       end
//     START_SCLK : begin
//         if (clk_ctr == 0) begin
// 			sclk <= 1;
// 			// good_to_reset_go <= 1;
//         end
//         if (clk_ctr >= CLKS_PER_BIT * 2) begin
// 			clk_ctr <= 0;
// 			state <= START_FSYNC;
//         end
//         else
// 			clk_ctr <= clk_ctr + 1;
//       end

//     START_FSYNC : begin
//         if (clk_ctr == 0)
// 			fsync <= 0;
//         if (clk_ctr >= CLKS_PER_BIT) begin
// 			clk_ctr <= 0;
// 			state <= WORD_TRANSFER_1;
//         end
//         else
//           clk_ctr <= clk_ctr + 1;
//       end

//     WORD_TRANSFER_1 : begin
//         if (clk_ctr == 0) begin
// 			sclk <= 0;
//         	if (word_ctr == 0)
//             	sdata <= control[15-bit_ctr];
//           	else if (word_ctr == 1)
//             	sdata <= adreg0[15-bit_ctr];
//           	else if (word_ctr == 2)
//             	sdata <= adreg1[15-bit_ctr];
//             else
//               sdata <= control_2[15 - bit_ctr];
//         end
//         if (clk_ctr == CLKS_PER_BIT / 2)
// 			sclk <= 1;
//         if (bit_ctr >= 15 && clk_ctr >= ((CLKS_PER_BIT * 3) / 4)) begin
//           	bit_ctr <= 0;
//           	clk_ctr <= 0;
//           	state <= FSYNC_WAIT_HIGH_1;
//         end
//         else if (clk_ctr >= CLKS_PER_BIT) begin
//           	clk_ctr <= 0;
//           	bit_ctr <= bit_ctr + 1;   
//         end
//         else
// 			clk_ctr <= clk_ctr + 1;     
//     end
//     FSYNC_WAIT_HIGH_1 : begin
//         if (clk_ctr == 0)
// 			fsync <= 1;
//         if (clk_ctr == CLKS_PER_BIT / 4)
// 			sclk <= 0;
//         if (clk_ctr >= CLKS_PER_BIT * 2) begin
//           	clk_ctr <= 0;
//         if (word_ctr >= 3)
//             state <= SEND_COMPLETE;
//         else
//             state <= FSYNC_WAIT_LOW_1;
//         end
//         else
//           	clk_ctr <= clk_ctr + 1;
//     end
//     FSYNC_WAIT_LOW_1 : begin
//         if (clk_ctr == 0)
//           	fsync <= 0;
//         if (clk_ctr >= CLKS_PER_BIT) begin
//           	clk_ctr <= 0;
//           	word_ctr <= word_ctr + 1;
//           	state <= WORD_TRANSFER_1;
//         end
//         else
//           	clk_ctr <= clk_ctr + 1;
//     end
//     SEND_COMPLETE : begin
//         send_complete <= 1;
//         state <= CLEANUP;
//     end
//     CLEANUP : begin
//         send_complete <= 0;
//         // good_to_reset_go <= 0;
//         clk_ctr <= 0;
//         bit_ctr <= 0;
//         word_ctr <= 0;
//         state <= IDLE;
//     end
//     endcase
//   end
// endmodule
`timescale 1ns / 1ps

// it is critical that SYNC not be brought low on a falling edge of SCLK
// it is critical that SYNC be brought high between the 16th and 17th falling edges of SCLK
module ddsspi(clk, data, start, done, sclk_div, FSYNC, SCLK, SDATA);

input clk;
// input rst_n;
input [15:0] data;
input start;
output done;

input [31:0] sclk_div;

output wire FSYNC;
output wire SCLK;
output wire SDATA;
// wire ser_en;
// reg  ser_start_0 = 1'b0;
// reg	 ser_start_1 = 1'b0;
reg [31:0] sclk_counter = 32'd0;
reg sclk_enabled = 1'b0;
reg my_sclk = 1'b1;


// always @(posedge clk or negedge rst_n) begin
//     if(!rst_n) begin
//         ser_start_0 <= 1'b0;   
//         ser_start_1 <= 1'b0; 
//     end
//     else begin
//         ser_start_0 <= start;   
//         ser_start_1 <= ser_start_0;     
//     end
// end
// assign ser_en = ~ser_start_1 & ser_start_0;
assign ser_en = start;

always @(posedge clk) begin
	// if (!rst_n) begin
	// 	sclk_counter <= 32'b0;
	// 	my_sclk = 1'b0;
	// end

	// else 
        if (!sclk_enabled) begin
		sclk_counter <= 32'b0;
        my_sclk = 1'b1;
	end

	else if (sclk_counter == (sclk_div - 32'd1)) begin
		sclk_counter <= 32'b0;
		my_sclk <= !my_sclk;
	end
	else begin
		sclk_counter <= sclk_counter + 32'd1;
	end

end

assign SCLK = my_sclk;
wire SCLK_rise;
wire SCLK_rise2;
wire SCLK_fall;
assign SCLK_rise  = (my_sclk == 1'b0) & (sclk_counter == sclk_div - 32'd1); // SCLK低电平的最后一拍
assign SCLK_rise2 = (my_sclk == 1'b0) & (sclk_counter == sclk_div - 32'd2); // SCLK低电平倒数第二拍
assign SCLK_rise3 = (my_sclk == 1'b0) & (sclk_counter == sclk_div - 32'd3); // SCLK低电平倒数第二拍
assign SCLK_fall  = (my_sclk == 1'b1) & (sclk_counter == sclk_div - 32'd1); // 下降沿即将到来
reg [4:0] spi_bit_counter = 5'd0;
reg inc_spi_bit_counter = 1'b0;
reg reset_spi_bit_counter = 1'b0;

always @(posedge clk) begin
	// if (!rst_n)
	// 	spi_bit_counter <= 5'b0;
	if (reset_spi_bit_counter)
		spi_bit_counter <= 5'b0;
	else if (inc_spi_bit_counter)
		spi_bit_counter <= spi_bit_counter + 5'd1;
	else
		spi_bit_counter <= spi_bit_counter;
end

reg [15:0] spi_shift_reg = 16'd0;
reg spi_shift_reg_shift = 1'b0;
reg spi_shift_reg_load = 1'b0;

always @(posedge clk) begin
	// if (!rst_n)
	// 	spi_shift_reg <= 16'b0;
	// else 
    if (spi_shift_reg_load)
		spi_shift_reg <= data;
	else if (spi_shift_reg_shift)
		spi_shift_reg <= {spi_shift_reg[14:0], 1'b0};
    else if (done) begin
        spi_shift_reg <= {spi_shift_reg[14:0], 1'b0};
    end
end

assign SDATA = done ? 1'b0 : spi_shift_reg[15];

reg set_sclk_enabled = 1'b0;
reg clr_sclk_enabled = 1'b0;

// n - 1周期执行clr
always @(posedge clk) begin
	// if (!rst_n)
	// 	sclk_enabled <= 0;
	// else
        if (set_sclk_enabled)
		sclk_enabled <= 1;
	else if (clr_sclk_enabled)
		sclk_enabled <= 0;
end


reg my_done = 1'b0;
reg set_done = 1'b0;
reg clr_done = 1'b0;

always @(posedge clk) begin
	// if (!rst_n)
	// 	my_done <= 0;
    if (set_done)
		my_done <= 1;
	else if (clr_done)
		my_done <= 0;
end

assign done = my_done;

reg my_sync = 1'b1;
reg assert_sync = 1'b0;
reg deassert_sync = 1'b0;

always @(posedge clk) begin
	// if (!rst_n)
	// 	my_sync <= 1;
	if (assert_sync)
		my_sync <= 0;
	else if (deassert_sync)
		my_sync <= 1;
end

assign FSYNC = my_sync;

parameter SPI_IDLE = 1'b0;
parameter SPI_BITS = 1'b1;

reg state;
reg next_state;

always @(posedge clk) begin
	// if (!rst_n)
	// 	state <= SPI_IDLE;
	// else
		state <= next_state;
end
always @(*) begin
    case(state)
    SPI_IDLE : begin
        if (ser_en)
            next_state = SPI_BITS;
        else
            next_state = state;
    end
    SPI_BITS : begin
        if (spi_bit_counter == 5'd15 & SCLK_fall)
            next_state = SPI_IDLE;
        else
            next_state = state;
    end
    default: next_state = SPI_IDLE;
    endcase
end
always @(posedge clk) begin
    // if (!rst_n) begin
	//     inc_spi_bit_counter <= 1'b0;
	//     reset_spi_bit_counter <= 1'b0;
	//     spi_shift_reg_shift <= 1'b0;
	//     spi_shift_reg_load <= 1'b0;
	//     set_done <= 1'b0;
	//     clr_done <= 1'b0;
	//     assert_sync <= 1'b0;
	//     deassert_sync <= 1'b0;
	//     set_sclk_enabled <= 1'b0;
    //     clr_sclk_enabled <= 1'b0;
    // end
    // else begin
        case(state)
        SPI_IDLE : begin
            clr_done <= 1'b1;
            set_done <= 1'b0;
            if (ser_en) begin
                
                set_sclk_enabled <= 1'b1;
                clr_sclk_enabled <= 1'b0;
                spi_shift_reg_load <= 1'b1;
                reset_spi_bit_counter <= 1'b0;
                assert_sync <= 1'b1;
                deassert_sync <= 1'b0;
            end
            // else
            //     spi_shift_reg_load <= 0;
            // assert_sync使SYNC降为低电平，deassert_sync将SYNC调为高
        end
        SPI_BITS : begin
            spi_shift_reg_load <= 1'b0;
            clr_done <= 1'b0;
            // start <= 0;
            
            // else if (spi_bit_counter == 5'd16 & SCLK_rise2) begin
            // if (spi_bit_counter == 5'd15 & SCLK_rise2) begin
                
            //     set_done <= 1;
            //     assert_sync <= 1'b0;
            //     deassert_sync <= 1'b1;
            //     reset_spi_bit_counter <= 1;
            //     spi_shift_reg_shift <= 0;
            // end
            if (spi_bit_counter == 5'd15 & SCLK_fall) begin
                clr_sclk_enabled <= 1'b1;
                set_sclk_enabled <= 1'b0;
                set_done <= 1;
                assert_sync <= 1'b0;
                deassert_sync <= 1'b1;
                reset_spi_bit_counter <= 1;
                spi_shift_reg_shift <= 0;
            end
            // else if (spi_bit_counter == 5'd15 & SCLK_rise3) begin
            //     clr_sclk_enabled <= 1'b1;
            //     set_sclk_enabled <= 1'b0;
            //     set_done <= 1;
            //     assert_sync <= 1'b0;
            //     deassert_sync <= 1'b1;
            //     reset_spi_bit_counter <= 1;
            //     spi_shift_reg_shift <= 0;
            // end
            else if (SCLK_rise2) begin // 传输过程中, SCLK低电平的最后一拍, SCLK上升沿即将到来
                inc_spi_bit_counter <= 1;
                // spi_shift_reg_load <= 0;
                spi_shift_reg_shift <= 1;
                
            end
            else begin                 // 传输过程中, SYNC低电平有效, 但尚不需要移位
                spi_shift_reg_shift <= 0;
                inc_spi_bit_counter <= 0;
            end
        end
        endcase
    // end
end
endmodule