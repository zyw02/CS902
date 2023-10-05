`timescale 1ns / 1ps

// it is critical that SYNC not be brought low on a falling edge of SCLK
// it is critical that SYNC be brought high between the 16th and 17th falling edges of SCLK
module dacspi(clk, data, start, done, sclk_div, SYNC, SCLK, DIN);

input clk;
// input rst_n;
input [15:0] data;
input start;
output done;

input [31:0] sclk_div;

output wire SYNC;
output wire SCLK;
output wire DIN;
// wire ser_en;
// reg  ser_start_0 = 1'b0;
// reg	 ser_start_1 = 1'b0;
reg [31:0] sclk_counter = 32'd0;
reg sclk_enabled = 1'b0;
reg my_sclk = 1'b0;


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
	end

	else if (sclk_counter >= (sclk_div - 32'd1)) begin
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
assign SCLK_fall  = (my_sclk == 1'b1) & (sclk_counter == sclk_div - 32'd1);
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

reg [16:0] spi_shift_reg = 17'd0;
reg spi_shift_reg_shift = 1'b0;
reg spi_shift_reg_load = 1'b0;

always @(posedge clk) begin
	// if (!rst_n)
	// 	spi_shift_reg <= 16'b0;
	// else 
    if (spi_shift_reg_load)
		spi_shift_reg <= {1'b0, data[15:0]};
	else if (spi_shift_reg_shift)
		spi_shift_reg <= {spi_shift_reg[15:0], 1'b0};
end

assign DIN = done ? 1'b0 : spi_shift_reg[16];

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

assign SYNC = my_sync;

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
        if (spi_bit_counter == 5'd16 & SCLK_rise)
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
            if (spi_bit_counter == 5'd16 & SCLK_rise2) begin
                
                set_done <= 1;
                assert_sync <= 1'b0;
                deassert_sync <= 1'b1;
                reset_spi_bit_counter <= 1;
                spi_shift_reg_shift <= 0;
            end
            else if (spi_bit_counter == 5'd16 & SCLK_rise3) begin
                clr_sclk_enabled <= 1'b1;
                set_sclk_enabled <= 1'b0;
            end
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



