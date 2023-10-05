`timescale 1ns/1ps

module ad9434in (
    // From the system into the device
    input  [11:0] adc_data_in_p,  // adc数据(+)
    input  [11:0] adc_data_in_n,  // adc数据(-)
    output [11:0] hdata,
    input or_p,
    input or_n,
    input              dco_p,      // 差分时钟输入(+)
    input              dco_n,      // 差分时钟输入(-)
    output             clk_out    // 时钟输出
);
wire clock_enable = 1'b1;
wire io_reset = 1'b0;
// Signal declarations
////------------------------------
// After the buffer
wire [11:0] data_in_from_pins_int; // adc数据-差分转单端
// Between the delay and serdes
wire [11:0] data_in_from_pins_delay; // adc数据延迟
// Create the clock logic
wire clk_in_buf;
wire clk_div;
wire int_or;

IBUFDS ibufds_clk_inst
(
    .I(dco_p),
    .IB(dco_n),
    .O(clk_in_buf)
);
IBUFDS ibufds_or_inst
(
    .I(or_p),
    .IB(or_n),
    .O(int_or)
);
  
// BUFR generates the slow clock
BUFR clkout_buf_inst
(
    .O (clk_div),
    .CE(1'b1),
    .CLR(1'b0),
    .I (clk_in_buf)
);

assign clk_out = clk_div; // This is regional clock;
// 生成块
genvar i;
generate for (i = 0; i < 12; i = i + 1) begin: pins
    // Instantiate the buffers
    ////------------------------------
    // Instantiate a buffer for every bit of the data bus
    IBUFDS ibufds_inst (
        .I(adc_data_in_p[i]),
        .IB(adc_data_in_n[i]),
        .O(data_in_from_pins_int[i])
    );

    // Pass through the delay
    ////-------------------------------
   assign data_in_from_pins_delay[i] = data_in_from_pins_int[i];
 
    // Connect the delayed data to the fabric
    ////--------------------------------------

   // SDR模式，上升沿采集数据
    wire data_in_to_device_int;
    (* IOB = "true" *)
    FDRE fdre_in_inst (
        .D(data_in_from_pins_delay[i]),
        .C(clk_div),
        .CE(clock_enable),
        .R(io_reset),
        .Q(data_in_to_device_int)
    );
    assign hdata[i] = data_in_to_device_int;
end
endgenerate

always @(posedge clk_out) begin
    
end
endmodule

