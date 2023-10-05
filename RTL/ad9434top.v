`timescale 1ns / 1ps

module ad9434top(
    input sys_clk,
    input sys_rst_n,
    // selector control
    output reg SEL1,
    output reg SEL0,
    output reg EN0,
    output reg EN1,
    output reg AD9434_SDIO,
    output reg AD9434_SCLK,
    output reg AD9434_CSB,
    output reg AD9434_PWDN,
    output CLK_9434_P,
    output CLK_9434_N,
    // input differiential
    input [11:0] adc_data_in_p,
    input [11:0] adc_data_in_n,
    input or_p,
    input or_n,
    input dco_p,
    input dco_n,
    output [11:0] hdata,
    output clk_out
);


wire clk_dco;
wire clk_100m;
// reg en0 = 1'b1;
// reg en1 = 1'b0;
// reg sel0 = 1'b1;
// reg sel1 = 1'b0;
// reg ad9434_sclk = 1'b1;
// reg ad9434_sdio = 1'b1;
// reg ad9434_csb = 1'b1;
// reg ad9434_pwdn = 1'b0;

// assign AD9434_SCLK = ad9434_sclk;
// assign AD9434_PWDN = ad9434_pwdn;
// assign AD9434_CSB = ad9434_csb;
// assign AD9434_SDIO = ad9434_sdio;
// assign EN0 = en0;
// assign EN1 = en1;
// assign SEL1 = sel1;
// assign SEL0 = sel0;
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        SEL1 <= 1'b0;
        SEL0 <= 1'b0;
        EN0 <= 1'b0;
        EN1 <= 1'b0;
        AD9434_SDIO <= 1'b1;
        AD9434_SCLK <= 1'b1;
        AD9434_CSB <= 1'b1;
        AD9434_PWDN <= 1'b0;
    end
    else begin
        EN0 <= 1'b1;
        EN1 <= 1'b0;
        SEL0 <= 1'b1;
        SEL1 <= 1'b0;
    end
    // ad9434_pwdn <= 1'b0;
    // ad9434_data <= ad9434_in_data;
    // en0 <= 1'b1;
    // en1 <= 1'b0;
    // sel0 <= 1'b1;
    // sel1 <= 1'b0;
    // ad9434_sclk <= 1'b1;
    // ad9434_sdio <= 1'b1;
    // ad9434_csb <= 1'b1;
    // ad9434_pwdn <= 1'b0;
end

// ========================= generate high-frequency-differential clock for 9434 =================
clk_wiz_0 instance_name
   (
    // Clock out ports
    .clk_out1(clk_100m),     // output clk_out1
   // Clock in ports
    .clk_in1(sys_clk));      // input clk_in1

wire clk_100m_bufg;
BUFG BUFG_inst_user_clk (
    .O(clk_100m_bufg), // 1-bit output: Clock output
    .I(clk_100m)
);

wire clk_100m_bufg_oddr;
ODDR #(
    .DDR_CLK_EDGE("OPPOSITE_EDGE"), //"OPPOSITE_EDGE" or "SAME_EDGE"
    .INIT(1'b0),    // Initial value of Q: 1'b0 or 1'b1
    .SRTYPE("SYNC") // Set/Reset type: "SYNC" or "ASYNC"
) ODDR_out_clock_inst_user_clock (
    .Q(clk_100m_bufg_oddr),   // 1-bit DDR output
    .C(clk_100m_bufg),   // 1-bit clock input
    .CE(1'b1), // 1-bit clock enable input
    .D1(1'b1), // 1-bit data input (positive edge)
    .D2(1'b0), // 1-bit data input (negative edge)
    .R(),   // 1-bit reset
    .S()    // 1-bit set
);

OBUFDS OBUFDS_inst_user_clock (
    .O (CLK_9434_P),
    .OB(CLK_9434_N),     // Diff_n output
    .I (clk_100m_bufg_oddr)      // Buffer input
);
// ========================= Generate High-Frequency-Differential Clock for 9434 =================

// ========================= Convert Differiential Signals to Single-Ended =======================
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
    IBUFDS ibufds_inst
    (
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
    FDRE fdre_in_inst
    (
        .D(data_in_from_pins_delay[i]),
        .C(clk_div),
        .CE(clock_enable),
        .R(io_reset),
        .Q(data_in_to_device_int)
      );
    assign hdata[i] = data_in_to_device_int;
end
endgenerate
// ========================= Convert Differiential Signals to Single-Ended =======================
endmodule
