`timescale 1ns / 1ps

module ad9434top(
    input               sys_clk,
    input               sys_w_en,
    input               sys_clr_en,
    input               sys_rst_n,

    // selector control
    output              SEL1,
    output              SEL0,
    output              EN0,
    output              EN1,
    output              AD9434_SDIO,
    output              AD9434_SCLK,
    output              AD9434_CSB,
    output              AD9434_PWDN,
    output              CLK_9434_P,
    output              CLK_9434_N,
    // input differiential
    input       [11:0]  adc_data_in_p,
    input       [11:0]  adc_data_in_n,
    input               or_p,
    input               or_n,
    input               dco_p,
    input               dco_n,
    // BRAM PORTB OUTPUT
    output  reg [31:0]  BRAM_PORTB_addrb,
    output              BRAM_PORTB_clkb,
    output  reg [31:0]  BRAM_PORTB_dinb,
    output  reg         BRAM_PORTB_enb,
    input       [31:0]  BRAM_PORTB_doutb,
    output              BRAM_PORTB_rstb,
    output  reg [3:0]   BRAM_PORTB_web,
    output  reg         hdata_finish
);

// ================================== Chip Control For AD9434 ====================================
assign SEL0 = 1'b1;
assign SEL1 = 1'b0;
assign EN0  = 1'b1;
assign EN1  = 1'b0; 
assign AD9434_SDIO = 1'b1;
assign AD9434_CSB = 1'b1;
assign AD9434_SCLK = 1'b1;
assign AD9434_PWDN = 1'b0;
// ================================== Chip Control For AD9434 ====================================

wire clk_out;
wire clk_dco;


// ========================= generate high-frequency-differential clock for 9434 =================
wire clk_200m_bufg;
BUFG BUFG_inst_user_clk (
    .O(clk_200m_bufg), // 1-bit output: Clock output
    .I(sys_clk)
);

wire clk_200m_bufg_oddr;
ODDR #(
    .DDR_CLK_EDGE("OPPOSITE_EDGE"), //"OPPOSITE_EDGE" or "SAME_EDGE"
    .INIT(1'b0),    // Initial value of Q: 1'b0 or 1'b1
    .SRTYPE("SYNC") // Set/Reset type: "SYNC" or "ASYNC"
) ODDR_out_clock_inst_user_clock (
    .Q(clk_200m_bufg_oddr),   // 1-bit DDR output
    .C(clk_200m_bufg),   // 1-bit clock input
    .CE(1'b1), // 1-bit clock enable input
    .D1(1'b1), // 1-bit data input (positive edge)
    .D2(1'b0), // 1-bit data input (negative edge)
    .R(),   // 1-bit reset
    .S()    // 1-bit set
);

OBUFDS OBUFDS_inst_user_clock (
    .O (CLK_9434_P),
    .OB(CLK_9434_N),     // Diff_n output
    .I (clk_200m_bufg_oddr)      // Buffer input
);
// ========================= Generate High-Frequency-Differential Clock for 9434 =================

// ========================= Convert Differiential Signals to Single-Ended =======================
wire [11:0] hdata;
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

// ==================================== Receive High-Speed Data ==================================
// clear addr upper-edge capture

wire clr_flag;
assign clr_flag = sys_clr_en;


// hbram refresh & flush
reg [13:0] hcnt;
reg [31:0] cur_addr;
assign BRAM_PORTB_clkb = clk_out;
assign BRAM_PORTB_rstb = 1'b0;

always @(posedge clk_out or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        cur_addr <= 32'd0;
        BRAM_PORTB_dinb <= 32'd0;
        BRAM_PORTB_addrb <= 32'd0;
        BRAM_PORTB_web <= 4'h0;
        BRAM_PORTB_enb <= 1'b0;
        hcnt <= 14'd0;
        hdata_finish <= 1'b0;
    end
    else if (clr_flag) begin
        BRAM_PORTB_addrb <= 32'd0;
        cur_addr <= 32'd0;
        BRAM_PORTB_dinb <= 32'd0;
        hcnt <= 14'd0;
        hdata_finish <= 1'b0;
    end
    else if(sys_w_en && (hcnt <= 8191)) begin
        hcnt <= hcnt + 1;
        BRAM_PORTB_web <= 4'hf;
        BRAM_PORTB_enb <= 1'b1;
        BRAM_PORTB_dinb <= {20'h9434, hdata};
        BRAM_PORTB_addrb <= cur_addr;
        cur_addr <= cur_addr + 4;
    end
    else if (hcnt == 8192) begin
        hdata_finish <= 1'b1;
        BRAM_PORTB_web <= 4'h0;
        BRAM_PORTB_enb <= 1'b0;
        cur_addr <= 32'd0;
    end
    else begin
        BRAM_PORTB_web <= 4'h0;
        BRAM_PORTB_enb <= 1'b0;
        BRAM_PORTB_dinb <= 32'd0;
        BRAM_PORTB_addrb <= BRAM_PORTB_addrb;
        cur_addr <= 32'd0;
        hdata_finish <= 1'b0;
    end
end
// ==================================== Receive High-Speed Data ==================================

endmodule
