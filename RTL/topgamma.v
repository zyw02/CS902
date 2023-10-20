`timescale 1ns/1ps

/*
* ENGINEER : Wu Ziyan
* DATETIME : 2023/10/05 UTC-8 11:13
* REVISIONS : AMP DAC CLEAR OPERATION ADDED, ADC WORKFLOWS ADDED
*             DCDC : SAMPLE_CNT 50 -> 51
*             BRAM : added clr_flag (logic or) to judge if the addrb needed to be set to 0
*             ADC : 1M SAMPLE REVISION (CLK_TRG)
*             AMP : SAMPLE RATE CHANGED TO 500K
* VERSION : V0.23.1005             
* CONTENT : Unit modules test workflow (DCDC, AMP, ADC)
* TODO : PARAMETERS OPTIMIZATION 
*/

module topgamma #(
    parameter VIN_10 = 12'd1885,
    parameter VIN_8 = 12'd1310,
    parameter VC_66 = 12'd1081,
    parameter VC_0 = 12'd0,
    parameter VC_5 = 12'd820,
    // parameter DELAY_CNT = 32'd2,
    parameter DELAY_CNT = 32'd40_000_000,
    
    parameter BIAS_VE = 12'd2050,
    parameter BIAS_VE_RIRO = 12'd1640,
    parameter BIAS_VG_RIRO = 12'd328,
    parameter BIAS_VG_CMRR = 12'd2704,
    parameter BIAS_VG_ADC = 12'd2248,
    parameter BIAS_VD = 12'd1640,
    parameter FREQ_23_LSB = 16'h40f7,
    parameter FREQ_23_MSB = 16'h4000,
    parameter FREQ_200_LSB = 16'h4864,
    parameter FREQ_200_MSB = 16'h4000,
    parameter FREQ_20K_LSB = 16'h46dc,
    parameter FREQ_20K_MSB = 16'h400d
) (
    input               sys_clk,
    input               sys_rst_n,

    input               start_dcdc,
    input               start_amp,
    input               start_uadc,
    input               start_uu,
    input               start_ulo,
    input               start_uxo,
    input               start_uf,
    input               start_us,

    output              ADC_SCLK,
    output              ADC_CSN,
    output              ADC_DIN,
    input               ADC_DOUT,

    output              SYNC_DAC,
    output              SCLK_DAC,
    output              DIN_DAC,
    
    output              FSYNC_DDS,
    output              SCLK_DDS,
    output              SDATA_DDS,
    output              CLK_25M,

    input      [7:0]    uadc_data,
    output     reg      uadc_clk,

    output              clkb,
    output              rstb,
    output reg          enb,
    output reg [3:0]    web,
    output reg [31:0]   addrb,
    output reg [31:0]   dinb,
    input      [31:0]   doutb,
    
    output reg          ven_key,
    output reg          vf_key,
    output              AMP_KEY1, // CMRR_IN+
    output              AMP_KEY2, // CMRR_IN-
    output              AMP_KEY3, // CMRR_O //cmrr
    output              AMP_KEY4, // RI_RES //
    output              AMP_KEY5, // RI_IN+ //
    output              AMP_KEY6, // RI_O
    output              AMP_KEY7, // RI_IN-
    output              AMP_KEY8, // VOS_RESIN+
    output              AMP_KEY9, // VOS_IN+
    output              AMP_KEY10, // VOS_RESIN-
    output              AMP_KEY11, // VOS_O
    output              AMP_KEY12, // VOS_IN-
    output              AMP_KEY13, // RO remains low voltage
    output              AMP_VOSKEY,
    output              AMP_IOSKEY,
    output              AMP_RIROKEY,
    output              AMP_CMRRKEY,
    output              Unit_ampsin_key,
    output              Unit_adcsin_key,
    output reg          Unit_adc_key,
    output reg          Unit_xo_key,
    output reg          Unit_sram_key,
    output reg          Unit_fpga_key,
    output reg          Unit_logic_key,
    output reg          Unit_uart_key,

    output reg  [7:0]   Unit_finish,

    output reg  [15:0]  ps_tracker,
    // 9434 reuse
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
    output      [31:0]  BRAM_PORTB_addrb,
    output              BRAM_PORTB_clkb,
    output      [31:0]  BRAM_PORTB_dinb,
    output              BRAM_PORTB_enb,
    input       [31:0]  BRAM_PORTB_doutb,
    output              BRAM_PORTB_rstb,
    output      [3:0]   BRAM_PORTB_web,

    // UNIT UART INOUT
    input               uu_rxd,
    output              RS422_DI,

    // UNIT LOGIC INOUT
    output              ulo_txd,
    input               ulo_rxd,
    // UNIT SRAM INOUT
    output      [17:0]  SRAM_A,       // ADDR DATA BUS
    inout       [15:0]  SRAM_DB,      // INOUT DATA BUS
    output              SRAM_WE_N,    // WRITE ENABLE
    output              SRAM_OE_N,    // READOUT ENABLE
    output              SRAM_UB_N,    // MSB ENABLE
    output              SRAM_LB_N,    // LSB ENABLE
    output              SRAM_CE_N,    // CHIP ENABLE
    // UNIT FPGA INOUT
    input               SCK,
    input               SDI,
    input               SCS,
    output  reg         start_fpga,                
    // error_count
    output  reg [31:0]  digital_err_cnt
);

// ========================= CONTROLLING SIGNALS FOR RELAY DAC ADC DDS BRAM ===================
// DAC & DDS SCLK DIV
reg [31:0] sclk_div;

// ADC128S102
wire adc_state;
wire adc_done;
reg start_adc;
reg [2:0]chan_adc;
wire [11:0] adc_data;
reg  [15:0] data_2_bram;
reg  [31:0] user_freq;

// ADC128S102 SAMPLING CONTROL
reg [31:0] sum;
reg [31:0] sample_cnt;
reg [8:0] sample_flag;

// DAC
reg start_dac;
wire dac_done_wtm;
wire dac_done_channel;
reg [2:0] chan_dac;
reg [11:0] vout;

// DDS
reg dds_start;
wire dds_done;
reg [15:0] user_lsb;
reg [15:0] user_msb;

// SIMULATION INTERFACE RESERVATION
assign adc_en = start_adc;
assign one_conv_done = adc_done;

// AMP KEYS DEFINITION
reg cmrr_in_p;
assign AMP_KEY1 = cmrr_in_p;
reg cmrr_in_n;
assign AMP_KEY2 = cmrr_in_n;
reg cmrr_o;
assign AMP_KEY3 = cmrr_o;
reg ri_res;
assign AMP_KEY4 = ri_res;
reg ri_in_p;
assign AMP_KEY5 = ri_in_p;
reg ri_o;
assign AMP_KEY6 = ri_o;
reg ri_in_n;
assign AMP_KEY7 = ri_in_n;
reg vos_resin_p;
assign AMP_KEY8 = vos_resin_p;
reg vos_in_p;
assign AMP_KEY9 = vos_in_p;
reg vos_resin_n;
assign AMP_KEY10 = vos_resin_n;
reg vos_o;
assign AMP_KEY11 = vos_o;
reg vos_in_n;
assign AMP_KEY12 = vos_in_n;
reg ro;
assign AMP_KEY13 = ro;
reg voskey;
reg ioskey;
reg rirokey;
reg cmrrkey;
assign AMP_VOSKEY = voskey;
assign AMP_IOSKEY = ioskey;
assign AMP_RIROKEY = rirokey;
assign AMP_CMRRKEY = cmrrkey;

// 9833 OUTPUT KEY SELECTION
reg ampsinkey;
reg adcsinkey;
assign Unit_adcsin_key = adcsinkey;
assign Unit_ampsin_key = ampsinkey;

// BRAM-CONTROLLING SIGNALS
reg start_wr;
wire clr_flag;

// 9434 CONTROLLING SIGNALS
reg hdata_w_en;
reg hdata_clr_en;
wire hdata_w_finish;
// ========================= CONTROLLING SIGNALS FOR RELAY DAC ADC DDS BRAM ===================

// ============================ DIGITAL CHIPS ERROR COUNT TEMPORARY VALUE =====================
// UNIT UART CONTROL
reg  rs422_en;
wire rs422_finish;
wire [31:0] rs422_err_count;

// UNIT LOGIC CONTROL
reg not_en;
wire not_finish;
wire [15:0] not_err_count;


// UNIT FPGA CONTROL
reg spi_recv_ack;
wire spi_recvd;
wire [31:0] spi_recv_data;

// UNIT SRAM CONTROL
reg sram_read_en;
wire sram_read_finish;
wire [31:0] usram_err_count;

// ========================= DIGITAL CHIPS ERROR COUNT TEMPORARY VALUE ========================

// ================================= 1M CLOCKING FOR AD9280 ===================================
reg [7:0] clkcnt;
reg [8:0] clk_trg;
reg clk_sram_i;
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        clk_trg <= 9'd0; 
    end
    else if (clk_trg < 199) begin
        clk_trg <= clk_trg + 1;
    end
    else
        clk_trg <= 9'd0;
end
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        clkcnt <= 8'd0;
        uadc_clk <= 1'b0;
        
    end
    else if (clkcnt < 99) begin
        clkcnt <= clkcnt + 1;
    end
    else begin
        clkcnt <= 8'd0;
        uadc_clk <= ~uadc_clk;
    end
end
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        clkcnt <= 8'd0;
        clk_sram_i <= 1'b0;
        
    end
    else if (clkcnt < 199) begin
        clkcnt <= clkcnt + 1;
    end
    else begin
        clk_sram_i <= ~clk_sram_i;
        clkcnt <= 8'd0;
    end
end
wire clk_sram;
BUFG BUFG_inst (
      .O(clk_sram), // 1-bit output: Clock output
      .I(clk_sram_i)  // 1-bit input: Clock input
);
// ================================= 1M CLOCKING FOR AD9280 ===================================

// ============================== CLOCKING FOR AD9833 & AD9434=================================
clk_wiz_0 instance_name
(
 // Clock out ports
 .clk_out1(CLK_25M),     // output clk_out1
// Clock in ports
 .clk_in1(sys_clk));     // input clk_in1
// ============================== CLOCKING FOR AD9833 & AD9434=================================

// ================================= SYSTEM DELAY =============================================
reg delay_en;
reg delay_done;
reg [31:0] delay_cnt;
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        delay_cnt <= 32'd0;
        delay_done <= 1'b0;
    end
    else if (delay_en) begin
        if (delay_cnt < DELAY_CNT - 1) begin
            delay_done <= 1'b0;
            delay_cnt  <= delay_cnt + 32'd1;
        end
        else begin
            delay_done <= 1'b1;
            delay_cnt  <= 32'd0;
        end
    end
    else begin
        delay_cnt  <= 32'd0;
        delay_done <= 1'b0;
    end
end
localparam DELAY_RIRO = 32'd100_000_000;
reg dly_riro_en;
reg dly_riro_done;
reg [31:0] dly_riro_cnt;
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        dly_riro_cnt <= 32'd0;
        dly_riro_done <= 1'b0;
    end
    else if (dly_riro_en) begin
        if (dly_riro_cnt < DELAY_RIRO - 1) begin
            dly_riro_done <= 1'b0;
            dly_riro_cnt  <= dly_riro_cnt + 32'd1;
        end
        else begin
            dly_riro_done <= 1'b1;
            dly_riro_cnt  <= 32'd0;
        end
    end
    else begin
        dly_riro_cnt  <= 32'd0;
        dly_riro_done <= 1'b0;
    end
end
// ================================= SYSTEM DELAY =============================================

// =================================PS START SIGNAL PROCESSING=================================
// DCDC STARTUP TRIGGER
wire dcdc_flag;        
reg dcdc_en_0;
reg dcdc_en_1;
assign dcdc_flag = ~dcdc_en_1 & dcdc_en_0;
always @(posedge sys_clk or negedge sys_rst_n) begin
    if(!sys_rst_n) begin
        dcdc_en_0 <= 1'b0;   
        dcdc_en_1 <= 1'b0; 
    end
    else begin
        dcdc_en_0 <= start_dcdc;
        dcdc_en_1 <= dcdc_en_0;     
    end
end

// AMP STARTUP TRIGGER
wire amp_flag;        
reg amp_en_0;
reg amp_en_1;
assign amp_flag = ~amp_en_1 & amp_en_0;
always @(posedge sys_clk or negedge sys_rst_n) begin
    if(!sys_rst_n) begin
        amp_en_0 <= 1'b0;   
        amp_en_1 <= 1'b0; 
    end
    else begin
        amp_en_0 <= start_amp;
        amp_en_1 <= amp_en_0;     
    end
end

// UNIT ADC STARTUP TRIGGER
wire uadc_flag;        
reg uadc_en_0;
reg uadc_en_1;
assign uadc_flag = ~uadc_en_1 & uadc_en_0;
always @(posedge sys_clk or negedge sys_rst_n) begin
    if(!sys_rst_n) begin
        uadc_en_0 <= 1'b0;   
        uadc_en_1 <= 1'b0; 
    end
    else begin
        uadc_en_0 <= start_uadc;
        uadc_en_1 <= uadc_en_0;     
    end
end

// UNIT UART STARTUP TRIGGER
wire uu_flag;        
reg uu_en_0;
reg uu_en_1;
assign uu_flag = ~uu_en_1 & uu_en_0;
always @(posedge sys_clk or negedge sys_rst_n) begin
    if(!sys_rst_n) begin
        uu_en_0 <= 1'b0;   
        uu_en_1 <= 1'b0; 
    end
    else begin
        uu_en_0 <= start_uu;
        uu_en_1 <= uu_en_0;     
    end
end

// UNIT LOGIC STARTUP TRIGGER
wire ulo_flag;        
reg ulo_en_0;
reg ulo_en_1;
assign ulo_flag = ~ulo_en_1 & ulo_en_0;
always @(posedge sys_clk or negedge sys_rst_n) begin
    if(!sys_rst_n) begin
        ulo_en_0 <= 1'b0;   
        ulo_en_1 <= 1'b0; 
    end
    else begin
        ulo_en_0 <= start_ulo;
        ulo_en_1 <= ulo_en_0;     
    end
end

// UNIT Xtal Oscillator STARTUP TRIGGER
wire uxo_flag;        
reg uxo_en_0;
reg uxo_en_1;
assign uxo_flag = ~uxo_en_1 & uxo_en_0;
always @(posedge sys_clk or negedge sys_rst_n) begin
    if(!sys_rst_n) begin
        uxo_en_0 <= 1'b0;   
        uxo_en_1 <= 1'b0; 
    end
    else begin
        uxo_en_0 <= start_uxo;
        uxo_en_1 <= uxo_en_0;     
    end
end

// UNIT FPGA STARTUP TRIGGER
wire uf_flag;        
reg uf_en_0;
reg uf_en_1;
assign uf_flag = ~uf_en_1 & uf_en_0;
always @(posedge sys_clk or negedge sys_rst_n) begin
    if(!sys_rst_n) begin
        uf_en_0 <= 1'b0;   
        uf_en_1 <= 1'b0; 
    end
    else begin
        uf_en_0 <= start_uf;
        uf_en_1 <= uf_en_0;     
    end
end

// UNIT SRAM STARTUP TRIGGER
wire us_flag;        
reg us_en_0;
reg us_en_1;
assign us_flag = ~us_en_1 & us_en_0;
always @(posedge sys_clk or negedge sys_rst_n) begin
    if(!sys_rst_n) begin
        us_en_0 <= 1'b0;   
        us_en_1 <= 1'b0; 
    end
    else begin
        us_en_0 <= start_us;
        us_en_1 <= us_en_0;     
    end
end
// ================================= PS START SIGNAL PROCESSING =================================

// ================================= FSM NODE DEFINITION ========================================
localparam IDLE = 5'd0;
localparam DCDC_ICC = 5'd1;
localparam DCDC_VOUT = 5'd2;
localparam DCDC_LINE = 5'd3;
localparam DCDC_LOAD = 5'd4;
localparam AMP_VOS = 5'd5;
localparam AMP_IOS_N = 5'd6;
localparam AMP_IOS_P = 5'd7;
localparam AMP_RIRO = 5'd8;
localparam AMP_CMRR = 5'd9;
localparam UNITADC = 5'd10;
localparam UNITUART = 5'd11;
localparam UNITLOGIC = 5'd12;
localparam UNITXO = 5'd13;
localparam UNITFPGA = 5'd14; 
localparam UNITSRAM = 5'd15;


// STATE CHANGE FLAG DEFINITION
reg [4:0] cstate = IDLE;
reg [4:0] nstate = IDLE;
reg DCDC_VOUT_DONE;
reg DCDC_ICC_DONE;
reg DCDC_LOAD_DONE;
reg DCDC_LINE_DONE;
reg AMP_VOS_DONE;
reg AMP_IOS_N_DONE;
reg AMP_IOS_P_DONE;
reg AMP_RIRO_DONE;
reg AMP_CMRR_DONE;
reg UNITADC_DONE;
reg UNITUART_DONE;
reg UNITLOGIC_DONE;
reg UNITXO_DONE;
reg UNITFPGA_DONE;
reg UNITSRAM_DONE;

// CHILD STATE SEQ TRACKER (FOR PS TRACKER)
reg [5:0] dcdc_seq;
reg [5:0] amp_seq;
reg [3:0] uadc_seq;
reg [2:0] uu_seq;
reg [2:0] ulo_seq;
reg [2:0] uxo_seq;
reg [2:0] ufpga_seq;
reg [2:0] usram_seq;
// ================================= FSM NODE DEFINITION ========================================

// ================================= Three-Block FSM ============================================
// The First block of a 3-block state machine
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        cstate <= IDLE;
    end
    else
        cstate <= nstate;
end
// The Second block of a 3-block state machine
always @ (*) begin 
    case(cstate)
        IDLE: begin 
            if (dcdc_flag) begin
                nstate = DCDC_ICC;
            end
            else if (amp_flag) begin
                nstate = AMP_VOS;
            end
            else if (uadc_flag) begin
                nstate = UNITADC;
            end
            else if (uu_flag) begin
                nstate = UNITUART;
            end
            else if (ulo_flag) begin
                nstate = UNITLOGIC;
            end
            else if (uxo_flag) begin
                nstate = UNITXO;
            end
            else if (us_flag) begin
                nstate = UNITSRAM;
            end
            else if (uf_flag) begin
                nstate = UNITFPGA;
            end    
            else begin
               nstate = cstate; 
            end
        end
        DCDC_ICC: begin
            if (DCDC_ICC_DONE) begin
                nstate = DCDC_VOUT;
            end
            else begin
                nstate = cstate;
            end
        end
        DCDC_VOUT: begin 
            if (DCDC_VOUT_DONE) begin
                nstate = DCDC_LINE;
            end
            else begin
                nstate = cstate;
            end
        end
        DCDC_LINE : begin
            if (DCDC_LINE_DONE) begin
                nstate = DCDC_LOAD;
            end
            else begin
                nstate = cstate;
            end
        end
        DCDC_LOAD : begin
            if (DCDC_LOAD_DONE) begin
                nstate = IDLE;
            end
            else begin
                nstate = cstate;
            end
        end
        AMP_VOS: begin 
            if (AMP_VOS_DONE) begin
                nstate = AMP_IOS_N;
            end
            else begin
                nstate = cstate;
            end
        end
        AMP_IOS_N: begin
            if (AMP_IOS_N_DONE) begin
                nstate = AMP_IOS_P;
            end
            else begin
                nstate = cstate;
            end
        end
        AMP_IOS_P : begin
            if (AMP_IOS_P_DONE) begin
                nstate = AMP_RIRO;
            end
            else begin
                nstate = cstate;
            end
        end
        AMP_RIRO : begin
            if (AMP_RIRO_DONE) begin
                nstate = AMP_CMRR;
            end
            else begin
                nstate = cstate;
            end
        end
        AMP_CMRR : begin
            if (AMP_CMRR_DONE) begin
                nstate = IDLE;
            end
            else begin
                nstate = cstate;
            end
        end
        UNITADC : begin
            if (UNITADC_DONE) begin
                nstate = IDLE;
            end
            else begin
                nstate = cstate;
            end
        end
        UNITUART : begin
            if (UNITUART_DONE) begin
                nstate = IDLE;
            end
            else begin
                nstate = cstate;
            end
        end
        UNITLOGIC : begin
            if (UNITLOGIC_DONE) begin
                nstate = IDLE;
            end
            else begin
                nstate = cstate;
            end 
        end
        UNITXO : begin
            if (UNITXO_DONE) begin
                nstate = IDLE;
            end
            else begin
                nstate = cstate;
            end 
        end
        UNITSRAM : begin
            if (UNITSRAM_DONE) begin
                nstate = IDLE;
            end
            else begin
                nstate = cstate;
            end 
        end
        UNITFPGA : begin
            if (UNITFPGA_DONE) begin
                nstate = IDLE;
            end
            else begin
                nstate = cstate;
            end 
        end
        default : begin 
            nstate = IDLE;
        end
    endcase
end
// The Third block of a 3-block state machine
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        // Relay Control
        vf_key <= 1'b0;
        ven_key <= 1'b0;
        voskey  <= 1'b0; 
        ioskey  <= 1'b0;
        rirokey <= 1'b1;
        cmrrkey <= 1'b0;
        cmrr_in_p <= 1'b0;
        cmrr_in_n <= 1'b0;
        cmrr_o <= 1'b0;
        ri_res <= 1'b1;
        ri_in_p <= 1'b0;
        ri_o <= 1'b0;
        ri_in_n <= 1'b0;
        vos_resin_p <= 1'b0;
        vos_in_p <= 1'b0;
        vos_resin_n <= 1'b0;
        vos_o <= 1'b0;
        vos_in_n <= 1'b0;
        ro <= 1'b0;
        ampsinkey <= 1'b0;
        adcsinkey <= 1'b0;
        Unit_adc_key <= 1'b0;
        Unit_xo_key <= 1'b0; 
        Unit_sram_key <= 1'b0;
        Unit_fpga_key <= 1'b0;
        Unit_logic_key <= 1'b0;
        Unit_uart_key <= 1'b0;

        // FSM Rotation Flags
        DCDC_VOUT_DONE <= 1'b0;
        DCDC_ICC_DONE <= 1'b0;
        DCDC_LOAD_DONE <= 1'b0;
        DCDC_LINE_DONE <= 1'b0;
        AMP_VOS_DONE <= 1'b0;
        AMP_IOS_P_DONE <= 1'b0;
        AMP_IOS_N_DONE <= 1'b0;
        AMP_RIRO_DONE <= 1'b0;
        AMP_CMRR_DONE <= 1'b0;
        UNITADC_DONE <= 1'b0;
        UNITUART_DONE <= 1'b0;
        UNITLOGIC_DONE <= 1'b0;
        UNITXO_DONE <= 1'b0;
        UNITSRAM_DONE <= 1'b0;
        UNITFPGA_DONE <= 1'b0;

        // DAC ADC DDS Control Signals
        sclk_div <= 32'd32;
        start_dac <= 1'b0;
        start_adc <= 1'b0;
        start_wr <= 1'b0;
        sum <= 32'd0;
        delay_en <= 1'b0;
        vout <= 12'd0;
        chan_dac <= 3'd0;
        chan_adc <= 3'd0;
        data_2_bram <= 16'd0;
        user_freq <= 32'd0;
        dds_start <= 1'b0;
        user_lsb <= 16'd0;
        user_msb <= 16'd0;
        sample_cnt <= 32'd0;

        // Digital Chips Control
        digital_err_cnt <= 32'd0;
        not_en <= 1'b0;
        rs422_en <= 1'b0;
        start_fpga <= 1'b0;
        spi_recv_ack <= 1'b0;
        sram_read_en <= 1'b0;

        // Sequential Control Flags
        dcdc_seq <= 6'd0;
        amp_seq <= 6'd0;
        uadc_seq <= 4'd0;
        uu_seq <= 3'd0;
        ulo_seq <= 3'd0;
        uxo_seq <= 3'd0;
        usram_seq <= 3'd0;
        ufpga_seq <= 3'd0;
        // 9434 top control signals
        hdata_clr_en <= 1'b0;
        hdata_w_en <= 1'b0;

        // Finish-Signals for PS
        Unit_finish <= 8'd0;
    end
    else
        case (cstate)
        IDLE: begin
            // Relay Control
            vf_key <= 1'b0;
            ven_key <= 1'b0;
            voskey  <= 1'b0; 
            ioskey  <= 1'b0;
            rirokey <= 1'b0;
            cmrrkey <= 1'b0;
            cmrr_in_p <= 1'b0;
            cmrr_in_n <= 1'b0;
            cmrr_o <= 1'b0;
            ri_res <= 1'b1;
            ri_in_p <= 1'b0;
            ri_o <= 1'b0;
            ri_in_n <= 1'b0;
            vos_resin_p <= 1'b0;
            vos_in_p <= 1'b0;
            vos_resin_n <= 1'b0;
            vos_o <= 1'b0;
            vos_in_n <= 1'b0;
            ro <= 1'b0;
            ampsinkey <= 1'b0;
            adcsinkey <= 1'b0;
            Unit_adc_key <= 1'b0;
            Unit_xo_key <= 1'b0; 
            Unit_sram_key <= 1'b0;
            Unit_fpga_key <= 1'b0;
            Unit_logic_key <= 1'b0;
            Unit_uart_key <= 1'b0;
            
            // DAC ADC DDS Control Signals 
            dds_start <= 1'b0;
            sclk_div <= 32'd32;
            start_dac   <= 1'b0;
            start_adc   <= 1'b0;
            start_wr    <= 1'b0;
            sum         <= 32'd0;
            delay_en    <= 12'd0;
            vout        <= 12'd0;
            delay_en    <= 1'b0;
            chan_dac    <= 3'd0;
            chan_adc    <= 3'd0;
            data_2_bram <= 16'd0;
            user_freq   <= 32'd0;
            sample_cnt  <= 32'd0;
            sample_flag <= 9'd199;

            // Digital Chips Control
            not_en <= 1'b0;
            rs422_en <= 1'b0;
            start_fpga <= 1'b0;
            spi_recv_ack <= 1'b0;
            sram_read_en <= 1'b0;

            // Sequential Control Flags
            dcdc_seq <= 6'd0;
            amp_seq <= 6'd0;
            uadc_seq <= 4'd0;
            uu_seq <= 3'd0;
            ulo_seq <= 3'd0;
            uxo_seq <= 3'd0;
            usram_seq <= 3'd0;
            ufpga_seq <= 3'd0;

            // FSM Rotation Flags
            DCDC_VOUT_DONE <= 1'b0;
            DCDC_ICC_DONE <= 1'b0;
            DCDC_LOAD_DONE <= 1'b0;
            DCDC_LINE_DONE <= 1'b0;
            AMP_VOS_DONE   <= 1'b0;
            AMP_IOS_P_DONE <= 1'b0;
            AMP_IOS_N_DONE <= 1'b0;
            AMP_RIRO_DONE  <= 1'b0;
            AMP_CMRR_DONE  <= 1'b0;
            UNITADC_DONE <= 1'b0;
            UNITUART_DONE <= 1'b0;
            UNITLOGIC_DONE <= 1'b0;
            UNITXO_DONE <= 1'b0;
            UNITSRAM_DONE <= 1'b0;
            UNITFPGA_DONE <= 1'b0;
            // 9434 top control signals
            hdata_clr_en <= 1'b0;
            hdata_w_en <= 1'b0;

            // finish
            Unit_finish <= Unit_finish;
        end
        DCDC_ICC: begin
            case(dcdc_seq)
            6'd0 : begin
                Unit_finish <= 8'd0;
                if (dac_done_wtm) begin
                    ven_key <= 1'b0;
                    vout <= VC_0;
                    chan_dac <= 3'd2;
                    start_dac <= 1'b1;
                    dcdc_seq <= dcdc_seq + 1;
                end
                else begin
                    dcdc_seq <= dcdc_seq;
                end
            end
            6'd1 : begin
                start_dac <= 1'b0;
                if (dac_done_channel) begin
                    dcdc_seq <= dcdc_seq + 1;
                end
                else begin
                    dcdc_seq <= dcdc_seq;
                end
            end
            6'd2 : begin
                chan_dac <= 3'd1;
                vout <= VIN_10;
                start_dac <= 1'b1;
                dcdc_seq <= dcdc_seq + 1;
            end
            6'd3 : begin
                start_dac <= 1'b0;
                if (dac_done_channel) begin
                    delay_en <= 1'b1;
                    dcdc_seq <= dcdc_seq + 1;
                end
                else begin
                    dcdc_seq <= dcdc_seq;
                end
            end
            6'd4 : begin
                if (delay_done) begin
                    delay_en <= 1'b0;
                    start_adc <= 1'b1;
                    chan_adc <= 3'd3;
                    dcdc_seq <= dcdc_seq + 1;
                end
                else begin
                    dcdc_seq <= dcdc_seq;
                end
            end
            6'd5 : begin
                if ((sample_cnt < 50) && adc_done) begin
                    start_adc <= 1'b1;
                    chan_adc <= 3'd3;
                    sample_cnt <= sample_cnt + 1;
                    sum <= sum + adc_data;
                end
                else if (sample_cnt == 50) begin
                    start_adc <= 1'b0;
                    sample_cnt <= 0;
                    dcdc_seq <= dcdc_seq + 1;
                    sum <= sum / 50;
                end
            end
            6'd6 : begin
                start_wr <= 1'b1;
                data_2_bram <= {4'b1000,sum[11:0]};
                dcdc_seq <= dcdc_seq + 1;
            end
            6'd7 : begin
                sum <= 32'd0;
                start_wr <= 1'b0;
                DCDC_ICC_DONE <= 1'b1;
                dcdc_seq <= dcdc_seq + 1;
            end
            default :;
            endcase
        end
        DCDC_VOUT: begin     
            case(dcdc_seq)
            6'd8 : begin
                if (dac_done_wtm) begin
                    ven_key      <= 1'b1;
                    chan_dac     <= 3'd1;
                    vout         <= VIN_10; 
                    start_dac    <= 1'b1;
                    dcdc_seq <= dcdc_seq + 1;
                end
                else begin
                    dcdc_seq <= dcdc_seq;
                end
            end
            6'd9 : begin
                if (dac_done_channel) begin
                    start_dac <= 1'b0;
                    dcdc_seq <= dcdc_seq + 1;
                end
                else begin
                    start_dac <= 1'b0;
                    dcdc_seq <= dcdc_seq;
                end
            end
            6'd10 : begin
                chan_dac <= 3'd2;
                vout <= VC_66; 
                start_dac <= 1'b1;
                dcdc_seq <= dcdc_seq + 1;
            end
            6'd11 : begin
                if (dac_done_channel) begin
                    start_dac <= 1'b0;
                    dcdc_seq <= dcdc_seq + 1;
                end
                else begin
                    start_dac <= 1'b0;
                    dcdc_seq <= dcdc_seq;
                end
            end
            6'd12 : begin
                delay_en <= 1'b1;
                dcdc_seq <= dcdc_seq + 1;
            end
            6'd13 : begin
                if (delay_done) begin
                    delay_en <= 1'b0;
                    start_adc <= 1'b1;
                    chan_adc <= 3'd1;
                    dcdc_seq <= dcdc_seq + 1;
                end
                else begin
                    dcdc_seq <= dcdc_seq;
                end
            end
            6'd14 : begin
                if ((sample_cnt < 50) && adc_done) begin
                    start_adc <= 1'b1;
                    sample_cnt <= sample_cnt + 1;
                end
                else if ((sample_cnt >= 50) && (sample_cnt < 100) && adc_done) begin
                    start_adc <= 1'b1;
                    sample_cnt <= sample_cnt + 1;
                    sum <= sum + adc_data;
                end
                else if (sample_cnt == 100) begin
                    start_adc <= 1'b0;
                    sample_cnt <= 0;
                    dcdc_seq <= dcdc_seq + 1;
                    sum <= sum / 50;
                end
            end
            6'd15 : begin
                start_wr <= 1'b1;
                data_2_bram <= {4'b0010,sum[11:0]};
                dcdc_seq <= dcdc_seq + 1;
            end
            6'd16 : begin
                start_wr <= 1'b0;
                sum <= 32'd0;
                DCDC_VOUT_DONE <= 1'b1;
                dcdc_seq <= dcdc_seq + 1;
            end
            default :;
            endcase
        end
        DCDC_LINE: begin
            case(dcdc_seq)
            6'd17 : begin
                if (dac_done_wtm) begin
                    ven_key <= 1'b1;
                    vout <= VC_66;
                    chan_dac <= 3'd2;
                    start_dac   <= 1'b1;
                    dcdc_seq <= dcdc_seq + 1;
                end
                else begin
                    dcdc_seq <= dcdc_seq;
                end 
            end
            6'd18 : begin
                if (dac_done_channel) begin
                    start_dac   <= 1'b0;
                    dcdc_seq <= dcdc_seq + 1;
                end
                else begin
                    start_dac   <= 1'b0;
                    dcdc_seq <= dcdc_seq;
                end
            end
            6'd19: begin
                chan_dac     <= 3'd1;
                vout         <= VIN_8; 
                start_dac    <= 1'b1;
                dcdc_seq <= dcdc_seq + 1;
            end
            6'd20 : begin
                if (dac_done_channel) begin
                    start_dac   <= 1'b0;
                    delay_en <= 1'b1;
                    dcdc_seq <= dcdc_seq + 1;
                end
                else begin
                    start_dac   <= 1'b0;
                    dcdc_seq <= dcdc_seq;
                end
            end
            6'd21 : begin
                if (delay_done) begin
                    delay_en <= 1'b0;
                    chan_adc <= 3'd1;
                    start_adc <= 1'b1;
                    dcdc_seq <= dcdc_seq + 1;
                end
                else begin
                    dcdc_seq <= dcdc_seq;
                end
            end
            6'd22 : begin
                if ((sample_cnt < 50) && adc_done) begin
                    start_adc <= 1'b1;
                    sample_cnt <= sample_cnt + 1;
                end
                else if ((sample_cnt >= 50) && (sample_cnt < 100) && adc_done) begin
                    start_adc <= 1'b1;
                    sample_cnt <= sample_cnt + 1;
                    sum <= sum + adc_data;
                end
                else if (sample_cnt == 100) begin
                    start_adc <= 1'b0;
                    sample_cnt <= 0;
                    dcdc_seq <= dcdc_seq + 1;
                    sum <= sum / 50;
                end
            end
            6'd23 : begin
                start_wr <= 1'b1;
                data_2_bram <= {4'b0111,sum[11:0]};
                dcdc_seq <= dcdc_seq + 1;
            end
            6'd24 : begin
                start_wr <= 1'b0;
                sum <= 32'd0;
                DCDC_LINE_DONE <= 1'b1;
                dcdc_seq <= dcdc_seq + 1;
            end
            default :;
            endcase
        end
        DCDC_LOAD: begin            
            case(dcdc_seq)
            6'd25 : begin
                if (dac_done_wtm) begin
                    ven_key      <= 1'b1;
                    chan_dac     <= 3'd2;
                    vout         <= VC_5; 
                    start_dac    <= 1'b1;
                    dcdc_seq <= dcdc_seq + 1;
                end
                else begin
                    dcdc_seq <= dcdc_seq;
                end 
            end
            6'd26 : begin
                if (dac_done_channel) begin
                    start_dac    <= 1'b0;
                    delay_en     <= 1'b1;
                    dcdc_seq <= dcdc_seq + 1;
                end
                else begin
                    start_dac <= 1'b0;
                    dcdc_seq <= dcdc_seq;
                end
            end
            6'd27 : begin
                if (delay_done) begin
                    delay_en <= 1'b0;
                    start_adc <= 1'b1;
                    chan_adc <= 3'd1;
                    dcdc_seq <= dcdc_seq + 1;
                end
                else begin
                    dcdc_seq <= dcdc_seq;
                end
            end
            6'd28 : begin
                if ((sample_cnt < 50) && adc_done) begin
                    start_adc <= 1'b1;
                    sample_cnt <= sample_cnt + 1;
                end
                else if ((sample_cnt >= 50) && (sample_cnt < 100) && adc_done) begin
                    start_adc <= 1'b1;
                    sample_cnt <= sample_cnt + 1;
                    sum <= sum + adc_data;
                end
                else if (sample_cnt == 100) begin
                    start_adc <= 1'b0;
                    sample_cnt <= 0;
                    dcdc_seq <= dcdc_seq + 1;
                    sum <= sum / 50;
                end
            end
            6'd29 : begin
                start_wr <= 1'b1;
                data_2_bram <= {4'b0100,sum[11:0]};
                dcdc_seq <= dcdc_seq + 1;
            end
            6'd30 : begin
                start_wr <= 1'b0;
                vout <= VC_0;
                chan_dac <= 3'd2;
                start_dac <= 1'b1;
                dcdc_seq <= dcdc_seq + 1;
            end
            6'd31 : begin
                if (dac_done_channel) begin
                    start_dac <= 1'b0;
                    dcdc_seq <= dcdc_seq + 1;
                end
                else begin
                    start_dac <= 1'b0;
                    dcdc_seq <= dcdc_seq;
                end
            end
            6'd32 : begin
                vout <= VC_0;
                chan_dac <= 3'd1;
                start_dac <= 1'b1;
                dcdc_seq <= dcdc_seq + 1;
            end
            6'd33 : begin
                if (dac_done_channel) begin
                    start_dac <= 1'b0;
                    dcdc_seq <= dcdc_seq + 1;
                end
                else begin
                    start_dac <= 1'b0;
                    dcdc_seq <= dcdc_seq;
                end
            end
            6'd34 : begin
                ven_key  <= 1'b0;
                vf_key   <= 1'b0;
                start_wr <= 1'b0;
                sum      <= 32'd0;
                DCDC_LOAD_DONE <= 1'b1;
                Unit_finish <= 8'b00000001;
                dcdc_seq <= 6'd0;
            end
            default :;
            endcase
        end
        AMP_VOS: begin
            case(amp_seq)
            6'd0 : begin
                Unit_finish <= 8'd0;
                if (dac_done_wtm) begin
                    vos_in_n <= 1'b1;
                    vos_in_p <= 1'b1;
                    vos_o <= 1'b1;
                    vos_resin_n <= 1'b1;
                    vos_resin_p <= 1'b1;
                    voskey <= 1'b1;
                    amp_seq <= amp_seq + 1;
                    vout <= BIAS_VE;
                    chan_dac <= 3'd4;
                    start_dac <= 1'b1;
                end
                else begin
                    amp_seq <= amp_seq;
                end
            end
            6'd1 : begin
                start_dac <= 1'b0;
                if (dac_done_channel) begin
                    amp_seq <= amp_seq + 1;
                end
                else begin
                    amp_seq <= amp_seq;
                end
            end
            6'd2 : begin
                delay_en <= 1'b1;
                amp_seq <= amp_seq + 1;
            end
            6'd3 : begin
                if (delay_done) begin
                    delay_en <= 1'b0;
                    chan_adc <= 3'd0;
                    start_adc <= 1'b1;
                    amp_seq <= amp_seq + 1;
                end
                else
                    amp_seq <= amp_seq;
            end
            6'd4 : begin
                if ((sample_cnt < 10000) && adc_done) begin
                    start_adc <= 1'b1;
                    chan_adc <= 3'd0;
                    sum <= sum + adc_data;
                    sample_cnt <= sample_cnt + 1;
                    amp_seq <= amp_seq;
                end
                else if (sample_cnt == 10000) begin
                    sample_cnt <= 0;
                    start_adc <= 1'b0;
                    amp_seq <= amp_seq + 1;
                    sum <= sum / 10000;
                end
            end
            6'd5 : begin
                start_wr <= 1'b1;
                data_2_bram <= {4'b0000,sum[11:0]};
                amp_seq <= amp_seq + 1;
            end
            6'd6 : begin
                start_wr     <= 1'b0;
                sum          <= 32'd0;
                amp_seq <= amp_seq + 1;
            end
            6'd7 : begin
                AMP_VOS_DONE <= 1'b1;
                voskey       <= 1'b0;
                amp_seq <= amp_seq + 1;
            end
            default :;
            endcase
        end
        AMP_IOS_N: begin
            case(amp_seq)
            6'd8 : begin
                if (dac_done_wtm) begin
                    vos_in_n <= 1'b1;
                    vos_in_p <= 1'b1;
                    vos_o <= 1'b1;
                    vos_resin_n <= 1'b0;
                    vos_resin_p <= 1'b1;
                    rirokey <= 1'b1;
                    start_dac <= 1'b1;
                    chan_dac <= 3'd4;
                    vout <= BIAS_VE;
                    amp_seq   <= amp_seq + 1;
                end
                else begin
                    amp_seq <= amp_seq;
                end
            end
            6'd9 : begin
                if (dac_done_channel) begin
                    start_dac <= 1'b0;
                    delay_en <= 1'b1;
                    amp_seq <= amp_seq + 1;
                end
                else begin
                    start_dac <= 1'b0;
                    amp_seq <= amp_seq;
                end     
            end
            6'd10 : begin
                if (delay_done) begin
                    delay_en <= 1'b0;
                    start_adc <= 1'b1;
                    chan_adc <= 3'd0;
                    amp_seq <= amp_seq + 1;
                end
                else begin
                    amp_seq <= amp_seq;
                end
            end
            6'd11 : begin
                if ((sample_cnt < 10000) && adc_done) begin
                    start_adc <= 1'b1;
                    sample_cnt <= sample_cnt + 1;
                    sum <= sum + adc_data;
                    amp_seq <= amp_seq;
                end
                else if (sample_cnt == 10000) begin
                    start_adc <= 1'b0;
                    sample_cnt <= 0;
                    amp_seq <= amp_seq + 1;
                    sum <= sum / 10000;
                end
            end
            6'd12 : begin
                start_wr <= 1'b1;
                data_2_bram <= {4'b0001,sum[11:0]};
                amp_seq <= amp_seq + 1;
            end
            6'd13 : begin
                start_wr <= 1'b0;
                sum <= 32'd0;
                amp_seq <= amp_seq + 1;
                AMP_IOS_N_DONE <= 1'b1;
            end
            default:;
            endcase
        end
        AMP_IOS_P : begin
            case(amp_seq)
            6'd14 : begin
                if (dac_done_wtm) begin
                    vos_in_n <= 1'b1;
                    vos_in_p <= 1'b1;
                    vos_o <= 1'b1;
                    vos_resin_n <= 1'b1;
                    vos_resin_p <= 1'b0;
                    rirokey <= 1'b1;
                    start_dac <= 1'b1;
                    chan_dac <= 3'd4;
                    vout <= BIAS_VE;
                    amp_seq <= amp_seq + 1;
                end
                else begin
                    amp_seq <= amp_seq;
                end
            end
            6'd15 : begin
                if (dac_done_channel) begin
                    start_dac <= 1'b0;
                    delay_en <= 1'b1;
                    amp_seq <= amp_seq + 1;
                end
                else begin
                    start_dac <= 1'b0;
                    amp_seq <= amp_seq;
                end
            end
            6'd16 : begin
                if (delay_done) begin
                    delay_en <= 1'b0;
                    chan_adc <= 3'd0;
                    start_adc <= 1'b1;
                    amp_seq <= amp_seq + 1;
                end
                else begin
                    amp_seq <= amp_seq;
                end
            end
            6'd17 : begin
                if ((sample_cnt < 10000) && adc_done) begin
                    start_adc <= 1'b1;
                    sum <= sum + adc_data;
                    sample_cnt <= sample_cnt + 1;
                    amp_seq <= amp_seq;
                end
                else if (sample_cnt == 10000) begin
                    start_adc <= 1'b0;
                    sample_cnt <= 0;
                    amp_seq <= amp_seq + 1;
                    sum <= sum / 10000;
                end
            end
            6'd18 : begin
                start_wr <= 1'b1;
                data_2_bram <= {4'b0010,sum[11:0]};
                amp_seq <= amp_seq + 1;
            end
            6'd19 : begin
                start_wr <= 1'b0;
                sum <= 32'd0;
                vos_in_n <= 1'b0;
                vos_in_p <= 1'b0;
                vos_o <= 1'b0;
                vos_resin_n <= 1'b0;
                vos_resin_p <= 1'b0;
                AMP_IOS_P_DONE <= 1'b1;
                amp_seq <= amp_seq + 1;
            end
            default:;
            endcase
        end
        AMP_RIRO: begin
            case(amp_seq)
            // 500_000 : 23
            6'd20 : begin
                if (dac_done_wtm) begin
                    rirokey <= 1'b1;
                    ri_in_p <= 1'b1;
                    ri_o <= 1'b1;
                    ri_in_n <= 1'b1;
                    ampsinkey <= 1'b1;
                    user_lsb <= FREQ_23_LSB;
                    user_msb <= FREQ_23_MSB;
                    dds_start <= 1'b1;
                    amp_seq <= amp_seq + 1;
                    sample_flag <= 217;
                end
                else begin
                    amp_seq <= amp_seq;
                end 
            end
            6'd21 : begin
                if (dds_done) begin
                    dds_start <= 1'b0;
                    vout <= BIAS_VG_RIRO;
                    chan_dac <= 3'd0;
                    start_dac <= 1'b1;
                    amp_seq <= amp_seq + 1;
                end
                else begin
                    dds_start <= 1'b0;
                    amp_seq <= amp_seq;
                end
            end
            6'd22 : begin
                start_dac <= 1'b0;
                if (dac_done_channel) begin
                    amp_seq <= amp_seq + 1;
                end
                else begin
                    amp_seq <= amp_seq;
                end
            end
            6'd23 : begin
                chan_dac <= 3'd4;
                start_dac <= 1'b1;
                vout <= BIAS_VE_RIRO;
                amp_seq <= amp_seq + 1;
            end
            6'd24 : begin
                start_dac <= 1'b0;
                if (dac_done_channel) begin
                    dly_riro_en <= 1'b1;
                    amp_seq <= amp_seq + 1;
                end
                else begin
                    amp_seq <= amp_seq;
                end
            end
            6'd25 : begin
                if (dly_riro_done) begin
                    dly_riro_en <= 1'b0;
                    start_adc <= 1'b1;
                    chan_adc <= 3'd0;
                    amp_seq <= amp_seq + 1;
                end
                else amp_seq <= amp_seq;
            end
            6'd26 : begin
                if ((sample_cnt < 500) && (sample_flag == 217) && adc_done) begin
                    start_wr <= 1'b1;
                    data_2_bram <= {sample_cnt, 4'b0011, adc_data};
                    chan_adc <= 3'd0;
                    sample_cnt <= sample_cnt + 1;
                    amp_seq <= amp_seq;
                    sample_flag <= 0;
                end
                else if ((sample_cnt < 500) && (sample_flag < 217) && adc_done) begin
                    start_wr <= 1'b0;
                    sample_flag <= sample_flag + 1;
                end
                else if (sample_cnt < 500) begin
                    start_wr <= 1'b0;
                    amp_seq <= amp_seq;
                end
                else if (sample_cnt == 500) begin
                    start_wr <= 1'b0;
                    start_adc <= 1'b0;
                    sample_cnt <= 32'd0;
                    amp_seq <= amp_seq + 1;
                end
            end
            6'd27 : begin
                rirokey <= 1'b0;
                ri_in_p <= 1'b0;
                ri_o    <= 1'b0;
                ri_in_n <= 1'b0;
                AMP_RIRO_DONE <= 1'b1;
                amp_seq <= amp_seq + 1;
            end
            default : ;
            endcase
        end
        AMP_CMRR : begin
            case(amp_seq)
            6'd28 : begin
                if (dac_done_wtm) begin
                    cmrrkey <= 1'b1;
                    cmrr_in_p <= 1'b1;
                    cmrr_in_n <= 1'b1;
                    cmrr_o <= 1'b1;
                    ampsinkey <= 1'b1;
                    user_lsb <= FREQ_200_LSB;
                    user_msb <= FREQ_200_MSB;
                    dds_start <= 1'b1;
                    amp_seq <= amp_seq + 1;
                    sample_flag <= 24;
                end
                else amp_seq <= amp_seq; 
            end
            6'd29 : begin
                if (dds_done) begin
                    dds_start <= 1'b0;
                    vout <= BIAS_VG_CMRR;
                    chan_dac <= 3'd0;
                    start_dac <= 1'b1;
                    amp_seq <= amp_seq + 1;
                end
                else begin
                    dds_start <= 1'b0;
                    amp_seq <= amp_seq;
                end
            end
            6'd30 : begin
                if (dac_done_channel) begin
                    start_dac <= 1'b0;
                    amp_seq <= amp_seq + 1;
                end
                else begin
                    start_dac <= 1'b0;
                    amp_seq <= amp_seq;
                end
            end
            6'd31 : begin
                chan_dac <= 3'd4;
                start_dac <= 1'b1;
                vout <= BIAS_VE;
                amp_seq <= amp_seq + 1;
            end
            6'd32 : begin
                if (dac_done_channel) begin
                    start_dac <= 1'b0;
                    amp_seq <= amp_seq + 1;
                    delay_en <= 1'b1;
                end
                else begin
                    start_dac <= 1'b0; 
                    amp_seq <= amp_seq;
                end
            end
            6'd33 : begin
                if (delay_done) begin
                    delay_en <= 1'b0;
                    start_adc <= 1'b1;
                    chan_adc <= 3'd0;
                    amp_seq <= amp_seq + 1;
                end
                else amp_seq <= amp_seq;
            end
            6'd34 : begin 
                if ((sample_cnt < 500) && (sample_flag == 24) && adc_done) begin
                    start_wr <= 1'b1;
                    data_2_bram <= {4'b0100, adc_data};
                    start_adc <= 1'b1;
                    chan_adc <= 3'd0;
                    sample_cnt <= sample_cnt + 1;
                    amp_seq <= amp_seq;
                    sample_flag <= 0;
                end
                else if ((sample_cnt < 500) && (sample_flag < 24) && adc_done) begin
                    start_adc <= 1'b1;
                    start_wr <= 1'b0;
                    sample_flag <= sample_flag + 1;
                    amp_seq <= amp_seq;
                end
                else if (sample_cnt < 500) begin
                    start_adc <= 1'b1;
                    start_wr <= 1'b0;
                    amp_seq <= amp_seq;
                end
                else if (sample_cnt == 500) begin
                    start_wr <= 1'b0;
                    start_adc <= 1'b0;
                    sample_cnt <= 32'd0;
                    amp_seq <= amp_seq + 1;
                end
                
            end
            6'd35 : begin
                start_wr <= 1'b0;
                // VE OUTPUT VOLTAGE CLEAR
                vout <= VC_0;
                chan_dac <= 3'd4;
                start_dac <= 1'b1;
                amp_seq <= amp_seq + 1;
            end
            6'd36 : begin
                if (dac_done_channel) begin
                    start_dac <= 1'b0;
                    amp_seq <= amp_seq + 1;
                end
                else begin
                    start_dac <= 1'b0;
                    amp_seq <= amp_seq;
                end
            end
            6'd37 : begin
                vout <= VC_0;
                // VG OUTPUT VOLTAGE CLEAR
                chan_dac <= 3'd0;
                start_dac <= 1'b1;
                amp_seq <= amp_seq + 1;
            end
            6'd38 : begin
                if (dac_done_channel) begin
                    start_dac <= 1'b0;
                    amp_seq <= amp_seq + 1;
                end
                else begin
                    start_dac <= 1'b0;
                    amp_seq <= amp_seq;
                end
            end
            6'd39: begin
                cmrrkey <= 1'b0;
                cmrr_in_p <= 1'b0;
                cmrr_in_n <= 1'b0;
                cmrr_o <= 1'b0;
                ampsinkey <= 1'b0;
                amp_seq <= 6'd0;
                AMP_CMRR_DONE <= 1'b1;
                Unit_finish <= 8'b00000010;
            end
            default :;
            endcase
        end
        UNITADC : begin
            case(uadc_seq)
            4'd0 : begin
                Unit_finish <= 8'd0;
                if (dac_done_wtm) begin
                    adcsinkey <= 1'b1;
                    user_lsb <= FREQ_20K_LSB;
                    user_msb <= FREQ_20K_MSB;
                    dds_start <= 1'b1;
                    hdata_clr_en <= 1'b1;   
                    uadc_seq <= uadc_seq + 1;       
                end
                else begin 
                    uadc_seq <= uadc_seq;
                end
            end
            4'd1 : begin
                dds_start <= 1'b0;
                if (dds_done) begin
                    // VG由DAC的通道A给出
                    vout <= BIAS_VG_ADC;
                    chan_dac <= 3'd0;
                    start_dac <= 1'b1;
                    uadc_seq <= uadc_seq + 1;    
                end
                else begin
                    uadc_seq <= uadc_seq;
                end
            end
            4'd2 : begin
                start_dac <= 1'b0;
                if(dac_done_channel) begin
                    hdata_clr_en <= 1'b0;
                    uadc_seq <= uadc_seq + 1;
                end
                else begin
                    hdata_clr_en <= 1'b0; 
                    uadc_seq <= uadc_seq;
                end
            end
            4'd3 : begin
                // start DAC channel D
                chan_dac <= 3'd3;
                start_dac <= 1'b1;
                vout <= BIAS_VD;
                uadc_seq <= uadc_seq + 1;
            end
            4'd4 : begin
                start_dac <= 1'b0;
                if (dac_done_channel) begin
                    Unit_adc_key <= 1'b1;
                    delay_en <= 1'b1;
                    uadc_seq <= uadc_seq + 1; 
                end
                else begin
                    uadc_seq <= uadc_seq;
                end
            end
            4'd5 : begin
                if (delay_done) begin
                    delay_en <= 1'b0;
                    hdata_w_en <= 1'b1;
                    uadc_seq <= uadc_seq + 1;
                end
                else
                    uadc_seq <= uadc_seq;
            end
            4'd6 : begin
                if ((sample_cnt < 250) && (clk_trg == 49)) begin
                    start_wr <= 1'b1;
                    data_2_bram <= {4'b0000, uadc_data};
                    sample_cnt <= sample_cnt +1;
                    uadc_seq <= uadc_seq;
                end
                else if (sample_cnt < 250) begin
                    start_wr <= 1'b0;
                    uadc_seq <= uadc_seq;
                end
                else begin
                    start_wr <= 1'b0;
                    uadc_seq <= uadc_seq + 1;
                    sample_cnt <= 32'd0;
                end
            end
            4'd7 : begin
                if (hdata_w_finish) begin
                    hdata_w_en <= 1'b0;
                    Unit_adc_key <= 1'b0;
                    uadc_seq <= uadc_seq + 1;
                end
                else
                    uadc_seq <= uadc_seq;
            end
            4'd8 : begin
                // VD OUTPUT VOLTAGE CLEAR
                vout <= VC_0;
                chan_dac <= 3'd3;
                start_dac <= 1'b1;
                uadc_seq <= uadc_seq + 1;
            end
            4'd9 : begin
                start_dac <= 1'b0;
                if (dac_done_channel) begin
                    uadc_seq <= uadc_seq + 1;
                end
                else begin
                    uadc_seq <= uadc_seq;
                end
            end
            4'd10 : begin
                vout <= VC_0;
                // VG OUTPUT VOLTAGE CLEAR
                chan_dac <= 3'd0;
                start_dac <= 1'b1;
                uadc_seq <= uadc_seq + 1;
            end
            4'd11 : begin
                start_dac <= 1'b0;
                if (dac_done_channel) begin
                    uadc_seq <= uadc_seq + 1;
                end
                else begin
                    uadc_seq <= uadc_seq;
                end
            end
            4'd12 : begin
                UNITADC_DONE <= 1'b1;
                adcsinkey <= 1'b0;
                Unit_finish <= 8'b00000100;
            end
            endcase
        end
        UNITUART : begin
            case(uu_seq)
            3'd0 : begin
                Unit_finish <= 8'd0;
                hdata_clr_en <= 1'b1;
                Unit_uart_key <= 1'b1;
                delay_en <= 1'b1;
                uu_seq <= uu_seq + 1;
            end
            3'd1 : begin
                if (delay_done) begin
                    delay_en <= 1'b0;
                    hdata_clr_en <= 1'b0;
                    uu_seq <= uu_seq + 1;
                end
                else
                    uu_seq <= uu_seq;
            end
            3'd2 : begin
                rs422_en <= 1'b1;
                uu_seq <= uu_seq + 1;
            end
            3'd3 : begin
                if (hdata_w_finish && rs422_finish) begin
                    rs422_en <= 1'b0;
                    hdata_w_en <= 1'b0;
                    uu_seq <= uu_seq + 1;
                    digital_err_cnt <= rs422_err_count;
                end
                else
                    uu_seq <= uu_seq;
            end
            3'd4 : begin
                Unit_finish <= 8'b00001000;
                Unit_uart_key <= 1'b0;
                UNITUART_DONE <= 1'b1;
            end
            endcase
        end
        UNITLOGIC : begin
            case(ulo_seq)
            3'd0 : begin
                Unit_finish <= 8'd0;
                hdata_clr_en <= 1'b1;
                Unit_logic_key <= 1'b1;
                delay_en <= 1'b1;
                ulo_seq <= ulo_seq + 1;
            end
            3'd1 : begin
                if (delay_done) begin
                    delay_en <= 1'b0;
                    hdata_clr_en <= 1'b0;
                    ulo_seq <= ulo_seq + 1;
                end
                else
                    ulo_seq <= ulo_seq;
            end
            3'd2 : begin
                not_en <= 1'b1;
                ulo_seq <= ulo_seq + 1;
            end
            3'd3 : begin
                if (hdata_w_finish && not_finish) begin
                    not_en <= 1'b0;
                    hdata_w_en <= 1'b0;
                    ulo_seq <= ulo_seq + 1;
                    digital_err_cnt <= not_err_count;
                end
                else
                    ulo_seq <= ulo_seq;
            end
            3'd4 : begin
                Unit_finish <= 8'b00010000;
                Unit_logic_key <= 1'b0;
                UNITLOGIC_DONE <= 1'b1;
            end
            endcase
        end
        UNITXO : begin
            case(uxo_seq)
            3'd0 : begin
                Unit_finish <= 8'd0;
                hdata_clr_en <= 1'b1;
                Unit_xo_key <= 1'b1;
                delay_en <= 1'b1;
                uxo_seq <= uxo_seq + 1;
            end
            3'd1 : begin
                if (delay_done) begin
                    delay_en <= 1'b0;
                    hdata_clr_en <= 1'b0;
                    uxo_seq <= uxo_seq + 1;
                end
                else
                    uxo_seq <= uxo_seq;
            end
            3'd2 : begin
                hdata_w_en <= 1'b1;
                uxo_seq <= uxo_seq + 1;
            end
            3'd3 : begin
                if (hdata_w_finish) begin
                    hdata_w_en <= 1'b0;
                    uxo_seq <= uxo_seq + 1;
                end
                else
                    uxo_seq <= uxo_seq;
            end
            3'd4 : begin
                Unit_finish <= 8'b00100000;
                Unit_xo_key <= 1'b0;
                UNITXO_DONE <= 1'b1;
            end
            endcase
        end
        UNITSRAM : begin
            case(usram_seq)   
            3'd0 : begin
                Unit_finish <= 8'd0;
                hdata_clr_en <= 1'b1;
                Unit_sram_key <= 1'b1;
                delay_en <= 1'b1;
                usram_seq <= usram_seq + 1;
            end
            3'd1 : begin
                if (delay_done) begin
                    delay_en <= 1'b0;
                    hdata_clr_en <= 1'b0;
                    usram_seq <= usram_seq + 1;
                end
                else
                    usram_seq <= usram_seq;
            end
            3'd2 : begin
                sram_read_en <= 1'b1;
                usram_seq <= usram_seq + 1;
            end
            3'd3 : begin
                hdata_w_en <= 1'b1;
                usram_seq <= usram_seq + 1;
            end
            3'd4 : begin
                if (sram_read_finish) begin
                    sram_read_en <= 1'b0;
                    usram_seq <= usram_seq + 1;
                    digital_err_cnt <= usram_err_count;
                end
                else
                    usram_seq <= usram_seq;
            end
            3'd5 : begin
                if (hdata_w_finish) begin
                    hdata_w_en <= 1'b0;
                    usram_seq <= usram_seq + 1;
                end
                else
                    usram_seq <= usram_seq;
            end
            3'd6 : begin
                Unit_finish <= 8'b01000000;
                Unit_sram_key <= 1'b0;
                UNITSRAM_DONE <= 1'b1;
            end
            endcase
        end
        UNITFPGA : begin
            case(ufpga_seq)   
            3'd0 : begin
                Unit_finish <= 8'd0;
                hdata_clr_en <= 1'b1;
                Unit_fpga_key <= 1'b1;
                delay_en <= 1'b1;
                ufpga_seq <= ufpga_seq + 1;
            end
            3'd1 : begin
                if (delay_done) begin
                    delay_en <= 1'b0;
                    hdata_clr_en <= 1'b0;
                    ufpga_seq <= ufpga_seq + 1;
                end
                else
                    ufpga_seq <= ufpga_seq;
            end
            3'd2 : begin
                start_fpga <= 1'b1;
                ufpga_seq <= ufpga_seq + 1;
            end
            3'd3 : begin
                if (spi_recvd) begin
                    start_fpga <= 1'b0;
                    spi_recv_ack <= 1'b1;
                    ufpga_seq <= ufpga_seq + 1;
                    hdata_w_en <= 1'b1;
                    digital_err_cnt <= spi_recv_data;
                end
                else
                    ufpga_seq <= ufpga_seq;
            end
            3'd4 : begin
                if (hdata_w_finish) begin
                    hdata_w_en <= 1'b0;
                    ufpga_seq <= ufpga_seq + 1;
                end
                else
                    ufpga_seq <= ufpga_seq;
            end
            3'd5 : begin
                Unit_finish <= 8'b10000000;
                Unit_fpga_key <= 1'b0;
                UNITFPGA_DONE <= 1'b1;
            end
            endcase
        end
        default :;
        endcase
end
// ================================= Three-Block FSM ============================================

// ================================= BRAM WRITE CONTROL =========================================
reg flow_bram;
assign clkb = sys_clk;
assign rstb = 1'b0;
assign clr_flag = dcdc_flag || amp_flag || uadc_flag;
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        flow_bram <= 1'b0;
        dinb <= 32'd0;
        addrb <= 32'd0;
        web <= 4'h0;
        enb <= 1'b0;
    end
    else if (clr_flag) begin
        addrb <= 32'd0;
        flow_bram <= 1'b0;
        dinb <= 32'd0;
    end
    else
        case(flow_bram)
        1'b0 : begin
            if(start_wr) begin
                web <= 4'hf;
                enb <= 1'b1;
                flow_bram <= flow_bram + 1'b1;
                dinb <= data_2_bram;
                addrb <= addrb;
            end
            else begin
                web <= 4'h0;
                enb <= 1'b0;
                flow_bram <= flow_bram;
                dinb <= 32'd0;
                addrb <= addrb;
            end
        end
        1'b1 : begin
            web <= 4'h0;
            enb <= 1'b0;
            flow_bram <= 1'b0;
            addrb <= addrb + 32'd4;
        end
        default :;
        endcase
end
// ================================= BRAM WRITE CONTROL =========================================

// ================================= PS TRACKER SETTTING ========================================
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        ps_tracker <= 16'd0;
    end
    else
        case(cstate)
        IDLE : begin
            ps_tracker <= 16'd0;
        end
        DCDC_ICC : begin
            ps_tracker <= {6'b000100, dcdc_seq};
        end
        DCDC_VOUT : begin
            ps_tracker <= {6'b000100, dcdc_seq};
        end
        DCDC_LINE : begin
            ps_tracker <= {6'b000100, dcdc_seq};
        end
        DCDC_LOAD : begin
            ps_tracker <= {6'b000100, dcdc_seq};
        end
        AMP_VOS : begin
            ps_tracker <= {6'b001000, amp_seq};
        end
        AMP_IOS_N : begin
            ps_tracker <= {6'b001000, amp_seq};
        end
        AMP_IOS_P : begin
            ps_tracker <= {6'b001000, amp_seq};
        end
        AMP_RIRO : begin
            ps_tracker <= {6'b001000, amp_seq};
        end
        AMP_CMRR : begin
            ps_tracker <= {6'b001000, amp_seq};
        end
        UNITADC : begin
            ps_tracker <= {8'b00110000, uadc_seq};
        end
        UNITUART : begin
            ps_tracker <= {9'b010000000, uu_seq};
        end
        UNITLOGIC : begin
            ps_tracker <= {9'b010100000, ulo_seq};
        end
        UNITXO : begin
            ps_tracker <= {9'b011000000, uxo_seq};
        end
        UNITSRAM : begin
            ps_tracker <= {9'b011100000, usram_seq};
        end
        UNITFPGA : begin
            ps_tracker <= {9'b100000000, ufpga_seq};
        end
        endcase
end
// ================================= PS TRACKER SETTTING ========================================

// ================================= MODULE INSTANTIATION =======================================

myadctop  myadctop_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .start_en(start_adc),
    .channel(chan_adc),
    .ADC_DOUT(ADC_DOUT),
    .sclk(ADC_SCLK),
    .cs_n(ADC_CSN),
    .din(ADC_DIN),
    .conv_done(adc_done),
    .ADC_Data(adc_data)
);
ddstop  ddstop_inst (
    .clk(sys_clk),
    .start_dds(dds_start),
    .freq_lsb(user_lsb),
    .freq_msb(user_msb),
    .dds_done(dds_done),
    .FSYNC(FSYNC_DDS),
    .SCLK_DDS(SCLK_DDS),
    .SDATA(SDATA_DDS)
);
dactop  dactop_inst (
    .clk(sys_clk),
    .start(start_dac),
    .sclk_div(sclk_div),
    .channel(chan_dac),
    .dac(vout),
    .set_channel(dac_done_channel),
    .set_wtm(dac_done_wtm),
    .SYNC(SYNC_DAC),
    .SCLK(SCLK_DAC),
    .DIN(DIN_DAC)
);
notgate  notgate_inst (
    .sys_clk(clk_sram),
    .sys_rst_n(sys_rst_n),
    .NOT_START(not_en),
    .NOT_FINISH(not_finish),
    .NOT_IN(ulo_rxd),
    .NOT_OUT(ulo_txd),
    .NOT_ERROR(not_err_count)
);
rs422  rs422_inst (
    .sys_clk(clk_sram),
    .sys_rst_n(sys_rst_n),
    .RS422_START(rs422_en),
    .RS422_FINISH(rs422_finish),
    .RS422_IN(uu_rxd),
    .RS422_DI(RS422_DI),
    .RS422_ERROR(rs422_err_count)
  );
sramtwo  sram_inst (
    .SRAM_sys_clk(clk_sram),
    .SRAM_rst_n(sys_rst_n),
    .SRAM_start(sram_read_en),
    .SRAM_error(usram_err_count),
    .SRAM_end(sram_read_finish),
    .SRAM_A(SRAM_A),
    .SRAM_DB(SRAM_DB),
    .SRAM_WE_N(SRAM_WE_N),
    .SRAM_OE_N(SRAM_OE_N),
    .SRAM_UB_N(SRAM_UB_N),
    .SRAM_LB_N(SRAM_LB_N),
    .SRAM_CE_N(SRAM_CE_N)
);
spi_i  spi_i_inst (
    .sys_clk(sys_clk),
    .n_rst(sys_rst_n),
    .SCK(SCK),
    .SDI(SDI),
    .SCS(SCS),
    .revData_o(spi_recv_data),
    .revDone_o(spi_recvd),
    .readDone_i(spi_recv_ack)
);
ad9434top  ad9434top_inst (
    .sys_clk(sys_clk),
    .sys_w_en(hdata_w_en),
    .sys_clr_en(hdata_clr_en),
    .sys_rst_n(sys_rst_n),
    .SEL1(SEL1),
    .SEL0(SEL0),
    .EN0(EN0),
    .EN1(EN1),
    .AD9434_SDIO(AD9434_SDIO),
    .AD9434_SCLK(AD9434_SCLK),
    .AD9434_CSB(AD9434_CSB),
    .AD9434_PWDN(AD9434_PWDN),
    .CLK_9434_P(CLK_9434_P),
    .CLK_9434_N(CLK_9434_N),
    .adc_data_in_p(adc_data_in_p),
    .adc_data_in_n(adc_data_in_n),
    .or_p(or_p),
    .or_n(or_n),
    .dco_p(dco_p),
    .dco_n(dco_n),
    .BRAM_PORTB_addrb(BRAM_PORTB_addrb),
    .BRAM_PORTB_clkb(BRAM_PORTB_clkb),
    .BRAM_PORTB_dinb(BRAM_PORTB_dinb),
    .BRAM_PORTB_enb(BRAM_PORTB_enb),
    .BRAM_PORTB_doutb(BRAM_PORTB_doutb),
    .BRAM_PORTB_rstb(BRAM_PORTB_rstb),
    .BRAM_PORTB_web(BRAM_PORTB_web),
    .hdata_finish(hdata_w_finish)
);
// ================================= MODULE INSTANTIATION =======================================
endmodule