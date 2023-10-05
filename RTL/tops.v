`timescale 1ns/1ps

/*
* ENGINEER : Wu Ziyan
* DATETIME : 2023/10/03 UTC-8 21:42
* REVISIONS : AMP DAC CLEAR OPERATION ADDED, ADC WORKFLOWS ADDED
*             DCDC : SAMPLE_CNT 50 -> 51
              BRAM : added clr_flag (logic or) to judge if the addrb needed to be set to 0
              ADC : 1M SAMPLE REVISION (CLK_TRG)
* CONTENT : Unit modules test workflow (DCDC, AMP, ADC)
* TODO : PARAMETERS OPTIMIZATION 
*/

module tops #(
    parameter VIN_10 = 12'd1885,
    parameter VIN_8 = 12'd1310,
    parameter VC_66 = 12'd1081,
    parameter VC_0 = 12'd0,
    parameter VC_5 = 12'd820,
    // parameter DELAY_CNT = 32'd2,
    parameter DELAY_CNT = 32'd10_000_000,
    parameter BIAS_VE = 12'd2050,
    parameter BIAS_VE_RIRO = 12'd1640,
    parameter BIAS_VG_RIRO = 12'd328,
    parameter BIAS_VG_CMRR = 12'd2704,
    parameter BIAS_VG_ADC = 12'd2458,
    parameter BIAS_VD = 12'd1640,
    parameter FREQ_23_LSB = 16'h40f7,
    parameter FREQ_23_MSB = 16'h4000,
    parameter FREQ_200_LSB = 16'h4864,
    parameter FREQ_200_MSB = 16'h4000,
    parameter FREQ_20K_LSB = 16'h46dc,
    parameter FREQ_20K_MSB = 16'h400d,
    parameter ADC_20K = 25_000_000 / 320_000,
    // parameter ADC_20K = 10,
    parameter ADC_23 = 25_000_000 / 36800
    // parameter ADC_23 = 10 
) (
    input               sys_clk,
    input               sys_rst_n,

    input               start_dcdc,
    input               start_amp,
    input               start_uadc,

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
    
    // output              adc_en,
    // output              one_conv_done,
    
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
    output reg          Unit_dcdc_finish,
    output reg          Unit_amp_finish,
    output reg          Unit_adc_finish,
    output reg  [15:0]   ps_tracker        
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
reg [8:0] sample_cnt;

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
// ========================= CONTROLLING SIGNALS FOR RELAY DAC ADC DDS BRAM ===================

// ================================= 1M CLOCKING FOR AD9280 ===================================
reg [4:0] clkcnt;
reg [5:0] clk_trg;
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        clk_trg <= 6'd0; 
    end
    else if (clk_trg < 49) begin
        clk_trg <= clk_trg + 1;
    end
    else
        clk_trg <= 6'd0;
end
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        clkcnt <= 5'd0;
        uadc_clk <= 1'b0;
        
    end
    else if (clkcnt < 24) begin
        clkcnt <= clkcnt + 1;
    end
    else begin
        clkcnt <= 5'd0;
        uadc_clk <= ~uadc_clk;
    end
end
// ================================= 1M CLOCKING FOR AD9280 ===================================

// ================================= 25M CLOCKING FOR AD9833 ==================================
clk_wiz_0 instance_name
   (
    // Clock out ports
    .clk_out1(CLK_25M),     // output clk_out1
   // Clock in ports
    .clk_in1(sys_clk));      // input clk_in1
// ================================= 25M CLOCKING FOR AD9833 ==================================

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

// Unit ADC STARTUP TRIGGER
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

// CHILD STATE SEQ TRACKER (FOR PS TRACKER)
reg [5:0] dcdc_seq;
reg [5:0] amp_seq;
reg [3:0] uadc_seq;
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
        default : begin 
            nstate = IDLE;
        end
    endcase
end
// The Third block of a 3-block state machine
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
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
        sclk_div <= 32'd8;
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
        sample_cnt <= 9'd0;
        dcdc_seq <= 6'd0;
        amp_seq <= 6'd0;
        uadc_seq <= 4'd0;
        Unit_dcdc_finish <= 1'b0;
        Unit_amp_finish <= 1'b0;
        Unit_adc_finish <= 1'b0;
    end
    else
        case (cstate)
        IDLE: begin
            vf_key <= 1'b0;
            ven_key <= 1'b0;
            sclk_div <= 32'd8;
            DCDC_VOUT_DONE <= 1'b0;
            DCDC_ICC_DONE <= 1'b0;
            DCDC_LOAD_DONE <= 1'b0;
            DCDC_LINE_DONE <= 1'b0;
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
            sample_cnt  <= 9'd0;
            dcdc_seq <= 6'd0;
            amp_seq <= 6'd0;
            uadc_seq <= 4'd0;
            AMP_VOS_DONE   <= 1'b0;
            AMP_IOS_P_DONE <= 1'b0;
            AMP_IOS_N_DONE <= 1'b0;
            AMP_RIRO_DONE  <= 1'b0;
            AMP_CMRR_DONE  <= 1'b0;
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
            dds_start <= 1'b0;
        end
        DCDC_ICC: begin
            case(dcdc_seq)
            6'd0 : begin
                if (dac_done_wtm) begin
                    Unit_dcdc_finish <= 1'b0;
                    ven_key <= 1'b0;
                    vout <= VC_0;
                    chan_dac <= 3'd2;
                    start_dac <= 1'b1;
                    dcdc_seq <= dcdc_seq + 1;
                end
                else begin
                    Unit_dcdc_finish <= 1'b0;
                    dcdc_seq <= dcdc_seq;
                end
            end
            6'd1 : begin
                if (dac_done_channel) begin
                    start_dac <= 1'b0;
                    dcdc_seq <= dcdc_seq + 1;
                end
                else begin
                    start_dac <= 1'b0;
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
                if (dac_done_channel) begin
                    start_dac <= 1'b0;
                    delay_en <= 1'b1;
                    dcdc_seq <= dcdc_seq + 1;
                end
                else begin
                    start_dac <= 1'b0;
                    dcdc_seq <= dcdc_seq;
                end
            end
            6'd4 : begin
                if (delay_done) begin
                    delay_en <= 1'b0;
                    dcdc_seq <= dcdc_seq + 1;
                end
                else begin
                    dcdc_seq <= dcdc_seq;
                end
            end
            6'd5 : begin
                if (sample_cnt < 9'd50) begin
                    start_adc <= 1'b1;
                    chan_adc <= 3'd3;
                    user_freq <= ADC_20K;
                    sample_cnt <= sample_cnt + 1;
                    dcdc_seq <= dcdc_seq + 1;
                end
                else begin
                    sample_cnt   <= 9'd0;
                    dcdc_seq <= dcdc_seq + 2;
                    sum <= sum / 50;
                end
            end
            6'd6 : begin
                if (adc_done) begin
                    start_adc <= 1'b0;
                    sum <= sum + adc_data;
                    dcdc_seq <= dcdc_seq - 1;
                end
                else begin
                    start_adc <= 1'b0;
                    dcdc_seq <= dcdc_seq;
                end
            end
            6'd7 : begin
                start_wr <= 1'b1;
                data_2_bram <= {4'b1000,sum[11:0]};
                dcdc_seq <= dcdc_seq + 1;
            end
            6'd8 : begin
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
            6'd9 : begin
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
            6'd10 : begin
                
                if (dac_done_channel) begin
                    start_dac <= 1'b0;
                    dcdc_seq <= dcdc_seq + 1;
                end
                else begin
                    start_dac <= 1'b0;
                    dcdc_seq <= dcdc_seq;
                end
            end
            6'd11 : begin
                chan_dac <= 3'd2;
                vout <= VC_66; 
                start_dac <= 1'b1;
                dcdc_seq <= dcdc_seq + 1;
            end
            6'd12 : begin
                if (dac_done_channel) begin
                    start_dac <= 1'b0;
                    dcdc_seq <= dcdc_seq + 1;
                end
                else begin
                    start_dac <= 1'b0;
                    dcdc_seq <= dcdc_seq;
                end
            end
            6'd13 : begin
                delay_en <= 1'b1;
                dcdc_seq <= dcdc_seq + 1;
            end
            6'd14 : begin
                if (delay_done) begin
                    delay_en <= 1'b0;
                    dcdc_seq <= dcdc_seq + 1;
                end
                else begin
                    dcdc_seq <= dcdc_seq;
                end
            end
            6'd15 : begin
                if (sample_cnt < 9'd100) begin
                    start_adc <= 1'b1;
                    user_freq <= ADC_20K;
                    chan_adc <= 3'd1;
                    sample_cnt <= sample_cnt + 1;
                    dcdc_seq <= dcdc_seq + 1;
                end
                else begin
                    sample_cnt <= 9'd0;
                    dcdc_seq <= dcdc_seq + 2;
                    sum <= sum / 50;
                end
            end
            6'd16 : begin
                if (adc_done && (sample_cnt>=9'd51)) begin
                    start_adc <= 1'b0;
                    sum <= sum + adc_data;
                    dcdc_seq <= dcdc_seq - 1;
                end
                else if(adc_done && (sample_cnt<9'd51)) begin
                    start_adc <= 1'b0;
                    dcdc_seq <= dcdc_seq - 1;
                end
                else begin
                    start_adc <= 1'b0;
                    dcdc_seq <= dcdc_seq;
                end
            end
            6'd17 : begin
                start_wr <= 1'b1;
                data_2_bram <= {4'b0010,sum[11:0]};
                dcdc_seq <= dcdc_seq + 1;
            end
            6'd18 : begin
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
            6'd19 : begin
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
            6'd20 : begin
                if (dac_done_channel) begin
                    start_dac   <= 1'b0;
                    dcdc_seq <= dcdc_seq + 1;
                end
                else begin
                    start_dac   <= 1'b0;
                    dcdc_seq <= dcdc_seq;
                end
            end
            6'd21: begin
                chan_dac     <= 3'd1;
                vout         <= VIN_8; 
                start_dac    <= 1'b1;
                dcdc_seq <= dcdc_seq + 1;
            end
            6'd22 : begin
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
            6'd23 : begin
                if (delay_done) begin
                    delay_en <= 1'b0;
                    dcdc_seq <= dcdc_seq + 1;
                end
                else begin
                    dcdc_seq <= dcdc_seq;
                end
            end
            6'd24 : begin
                if (sample_cnt < 9'd100) begin
                    start_adc <= 1'b1;
                    chan_adc <= 3'd1;
                    user_freq <= ADC_20K;
                    sample_cnt <= sample_cnt + 1;
                    dcdc_seq <= dcdc_seq + 1;
                end
                else begin
                    sample_cnt <= 9'd0;
                    dcdc_seq <= dcdc_seq + 2;
                    sum <= sum / 50;
                end
            end
            6'd25 : begin
                if (adc_done && (sample_cnt>=9'd51)) begin
                    start_adc <= 1'b0;
                    sum  <= sum + adc_data;
                    dcdc_seq <= dcdc_seq - 6'd1;
                end
                else if(adc_done && (sample_cnt<9'd51)) begin
                    start_adc <= 1'b0;
                    dcdc_seq <= dcdc_seq - 1;
                end
                else begin
                    start_adc <= 1'b0;
                    dcdc_seq <= dcdc_seq;
                end
            end
            6'd26 : begin
                start_wr <= 1'b1;
                data_2_bram <= {4'b0111,sum[11:0]};
                dcdc_seq <= dcdc_seq + 1;
            end
            6'd27 : begin
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
            6'd28 : begin
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
            6'd29 : begin
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
            6'd30 : begin
                if (delay_done) begin
                    delay_en <= 1'b0;
                    dcdc_seq <= dcdc_seq + 1;
                end
                else begin
                    dcdc_seq <= dcdc_seq;
                end
            end
            6'd31 : begin
                if (sample_cnt < 9'd100) begin
                    start_adc <= 1'b1;
                    chan_adc <= 3'd1;
                    user_freq <= ADC_20K;
                    sample_cnt <= sample_cnt + 1;
                    dcdc_seq <= dcdc_seq + 1;
                end
                else begin
                    sample_cnt <= 9'd0;
                    dcdc_seq <= dcdc_seq + 2;
                    sum <= sum / 50;
                end
            end
            6'd32 : begin
                if (adc_done && (sample_cnt>=9'd51)) begin
                    start_adc <= 1'b0;
                    sum <= sum + adc_data;
                    dcdc_seq <= dcdc_seq - 1;
                end
                else if(adc_done && (sample_cnt<9'd51)) begin
                    start_adc <= 1'b0;
                    dcdc_seq <= dcdc_seq - 1;
                end
                else begin
                    start_adc <= 1'b0;
                    dcdc_seq <= dcdc_seq;
                end
            end
            6'd33 : begin
                start_wr <= 1'b1;
                data_2_bram <= {4'b0100,sum[11:0]};
                dcdc_seq <= dcdc_seq + 1;
            end
            6'd34 : begin
                start_wr <= 1'b0;
                vout <= VC_0;
                chan_dac <= 3'd2;
                start_dac <= 1'b1;
                dcdc_seq <= dcdc_seq + 1;
            end
            6'd35 : begin
                if (dac_done_channel) begin
                    start_dac <= 1'b0;
                    dcdc_seq <= dcdc_seq + 1;
                end
                else begin
                    start_dac <= 1'b0;
                    dcdc_seq <= dcdc_seq;
                end
            end
            6'd36 : begin
                vout <= VC_0;
                chan_dac <= 3'd1;
                start_dac <= 1'b1;
                dcdc_seq <= dcdc_seq + 1;
            end
            6'd37 : begin
                if (dac_done_channel) begin
                    start_dac <= 1'b0;
                    dcdc_seq <= dcdc_seq + 1;
                end
                else begin
                    start_dac <= 1'b0;
                    dcdc_seq <= dcdc_seq;
                end
            end
            6'd38 : begin
                ven_key  <= 1'b0;
                vf_key   <= 1'b0;
                start_wr <= 1'b0;
                sum      <= 32'd0;
                // pl_done  <= 1'b1;
                DCDC_LOAD_DONE <= 1'b1;
                Unit_dcdc_finish <= 1'b1;
                dcdc_seq <= 6'd0;
            end
            default :;
            endcase
        end
        AMP_VOS: begin
            case(amp_seq)
            6'd0 : begin
                if (dac_done_wtm) begin
                    Unit_amp_finish <= 1'b0;
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
                    Unit_amp_finish <= 1'b0;
                    amp_seq <= amp_seq;
                end
            end
            6'd1 : begin
                if (dac_done_channel) begin
                    start_dac <= 1'b0;
                    amp_seq <= amp_seq + 1;
                end
                else begin
                    start_dac <= 1'b0;
                    amp_seq <= amp_seq;
                end
            end
            6'd2 : begin
                amp_seq <= amp_seq + 1;
            end
            6'd3 : begin
                delay_en <= 1'b1;
                amp_seq <= amp_seq + 1;
            end
            6'd4 : begin
                if (delay_done) begin
                    delay_en <= 1'b0;
                    amp_seq <= amp_seq + 1;
                end
                else
                    amp_seq <= amp_seq;
            end
            6'd5 : begin
                amp_seq <= amp_seq + 1;
            end
            6'd6 : begin
                if (sample_cnt < 9'd50) begin
                    start_adc <= 1'b1;
                    user_freq <= ADC_20K;
                    chan_adc <= 3'd0;
                    sample_cnt <= sample_cnt + 9'd1;
                    amp_seq <= amp_seq + 1;
                end
                else begin
                    sample_cnt <= 9'd0;
                    amp_seq <= amp_seq + 2;
                    sum <= sum / 50;
                end
            end
            6'd7 : begin
                if (adc_done) begin
                    start_adc    <= 1'b0;
                    sum          <= sum + adc_data;
                    amp_seq <= amp_seq - 1;
                end
                else begin
                    start_adc <= 1'b0;
                    amp_seq <= amp_seq;
                end
            end
            6'd8 : begin
                start_wr <= 1'b1;
                data_2_bram <= {4'b0000,sum[11:0]};
                amp_seq <= amp_seq + 1;
            end
            6'd9 : begin
                start_wr     <= 1'b0;
                sum          <= 32'd0;
                amp_seq <= amp_seq + 1;
            end
            6'd10 : begin
                AMP_VOS_DONE <= 1'b1;
                voskey       <= 1'b0;
                amp_seq <= amp_seq + 1;
            end
            default :;
            endcase
        end
        AMP_IOS_N: begin
            case(amp_seq)
            6'd11 : begin
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
            6'd12 : begin
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
            6'd13 : begin
                if (delay_done) begin
                    delay_en <= 1'b0;
                    amp_seq <= amp_seq + 1;
                end
                else begin
                    amp_seq <= amp_seq;
                end
            end
            6'd14 : begin
                if (sample_cnt < 9'd50) begin
                    start_adc <= 1'b1;
                    user_freq <= ADC_20K;
                    chan_adc <= 3'd0;
                    sample_cnt <= sample_cnt + 1;
                    amp_seq <= amp_seq + 1;
                end
                else begin
                    sample_cnt <= 9'd0;
                    amp_seq <= amp_seq + 2;
                    sum <= sum / 50;
                end
            end
            6'd15 : begin
                if (adc_done) begin
                    start_adc <= 1'b0;
                    sum <= sum + adc_data;
                    amp_seq <= amp_seq - 1;
                end
                else begin
                    start_adc <= 1'b0;
                    amp_seq <= amp_seq;
                end
            end
            6'd16 : begin
                start_wr <= 1'b1;
                data_2_bram <= {4'b0001,sum[11:0]};
                amp_seq <= amp_seq + 1;
            end
            6'd17 : begin
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
            6'd18 : begin
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
            6'd19 : begin
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
            6'd20 : begin
                if (delay_done) begin
                    delay_en <= 1'b0;
                    amp_seq <= amp_seq + 1;
                end
                else begin
                    amp_seq <= amp_seq;
                end
            end
            6'd21 : begin
                if (sample_cnt < 9'd50) begin
                    start_adc <= 1'b1;
                    user_freq <= ADC_20K;
                    chan_adc <= 3'd0;
                    sample_cnt <= sample_cnt + 1;
                    amp_seq <= amp_seq + 1;
                end
                else begin
                    sample_cnt <= 9'd0;
                    amp_seq <= amp_seq + 2;
                    sum <= sum / 50;
                end
            end
            6'd22 : begin
                if (adc_done) begin
                    start_adc <= 1'b0;
                    sum <= sum + adc_data;
                    amp_seq <= amp_seq - 1;
                end
                else begin
                    start_adc <= 1'b0;
                    amp_seq <= amp_seq;
                end
            end
            6'd23 : begin
                start_wr <= 1'b1;
                data_2_bram <= {4'b0010,sum[11:0]};
                amp_seq <= amp_seq + 1;
            end
            6'd24 : begin
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
            6'd25 : begin
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
                end
                else begin
                    amp_seq <= amp_seq;
                end 
            end
            6'd26 : begin
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
            6'd27 : begin
                if (dac_done_channel) begin
                    start_dac <= 1'b0;
                    amp_seq <= amp_seq + 1;
                end
                else begin
                    start_dac <= 1'b0;
                    amp_seq <= amp_seq;
                end
            end
            6'd28 : begin
                chan_dac <= 3'd4;
                start_dac <= 1'b1;
                vout <= BIAS_VE_RIRO;
                amp_seq <= amp_seq + 1;
            end
            6'd29 : begin
                if (dac_done_channel) begin
                    start_dac <= 1'b0;
                    delay_en <= 1'b1;
                    amp_seq <= amp_seq + 1;
                end
                else begin
                    amp_seq <= amp_seq;
                    start_dac <= 1'b0;
                end
            end
            6'd30 : begin
                if (delay_done) begin
                    delay_en <= 1'b0;
                    amp_seq <= amp_seq + 1;
                end
                else amp_seq <= amp_seq;
            end
            6'd31 : begin
                if (sample_cnt < 9'd500) begin
                    start_wr <= 1'b0;
                    start_adc <= 1'b1;
                    user_freq <= ADC_23;
                    chan_adc <= 3'd0;
                    sample_cnt <= sample_cnt + 1;
                    amp_seq <= amp_seq + 1;
                end
                else begin
                    start_wr <= 1'b0;
                    sample_cnt <= 9'd0;
                    amp_seq <= amp_seq + 2;
                end
            end
            6'd32 : begin
                if (adc_done) begin
                    start_wr <= 1'b1;
                    start_adc <= 1'b0;
                    data_2_bram <= {4'b0011, adc_data};
                    amp_seq <= amp_seq - 1;
                end
                else begin
                    start_adc <= 1'b0;
                    amp_seq <= amp_seq;
                end
            end
            6'd33 : begin
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
            6'd34 : begin
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
                end
                else amp_seq <= amp_seq; 
            end
            6'd35 : begin
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
                chan_dac <= 3'd4;
                start_dac <= 1'b1;
                vout <= BIAS_VE;
                amp_seq <= amp_seq + 1;
            end
            6'd38 : begin
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
            6'd39 : begin
                if (delay_done) begin
                    delay_en <= 1'b0;
                    amp_seq <= amp_seq + 1;
                end
                else amp_seq <= amp_seq;
            end
            6'd40 : begin 
                if (sample_cnt < 9'd500) begin
                    start_wr <= 1'b0;
                    start_adc <= 1'b1;
                    user_freq <= ADC_20K;
                    chan_adc <= 3'd0;
                    sample_cnt <= sample_cnt + 1;
                    amp_seq <= amp_seq + 1;
                end
                else begin
                    start_wr <= 1'b0;
                    sample_cnt <= 9'd0;
                    amp_seq <= amp_seq + 2;
                end
            end
            6'd41 : begin
                if (adc_done) begin
                    start_wr <= 1'b1;
                    start_adc <= 1'b0;
                    data_2_bram <= {4'b0100, adc_data};
                    amp_seq <= amp_seq - 1;
                end
                else begin
                    start_wr <= 1'b0;
                    start_adc <= 1'b0; 
                    amp_seq <= amp_seq;
                end
            end
            6'd42 : begin
                start_wr <= 1'b0;
                // VE OUTPUT VOLTAGE CLEAR
                vout <= VC_0;
                chan_dac <= 3'd4;
                start_dac <= 1'b1;
                amp_seq <= amp_seq + 1;
            end
            6'd43 : begin
                if (dac_done_channel) begin
                    start_dac <= 1'b0;
                    amp_seq <= amp_seq + 1;
                end
                else begin
                    start_dac <= 1'b0;
                    amp_seq <= amp_seq;
                end
            end
            6'd44 : begin
                vout <= VC_0;
                // VG OUTPUT VOLTAGE CLEAR
                chan_dac <= 3'd0;
                start_dac <= 1'b1;
                amp_seq <= amp_seq + 1;
            end
            6'd45 : begin
                if (dac_done_channel) begin
                    start_dac <= 1'b0;
                    amp_seq <= amp_seq + 1;
                end
                else begin
                    start_dac <= 1'b0;
                    amp_seq <= amp_seq;
                end
            end
            6'd46: begin
                cmrrkey <= 1'b0;
                cmrr_in_p <= 1'b0;
                cmrr_in_n <= 1'b0;
                cmrr_o <= 1'b0;
                ampsinkey <= 1'b0;
                amp_seq <= 6'd0;
                AMP_CMRR_DONE <= 1'b1;
                Unit_amp_finish <= 1'b1;
            end
            default :;
            endcase
        end
        UNITADC : begin
            case(uadc_seq)
            4'd0 : begin
                if (dac_done_wtm) begin
                    Unit_adc_finish <= 1'b0;
                    adcsinkey <= 1'b1;
                    user_lsb <= FREQ_20K_LSB;
                    user_msb <= FREQ_20K_MSB;
                    dds_start <= 1'b1;   
                    uadc_seq <= uadc_seq + 1;       
                end
                else begin
                    Unit_adc_finish <= 1'b0; 
                    uadc_seq <= uadc_seq;
                end
            end
            4'd1 : begin
                if (dds_done) begin
                    // VG由DAC的通道A给出
                    dds_start <= 1'b0;
                    vout <= BIAS_VG_ADC;
                    chan_dac <= 3'd0;
                    start_dac <= 1'b1;
                    uadc_seq <= uadc_seq + 1;    
                end
                else begin
                    dds_start <= 1'b0;
                    uadc_seq <= uadc_seq;
                end
            end
            4'd2 : begin
                if(dac_done_channel) begin
                    start_dac <= 1'b0;
                    uadc_seq <= uadc_seq + 1;
                end
                else begin
                    start_dac <= 1'b0; 
                    uadc_seq <= uadc_seq;
                end
            end
            4'd3 : begin
                // 启动DAC通道D
                chan_dac <= 3'd3;
                start_dac <= 1'b1;
                vout <= BIAS_VD;
                uadc_seq <= uadc_seq + 1;
            end
            4'd4 : begin
                if (dac_done_channel) begin
                    start_dac <= 1'b0;
                    delay_en <= 1'b1;
                    uadc_seq <= uadc_seq + 1; 
                end
                else begin
                    start_dac <= 1'b0;
                    uadc_seq <= uadc_seq;
                end
            end
            4'd5 : begin
                if (delay_done) begin
                    delay_en <= 1'b0;
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
                    sample_cnt <= 9'd0;
                end
            end
            4'd7 : begin
                // VD OUTPUT VOLTAGE CLEAR
                vout <= VC_0;
                chan_dac <= 3'd3;
                start_dac <= 1'b1;
                uadc_seq <= uadc_seq + 1;
            end
            4'd8 : begin
                if (dac_done_channel) begin
                    start_dac <= 1'b0;
                    uadc_seq <= uadc_seq + 1;
                end
                else begin
                    start_dac <= 1'b0;
                    uadc_seq <= uadc_seq;
                end
            end
            4'd9 : begin
                vout <= VC_0;
                // VG OUTPUT VOLTAGE CLEAR
                chan_dac <= 3'd0;
                start_dac <= 1'b1;
                uadc_seq <= uadc_seq + 1;
            end
            4'd10 : begin
                if (dac_done_channel) begin
                    start_dac <= 1'b0;
                    uadc_seq <= uadc_seq + 1;
                end
                else begin
                    start_dac <= 1'b0;
                    uadc_seq <= uadc_seq;
                end
            end
            4'd11 : begin
                UNITADC_DONE <= 1'b1;
                adcsinkey <= 1'b0;
                Unit_adc_finish <= 1'b1;
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
            ps_tracker <= {8'd4, dcdc_seq};
        end
        DCDC_VOUT : begin
            ps_tracker <= {8'd8, dcdc_seq};
        end
        DCDC_LINE : begin
            ps_tracker <= {8'd12, dcdc_seq};
        end
        DCDC_LOAD : begin
            ps_tracker <= {8'd16, dcdc_seq};
        end
        AMP_VOS : begin
            ps_tracker <= {8'd20, amp_seq};
        end
        AMP_IOS_N : begin
            ps_tracker <= {8'd24, amp_seq};
        end
        AMP_IOS_P : begin
            ps_tracker <= {8'd28, amp_seq};
        end
        AMP_RIRO : begin
            ps_tracker <= {8'd32, amp_seq};
        end
        AMP_CMRR : begin
            ps_tracker <= {8'd36, amp_seq};
        end
        UNITADC : begin
            ps_tracker <= {8'd40, uadc_seq};
        end
        endcase
end
// ================================= PS TRACKER SETTTING ========================================

// ================================= MODULE INSTANTIATION =======================================
adctop  adctop_inst (
    .clk(sys_clk),
    .user_freq(user_freq),
    .start_en(start_adc),
    .channel(chan_adc),
    .ADC_DOUT(ADC_DOUT),
    .ADC_sclk(ADC_SCLK),
    .ADC_csn(ADC_CSN),
    .ADC_din(ADC_DIN),
    .ADC_state(adc_state),
    .ADC_conv_done(adc_done),
    .ADC_Datain(adc_data)
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
// ================================= MODULE INSTANTIATION =======================================
endmodule
