`timescale 1ns/1ps
module ddstop 
(
    input clk,
    input start_dds,
    input [15:0] freq_lsb,
    input [15:0] freq_msb,
    output reg dds_done,
    output FSYNC,
    output SCLK_DDS,
    output SDATA
);
localparam IDLE = 5'b00001;
localparam CTRL_1 = 5'b00010;
localparam LSB = 5'b00100;
localparam MSB = 5'b01000;
localparam CTRL_2 = 5'b10000;

reg CTRL_1_DONE = 1'b0;
reg LSB_DONE = 1'b0;
reg MSB_DONE = 1'b0;
reg CTRL_2_DONE = 1'b0;
reg [4:0] state = IDLE;
reg [4:0] next_state = IDLE;
reg start_dds_serial = 1'b0;
wire serial_done;
reg [15:0] cur_data = 16'd0;
reg [3:0] flow_cnt = 4'd0;
// reg [15:0] freq_lsb = 16'h50c7;
// reg [15:0] freq_msb = 16'h4000;
reg [15:0] control = 16'h2100;
reg [15:0] control_2 = 16'h2000; 
reg [31:0] sclk_div = 32'd32;

always @(posedge clk) begin
    state <= next_state;
end
always @(*) begin
    case(state)
    IDLE : begin
        if (start_dds) begin
            next_state = CTRL_1;
        end
        else
            next_state = state; 
    end
    CTRL_1 : begin
    if (CTRL_1_DONE) begin
        next_state= LSB;
    end
    else
        next_state = state; 
    end
    LSB : begin
        if (LSB_DONE) begin
            next_state=MSB;
        end
        else
            next_state = state; 
    end
    MSB : begin
        if (MSB_DONE) begin
            next_state=CTRL_2;
        end
        else
            next_state = state; 
    end
    CTRL_2 : begin
        if (CTRL_2_DONE) begin
            next_state=IDLE;
        end
        else
            next_state = state; 
    end
    default : begin
        next_state = IDLE;
    end
    endcase
end 
always @(posedge clk) begin
    case(state)
    IDLE : begin
        CTRL_1_DONE <= 1'b0;
        CTRL_2_DONE <= 1'b0;
        LSB_DONE <= 1'b0;
        MSB_DONE <= 1'b0;
        start_dds_serial <= 1'b0;
        dds_done <= 1'b0;
        sclk_div <= 32'd32;
        dds_done <= 1'b0;
        flow_cnt <= 4'd0;
    end
    CTRL_1 : begin
        case(flow_cnt)
        4'd0 : begin
            start_dds_serial <= 1'b1;
            cur_data <= control;
            flow_cnt <= flow_cnt + 4'd1;
        end
        4'd1 : begin
            start_dds_serial <= 1'b0;
            if (serial_done) begin
                CTRL_1_DONE <= 1'b1;
                flow_cnt <= flow_cnt + 4'd1;
            end
            else
                flow_cnt <= flow_cnt;
        end
        endcase
    end
    
    LSB : begin
        case(flow_cnt)
        4'd2 : begin
            start_dds_serial <= 1'b1;
            cur_data <= freq_lsb;
            flow_cnt <= flow_cnt + 4'd1;
        end
        4'd3 : begin
            start_dds_serial <= 1'b0;
            if (serial_done) begin
                LSB_DONE <= 1'b1;
                flow_cnt <= flow_cnt + 4'd1;
            end
            else
                flow_cnt <= flow_cnt;
        end
        endcase
    end
    MSB : begin
        case(flow_cnt)
        4'd4 : begin
            start_dds_serial <= 1'b1;
            cur_data <= freq_msb;
            flow_cnt <= flow_cnt + 4'd1;
        end
        4'd5 : begin
            start_dds_serial <= 1'b0;
            if (serial_done) begin
                
                MSB_DONE <= 1'b1;
                flow_cnt <= flow_cnt + 4'd1;
            end
            else
                flow_cnt <= flow_cnt;
        end
        endcase 
    end
    CTRL_2 : begin
        case(flow_cnt)
        4'd6 : begin
            start_dds_serial <= 1'b1;
            cur_data <= control_2;
            flow_cnt <= flow_cnt + 4'd1;
        end
        4'd7 : begin
            start_dds_serial <= 1'b0;
            if (serial_done) begin
                dds_done <= 1'b1;
                CTRL_2_DONE <= 1'b1;
                flow_cnt <= flow_cnt + 4'd1;
            end
            else
                flow_cnt <= flow_cnt;
        end
        endcase
    end 
    default : begin
        CTRL_1_DONE <= 1'b0;
        CTRL_2_DONE <= 1'b0;
        LSB_DONE <= 1'b0;
        MSB_DONE <= 1'b0;
        start_dds_serial <= 1'b0;
        dds_done <= 1'b0;
        sclk_div <= 32'd32;
        dds_done <= 1'b0;
        flow_cnt <= 4'd0;
    end
    endcase
end 
ddsspi  ddsspi_inst (
    .clk(clk),
    .data(cur_data),
    .start(start_dds_serial),
    .done(serial_done),
    .sclk_div(sclk_div),
    .FSYNC(FSYNC),
    .SCLK(SCLK_DDS),
    .SDATA(SDATA)
  );
endmodule