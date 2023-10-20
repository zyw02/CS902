module notgate(
    input                   sys_clk,
    input                   sys_rst_n,
    input                   NOT_START,
    output  reg             NOT_FINISH,
    input                   NOT_IN,
    output  reg             NOT_OUT,
    output  reg [31:0]      NOT_ERROR
);
localparam  NOT_IDLE    = 4'b0001,
            NOT_SEND    = 4'b0010,
            NOT_RECV    = 4'b0100,
            NOT_END     = 4'b1000,
            PKG_LEN     = 65535;
reg         not_send_one;
reg         not_recv_one;
reg         not_idle_jmp;
reg         not_end_jmp;
wire        not_cmp;
reg [1:0]   wait_cycle;
reg [3:0]   cstate;
reg [3:0]   nstate;
(*keep = "true"*) reg [15:0]  counter;

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        cstate <= NOT_IDLE;
    end else
        cstate <= nstate;
end
always @(*) begin
    case(cstate)
        NOT_IDLE : begin
            if (NOT_START) begin
                nstate = NOT_SEND;
            end else
                nstate = cstate;
        end
        NOT_SEND : begin
            nstate = NOT_RECV;
        end
        NOT_RECV : begin
            if (not_end_jmp) begin
                nstate = NOT_END;
            end else if (not_recv_one) begin
                nstate = NOT_SEND;
            end else
                nstate = cstate;
        end
        NOT_END : begin
            if (not_idle_jmp) begin
                nstate = NOT_IDLE;
            end else
                nstate = cstate;
        end
        default : begin
            nstate = NOT_IDLE;
        end 
    endcase
end

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        not_idle_jmp <= 1'b0;
        not_end_jmp <= 1'b0;
        not_send_one <= 1'b0;
        not_recv_one <= 1'b0;
        wait_cycle <= 2'd0;
        counter <= 16'd0;
        NOT_OUT <= 1'b0;
        NOT_ERROR <= 32'd0;
        NOT_FINISH <= 1'b0;
    end else
        case(cstate)
            NOT_IDLE : begin
                not_idle_jmp <= 1'b0;
                not_end_jmp <= 1'b0;
                not_send_one <= 1'b0;
                not_recv_one <= 1'b0;
                wait_cycle <= 2'd0;
                counter <= 16'd0;
                NOT_OUT <= 1'b0;
                NOT_ERROR <= 32'd0;
                NOT_FINISH <= 1'b0;
            end
            NOT_SEND : begin
                NOT_OUT <= ~NOT_OUT;
                not_send_one <= 1'b1;
                not_recv_one <= 1'b0;
                wait_cycle <= 2'd0;
            end
            NOT_RECV : begin
                not_send_one <= 1'b0;
                if ((wait_cycle < 2'd2)) begin
                    wait_cycle <= wait_cycle + 1;
                end else if ((wait_cycle == 2'd2) && (not_cmp != NOT_IN) && (counter < PKG_LEN)) begin
                    NOT_ERROR <= NOT_ERROR + 1;
                    wait_cycle <= 2'd0;
                    not_recv_one <= 1'b1;
                    counter <= counter + 1;
                end else if ((wait_cycle == 2'd2) && (not_cmp == NOT_IN) && (counter < PKG_LEN)) begin
                    NOT_ERROR <= NOT_ERROR;
                    wait_cycle <= 2'd0;
                    not_recv_one <= 1'b1;
                    counter <= counter + 1;
                end else if ((wait_cycle == 2'd2) && (not_cmp != NOT_IN) && (counter == PKG_LEN)) begin
                    NOT_ERROR <= NOT_ERROR + 1;
                    wait_cycle <= 2'd0;
                    not_recv_one <= 1'b0;
                    not_end_jmp <= 1'b1;
                end else if ((wait_cycle == 2'd2) && (not_cmp == NOT_IN) && (counter == PKG_LEN)) begin
                    NOT_ERROR <= NOT_ERROR;
                    wait_cycle <= 2'd0;
                    not_recv_one <= 1'b0;
                    not_end_jmp <= 1'b1;
                end
            end
            NOT_END : begin
                not_idle_jmp <= 1'b1;
                NOT_FINISH <= 1'b1;
            end
        endcase
end
assign not_cmp = ~NOT_OUT;
// assign not_cmp = NOT_OUT;
endmodule