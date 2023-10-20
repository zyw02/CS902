module rs422(
    input                   sys_clk,
    input                   sys_rst_n,
    input                   RS422_START,
    output  reg             RS422_FINISH,
    input                   RS422_IN,
    output  reg             RS422_DI,
    output  reg [31:0]      RS422_ERROR
);
localparam  RS422_IDLE    = 4'b0001,
            RS422_SEND    = 4'b0010,
            RS422_RECV    = 4'b0100,
            RS422_END     = 4'b1000,
            PKG_LEN       = 65535;
reg         rs422_send_one;
reg         rs422_recv_one;
reg         rs422_idle_jmp;
reg         rs422_end_jmp;
wire        rs422_cmp;
reg [1:0]   wait_cycle;
reg [3:0]   cstate;
reg [3:0]   nstate;
(*keep = "true"*) reg [15:0]  counter;

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        cstate <= RS422_IDLE;
    end else
        cstate <= nstate;
end
always @(*) begin
    case(cstate)
        RS422_IDLE : begin
            if (RS422_START) begin
                nstate = RS422_SEND;
            end else
                nstate = cstate;
        end
        RS422_SEND : begin
            nstate = RS422_RECV;
        end
        RS422_RECV : begin
            if (rs422_end_jmp) begin
                nstate = RS422_END;
            end else if (rs422_recv_one) begin
                nstate = RS422_SEND;
            end else
                nstate = cstate;
        end
        RS422_END : begin
            if (rs422_idle_jmp) begin
                nstate = RS422_IDLE;
            end else
                nstate = cstate;
        end
        default : begin
            nstate = RS422_IDLE;
        end 
    endcase
end

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        rs422_idle_jmp <= 1'b0;
        rs422_end_jmp <= 1'b0;
        rs422_send_one <= 1'b0;
        rs422_recv_one <= 1'b0;
        wait_cycle <= 2'd0;
        counter <= 16'd0;
        RS422_DI <= 1'b0;
        RS422_ERROR <= 32'd0;
        RS422_FINISH <= 1'b0;
    end else
        case(cstate)
            RS422_IDLE : begin
                rs422_idle_jmp <= 1'b0;
                rs422_end_jmp <= 1'b0;
                rs422_send_one <= 1'b0;
                rs422_recv_one <= 1'b0;
                wait_cycle <= 2'd0;
                counter <= 16'd0;
                RS422_DI <= 1'b0;
                RS422_ERROR <= 32'd0;
                RS422_FINISH <= 1'b0;
            end
            RS422_SEND : begin
                RS422_DI <= ~RS422_DI;
                rs422_send_one <= 1'b1;
                rs422_recv_one <= 1'b0;
                wait_cycle <= 2'd0;
            end
            RS422_RECV : begin
                rs422_send_one <= 1'b0;
                if ((wait_cycle < 2'd2)) begin
                    wait_cycle <= wait_cycle + 1;
                end else if ((wait_cycle == 2'd2) && (rs422_cmp != RS422_IN) && (counter < PKG_LEN)) begin
                    RS422_ERROR <= RS422_ERROR + 1;
                    wait_cycle <= 2'd0;
                    rs422_recv_one <= 1'b1;
                    counter <= counter + 1;
                end else if ((wait_cycle == 2'd2) && (rs422_cmp == RS422_IN) && (counter < PKG_LEN)) begin
                    RS422_ERROR <= RS422_ERROR;
                    wait_cycle <= 2'd0;
                    rs422_recv_one <= 1'b1;
                    counter <= counter + 1;
                end else if ((wait_cycle == 2'd2) && (rs422_cmp != RS422_IN) && (counter == PKG_LEN)) begin
                    RS422_ERROR <= RS422_ERROR + 1;
                    wait_cycle <= 2'd0;
                    rs422_recv_one <= 1'b0;
                    rs422_end_jmp <= 1'b1;
                end else if ((wait_cycle == 2'd2) && (rs422_cmp == RS422_IN) && (counter == PKG_LEN)) begin
                    RS422_ERROR <= RS422_ERROR;
                    wait_cycle <= 2'd0;
                    rs422_recv_one <= 1'b0;
                    rs422_end_jmp <= 1'b1;
                end
            end
            RS422_END : begin
                rs422_idle_jmp <= 1'b1;
                RS422_FINISH <= 1'b1;
            end
        endcase
end
assign rs422_cmp = RS422_DI;
endmodule