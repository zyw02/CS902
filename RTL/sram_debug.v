module sramtwo  (  
    input               SRAM_sys_clk,
    input               SRAM_rst_n,  

    input               SRAM_start,       //开始标志

    output  reg [31:0]  SRAM_error,       //错误计数
    output  reg         SRAM_end,       //结束标志

    output  reg [17:0]  SRAM_A,       //SRAM地址线
    inout       [15:0]  SRAM_DB,      //SRAM数据线
    output  reg         SRAM_WE_N,    //写使能
    output  reg         SRAM_OE_N,    //读使能
    output              SRAM_UB_N,    //高字节有效
    output              SRAM_LB_N,    //低字节有效
    output              SRAM_CE_N    //芯片使能
);     
wire [15:0] SRAM_DB_I;
wire [15:0] SRAM_DB_O;
localparam TEST_DATA = 16'haa55;
(*keep = "true"*) reg [17:0] cur;
/*------------------------状态机------------------------*/

reg [15:0]  wr_data ;   // 写数据寄存器
reg [15:0]  rd_std;
(*keep = "true"*) reg [15:0]  probe_data;

parameter   W_IDLE  =   5'b00001,
            WRITE   =   5'b00010,  
            R_IDLE  =   5'b00100,  
            READ    =   5'b01000, 
            R_END   =   5'b10000;

(*keep = "true"*) reg [4:0]   cstate,
            nstate;   

(*keep = "true"*) reg [2:0] cycle_cnt; 
always @ (posedge SRAM_sys_clk or negedge SRAM_rst_n) begin  //时序逻辑控制状态变迁。  
    if(!SRAM_rst_n) begin
        cstate <= W_IDLE    ;  
    end
    else begin
        cstate <= nstate    ;  
    end
end

reg W_FINISH;           //写完成，用于状态转移
reg R_FINISH;           //读完成，用于状态转移
reg error_flag;           //错误标志，用于状态转移

always @ (*) begin          //组合逻辑控制不同状态的转换。  
    case (cstate)  
        W_IDLE: begin
            nstate = WRITE;
        end
        WRITE: begin
            if(W_FINISH) begin
                nstate <= R_IDLE;       
            end  
            else begin
                nstate <= WRITE;  
            end
        end
        R_IDLE: begin
            if(W_FINISH && SRAM_start) begin
                nstate <= READ;  
            end
            else begin
                nstate <= R_IDLE;    
            end   
        end      
        READ: begin
            if(R_FINISH) begin
                nstate <= R_END;  
            end
            else begin
                nstate <= READ;    
            end          
        end
        R_END: begin
            if(error_flag) begin
                nstate <= W_IDLE;
            end
            else begin
                nstate <= R_IDLE;
            end
        end
        default: begin
            nstate <= W_IDLE;
        end
    endcase  
end  
 
always @ (posedge SRAM_sys_clk or negedge SRAM_rst_n) begin      
    if(!SRAM_rst_n) begin
        SRAM_WE_N <= 1'b1;
        SRAM_OE_N <= 1'b1;
        SRAM_A <= 18'd0;  
        error_flag <= 1'b0;
        SRAM_end <= 1'b0;
        R_FINISH <= 1'b0;
        W_FINISH <= 1'b0;
        wr_data <= 16'd0;
        rd_std <= 16'd0;
        SRAM_error <= 32'd0;
        cycle_cnt <= 3'd0;
        cur <= 18'd0;
        probe_data <= 16'd0;
    end
    else begin  
        case (cstate)  
            W_IDLE: begin
                SRAM_WE_N <= 1'b0;
                SRAM_A <= 18'd0;
                error_flag <= 1'b0;
                SRAM_end <= 1'b0;
                W_FINISH <= 1'b0;
                wr_data <= TEST_DATA;
                SRAM_error <= 32'd0;
                cycle_cnt <= 3'd0;
                cur <= 18'd0;
                probe_data <= 16'd0;
            end
            WRITE: begin
                if(SRAM_A == 18'h3FFFF) begin
                    W_FINISH <= 1'b1;
                    SRAM_WE_N <= 1'b1;
                end
                else begin
                    wr_data <= wr_data ^ 16'hffff;
                    SRAM_A <= SRAM_A + 1'b1;
                end
            end
            R_IDLE: begin
                SRAM_end <= 1'b0;
                SRAM_A <= 18'd0;
                SRAM_OE_N <= 1'b0;
                R_FINISH <= 1'b0;
                rd_std <= TEST_DATA;
                SRAM_error <= 32'd0;
                cycle_cnt <= 3'd0;
                probe_data <= SRAM_DB_I;
            end
            READ: begin
                cur <= SRAM_A;
                probe_data <= SRAM_DB_I;
                if (cycle_cnt == 3'd7) begin
                    if(SRAM_DB_I != rd_std) begin
                        SRAM_error <= SRAM_error + 1'b1;
                        error_flag <= 1'b1;
                    end
                    else begin
                        SRAM_error <= SRAM_error;
                        error_flag <= error_flag;
                    end
                    if((SRAM_A == 18'h3FFFF)) begin
                        R_FINISH <= 1'b1;
                        cycle_cnt <= 1'b0;
                        SRAM_OE_N <= 1'b1;
                    end
                    else begin
                        rd_std <= rd_std ^ 16'hffff;
                        SRAM_A <= SRAM_A + 1'b1;
                        cycle_cnt <= 1'b0;
                    end
                end else
                    cycle_cnt <= cycle_cnt + 1;
            end
            R_END: begin
                SRAM_end <= 1'b1;
            end
            default: begin    
                SRAM_WE_N <= 1'b1;
                SRAM_OE_N <= 1'b1;
                SRAM_A <= 18'd0;  
                error_flag <= 1'b0;
                SRAM_end <= 1'b0;
                R_FINISH <= 1'b0;   
                W_FINISH <= 1'b0;
                wr_data <= 16'd0;
                SRAM_error <= 32'd0;
            end 
        endcase  
    end  
end

/*-------------------------end--------------------------*/
//
assign SRAM_DB_O = wr_data; 
genvar i;
generate
    for(i=0;i<16;i=i+1) begin
        // iobuf
        IOBUF #(
            .DRIVE(12), // Specify the output drive strength
            .IBUF_LOW_PWR("TRUE"),  // Low Power - "TRUE", High Performance = "FALSE" 
            .IOSTANDARD("DEFAULT"), // Specify the I/O standard
            .SLEW("SLOW") // Specify the output slew rate
        ) IOBUF_inst (
            .O(SRAM_DB_I[i]),     // Buffer output
            .IO(SRAM_DB[i]),   // Buffer inout port (connect directly to top-level port)
            .I(SRAM_DB_O[i]),     // Buffer input
            .T(SRAM_WE_N)      // 3-state enable input, high=input, low=output
        );
    end
endgenerate 
 
assign SRAM_UB_N = 1'b0;    //始终有效
assign SRAM_LB_N = 1'b0;    //始终有效
assign SRAM_CE_N = 1'b0;    //始终有效

endmodule  