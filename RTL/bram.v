module BRAM_0(  
    input   BRAM_sys_clk    ,
    input   BRAM_rst_n      ,  

    input   BRAM_start      ,                   //
    input   send_end        ,                   //

    output  reg [31:0]  BRAM_error  ,           //
    output  reg BRAM_end                        //  
 
);   

parameter   TEST_DATA   =   16'hAA55    ;       //


/*-----------------------BRAM-----------------*///
reg                         wea     ;           //
reg [9:0]                   addra   ;           //
(*KEEP="TRUE"*)reg [15:0]   dina    ;           //BRAM
reg [9:0]                   addrb   ;           //BRAM
(*KEEP="TRUE"*)wire [15:0]  doutb   ;           //BRAM

BRAM_ip test_bram (
  .clka(BRAM_sys_clk), // input clka
  .wea(wea), // input [0 : 0] wea
  .addra(addra), // input [9 : 0] addra
  .dina(dina), // input [15 : 0] dina
  .clkb(BRAM_sys_clk), // input clkb
  .addrb(addrb), // input [9 : 0] addrb
  .doutb(doutb) // output [15 : 0] doutb
);

/*
wire    WEA     ;
wire [9:0]  ADDRA   ;
wire [15:0] DINA    ;
wire [9:0]  ADDRB   ;
wire [15:0] DOUTB   ;

*/


/*
assign  WEA = wea   ;
assign  ADDRA = addra   ;
assign  DINA  = dina    ;
assign  ADDRB = addrb   ;
assign  DOUTB = doutb   
*/

/*-------------------------end---------------------------*/

/*------------------------test------------------------*/

reg [15:0]  wr_data ;   // SRAM

parameter   W_IDLE  =   5'b00001    ,
            WRITE   =   5'b00010    ,  
            R_IDLE  =   5'b00100    ,  
            READ    =   5'b01000    , 
            R_END   =   5'b10000    ;

(*KEEP="TRUE"*)reg [4:0]    cstate  ,
                            nstate  ;   
 
always @ (posedge BRAM_sys_clk or negedge BRAM_rst_n) begin  //鏃跺簭閫昏緫鎺у埗鐘舵€佸彉杩
    if(!BRAM_rst_n) begin
        cstate <= W_IDLE;  
    end
    else begin
        cstate <= nstate;  
    end
end
 
reg W_FINISH    ;       //
reg R_FINISH    ;       //     
reg error_flag  ;       //
 
always @ (*) begin      //
    case (cstate)  
        W_IDLE: begin
            nstate <= WRITE         ;
        end
        WRITE: begin
            if(W_FINISH) begin
                nstate <= R_IDLE    ;       
            end  
            else begin
                nstate <= WRITE     ;  
            end
        end
        R_IDLE: begin
            if(BRAM_start && W_FINISH) begin
                nstate <= READ      ;  
            end
            else begin
                nstate <= R_IDLE    ;    
            end   
        end      
        READ: begin
            if(R_FINISH) begin
                nstate <= R_END     ;  
            end
            else begin
                nstate <= READ      ;    
            end          
        end
        R_END: begin
            if(send_end) begin
                if(error_flag) begin
                    nstate <= W_IDLE    ;
                end
                else begin
                    nstate <= R_IDLE    ;
                end
            end
            else begin
                nstate <= R_END         ;
            end
        end
        default: begin
            nstate <= W_IDLE            ;
        end  
    endcase  
end  

 
always @ (posedge BRAM_sys_clk or negedge BRAM_rst_n) begin      
    if(!BRAM_rst_n) begin
        addra <= 10'd0      ;
        dina <= 16'd0       ;
        wea <= 1'b0         ;
        addrb <= 10'd0      ;
        error_flag <= 1'b0  ;
        BRAM_end <= 1'b0    ;
        R_FINISH <= 1'b0    ;
        W_FINISH <= 1'b0    ;
        BRAM_error <= 32'd0 ;
    end
    else begin  
        case (cstate)  
            W_IDLE: begin
                addra <= 10'd0      ;
                wea <= 1'b1         ;
                dina <= TEST_DATA   ;
                error_flag <= 1'b0  ;
                BRAM_end <= 1'b0    ;
                W_FINISH <= 1'b0    ;
            end
            WRITE: begin
                if(addra < 10'h3FF) begin
                    addra <= addra + 1'b1   ;
                end
                else begin
                    W_FINISH <= 1'b1        ;
                    wea <= 1'b0             ;
                end
            end
            R_IDLE: begin
                addrb <= 10'd1      ;
                BRAM_end <= 1'b0    ;
                R_FINISH <= 1'b0    ;
                BRAM_error <= 32'd0 ;
            end
            READ: begin
                if(doutb != TEST_DATA) begin
                //if(doutb != 16'b0) begin	//test
                    BRAM_error <= BRAM_error + 1'b1   ;
                    error_flag <= 1'b1                                  ;
                end
                else begin
                    BRAM_error  <=  BRAM_error  ;
                    error_flag  <=  error_flag  ;
                end
                if(addrb == 10'h3FF) begin
                    R_FINISH <= 1'b1        ;
                    
                end
                else begin
                    addrb <= addrb + 1'b1   ;
                end

            end
            R_END: begin
                BRAM_end <= 1'b1            ;
                addrb <= 0   ;
            end
            default: begin
                addra <= 10'd0              ;
                dina <= 16'd0               ;
                wea <= 1'b0                 ;
                addrb <= 10'd0              ;
                error_flag <= 1'b0          ;
                BRAM_end <= 1'b0            ;
                R_FINISH <= 1'b0            ;
                W_FINISH <= 1'b0            ;
                BRAM_error <= 32'd0         ;
            end 
        endcase  
    end  
end

/*---------------------end---------------------*/

endmodule