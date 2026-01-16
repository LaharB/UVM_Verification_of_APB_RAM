//the APB_RAM acts as a slave
// DUT  
module apb_ram(
    //M to S
    input presetn, //active low reset
    input pclk,
    input psel,
    input penable,
    input pwrite,
    input [31:0] paddr, pwdata, //32-bit address and data
    
    //S to M
    output reg [31:0] prdata,
    output reg pready, pslverr 
);

    reg [31:0] mem[32]; //this is the memory on which will do transactions according to APB Protocol

    typedef enum {idle = 0, setup = 1, access = 2, transfer = 3} state_type; //no data type specified - int(default)
    state_type state = idle;

    always@(posedge pclk)
        begin
            if(presetn == 1'b0)
                begin
                    state <= idle;
                    prdata <= 32'h00000000;
                    pready <= 1'b0;
                    pslverr <= 1'b0; 
                //initialize the memory to 0
                for(int i = 0; i < 32; i++)
                    begin
                        mem[i] <= 0;
                    end
                end
            else
                    begin
                        case(state)
                            
                        idle : begin
                            prdata <= 32'h00000000;
                            pready <= 1'b0;
                            pslverr <= 1'b0;                            
                            state <= setup; 
                        end

                        setup : 
                            begin
                                if(psel == 1'b1) //M to S
                                    state <= access;
                                else
                                    state <= setup;
                            end

                        access : 
                            begin
                                //write
                                if(pwrite && penable)
                                    begin
                                        if(paddr < 32) // if addr within range , write into the memory
                                            begin
                                               mem[paddr] <= pwdata;
                                               state <= transfer;
                                               pslverr <= 1'b0;
                                               pready <= 1'b1;
                                            end 
                                        else
                                            begin
                                                state <= transfer;
                                                pready <= 1;
                                                pslverr <= 1;
                                            end
                                    end
                            //read
                            else if(!pwrite && penable)
                                begin
                                    if(paddr < 32)
                                        begin
                                            prdata <= mem[paddr];
                                            state <= transfer;
                                            pready <= 1'b1;
                                            pslverr <= 1'b0;
                                        end 
                                    else 
                                        begin
                                            state <= transfer;
                                            pready <= 1'b1;
                                            pslverr <= 1'b1;
                                            prdata <= 32'hxxxxxxxx;
                                        end
                                end
                             else 
                                state <= setup; //stay in setup phase until penable becomes high
                        end 

                        transfer : 
                            begin
                                state <= setup;
                                pready <= 1'b0;
                                pslverr <= 1'b0;
                            end                       
                        
                        default : state <= idle;
                        
                        endcase
                    end 
        end   
endmodule

////////////////////////////////////////////////////////
// Interface 
interface apb_if();
    //M to S
    logic pclk;
    logic presetn;
    logic [31:0] paddr;
    logic pwrite;
    logic [31:0] pwdata;
    logic penable;
    logic psel;
    //S to M
    logic [31:0] prdata;
    logic pslverr;
    logic pready; 

endinterface  