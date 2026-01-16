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