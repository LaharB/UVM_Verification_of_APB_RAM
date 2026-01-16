# UVM Based Verification of APB RAM 

This repository showcases the verification of APB based RAM using Universal Verification Methodology(UVM).It is simulated using Vivado 2025.1 tool.

The APB RAM does not implement any wait state during write and read transaction and PROT or PSTRB signal.

__________________________________________________________________

<details><summary>AMBA APB Protocol: Technical Specification</summary>

## Introduction
The **Advanced Peripheral Bus (APB)** is part of the AMBA hierarchy and is optimized for low-bandwidth, low-complexity peripheral communication. Unlike AXI, APB is a non-pipelined, simple synchronous interface. It is primarily used to connect to registers in peripherals like UART, I2C, Timers, and GPIO.

## Key Features
* **Low Power:** Minimal signal switching for simple register access.
* **Low Complexity:** Simple state machine with only 3 states.
* **Synchronous:** All signal transitions are related to the rising edge of the clock.
* **Non-Pipelined:** Only one transaction is handled at a time.

---

## 1. APB State Machine
The APB transaction is governed by a simple 3-state Finite State Machine (FSM):

1.  **IDLE:** The default state. No transaction is occurring.
2.  **SETUP:** When a transfer is required, the bus moves into the SETUP state. The address (`PADDR`), data (`PWDATA`), and control signals are asserted. This state always lasts for exactly one clock cycle.
3.  **ACCESS:** The enable signal (`PENABLE`) is asserted. The bus remains in this state until the slave responds with `PREADY`.

![alt text](<docs/APB Protocol State Diagram.png>)
*Figure 1: APB Operating States*

---

## 2. Signal Descriptions
Based on the standard APB4 specification, the interface signals are defined as follows:

| Signal | Direction | Description |
| :--- | :--- | :--- |
| **PCLK** | Input | Clock signal. All transitions occur on the rising edge. |
| **PRESETn** | Input | System Reset (Active Low). |
| **PADDR** | Master &rarr; Slave | Peripheral Address bus (up to 32 bits). |
| **PSELx** | Master &rarr; Slave | Peripheral Select. Indicates that the slave device is selected. |
| **PENABLE** | Master &rarr; Slave | Enable. Indicates the second and subsequent cycles of a transfer. |
| **PWRITE** | Master &rarr; Slave | Write/Read. High = Write, Low = Read. |
| **PWDATA** | Master &rarr; Slave | Write Data bus (driven when PWRITE is High). |
| **PREADY** | Slave &rarr; Master | Ready. Used by the slave to extend a transfer (wait states). |
| **PRDATA** | Slave &rarr; Master | Read Data bus (driven when PWRITE is Low). |
| **PSLVERR** | Slave &rarr; Master | Slave Error. Indicates a failure in the transaction. |

---

## 3. Transaction Waveforms

### Write Transaction
In a write cycle, the master provides the address and data during the SETUP phase. The transaction completes when `PENABLE` and `PREADY` are both high.

### Read Transaction
In a read cycle, the slave provides the data on the `PRDATA` bus during the ACCESS phase. The master samples the data on the rising edge of the clock when `PREADY` is asserted.

![alt text](<docs/APB Protocol Write Transfer with No Wait state .png>)
*Figure 2: APB Write Timing Diagram without wait state*

![alt text](<docs/APB Protocol Read Transfer with No Wait state .png>)
*Figure 3: APB ReadTiming Diagram without wait state*

---

## 4. APB4 vs. APB3 Differences
* **PREADY:** Added in APB3 to allow slaves to insert wait states.
* **PSLVERR:** Added in APB3 to signal error conditions.
* **PPROT & PSTRB:** Added in APB4. `PPROT` indicates protection levels, and `PSTRB` allows for byte-level sparse data transfers (Write Strobes).

</details>

_________________________________________________________________

<details><summary>RTL Design</summary>

```systemverilog
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
```
</details>

__________________________________________________________________

<details><summary>Testbench</summary>

```systemverilog
`timescale 1ns/1ps
`include "uvm_macros.svh"
import uvm_pkg::*;

//tb acts as apb master
///////////////////////////////////////////////////////////////////
//1.configuration of env
class apb_config extends uvm_object;
    `uvm_object_utils(apb_config) //apb_congig belongs to uvm_object 

    function new(input string path = "apb_config");
        super.new(path);
    endfunction

    uvm_active_passive_enum is_active = UVM_ACTIVE;

endclass
/////////////////////////////////////////////////////////////////
//using enum outside so that every class can use it 
typedef enum bit [1:0] {readData = 0, writeData = 1, rst = 2} oper_mode; 

//2.TRANSACTION
class transaction extends uvm_sequence_item;

    rand oper_mode op;
    rand logic PWRITE;
    rand logic [31:0] PWDATA;
    rand logic [31:0] PADDR;

    //output of DUT 
    logic PREADY;
    logic PSLVERR;
    logic [31:0] PRDATA;

    //factory registration + automation
    `uvm_object_utils_begin(transaction)
        `uvm_field_int(PWRITE, UVM_ALL_ON);
        `uvm_field_int(PWDATA, UVM_ALL_ON);
        `uvm_field_int(PADDR, UVM_ALL_ON);
        `uvm_field_int(PREADY, UVM_ALL_ON);
        `uvm_field_int(PSLVERR, UVM_ALL_ON);
        `uvm_field_int(PRDATA, UVM_ALL_ON);
        `uvm_field_enum(oper_mode, op, UVM_DEFAULT);
    `uvm_object_utils_end

    //constraints
    constraint addr_c { PADDR <= 31;}  //without error
    constraint addr_c_err { PADDR > 31;} //for inducing error

    //constructor
    function new(input string path = "transaction");
        super.new(path);
    endfunction
        
endclass
//////////////////////////////////////////////////////////////////
//3. SEQUENCES or GENERATOR
//we will use different sequences for different types of operations
//WRITE SEQ
class write_data extends uvm_sequence#(transaction);
    `uvm_object_utils(write_data)

    transaction tr;

    //constructor
    function new(input string path = "write_data");
        super.new(path);
    endfunction

    //body task to perform randomization
    virtual task body();
        repeat(15)
            begin
                tr = transaction::type_id::create("tr");
                tr.addr_c.constraint_mode(1); //enable addr constr to keep addr < 32
                tr.addr_c_err.constraint_mode(0); //disable addr > 32 
                start_item(tr);
                assert(tr.randomize);
                tr.op = writeData; //overwrite to writeData oper mode
                finish_item(tr); 
            end
    endtask

endclass
/////////////////////////////////////////////////////////////////////////
//READ SEQ
class read_data extends uvm_sequence#(transaction);
    `uvm_object_utils(read_data)

    transaction tr;

    //constructor
    function new(input string path = "read_data");
        super.new(path);
    endfunction

    //body task to perform randomization
    virtual task body();
        repeat(15)
            begin
                tr = transaction::type_id::create("tr");
                tr.addr_c.constraint_mode(1); //enable addr constr to keep addr < 32
                tr.addr_c_err.constraint_mode(0); //disable addr > 32
                start_item(tr);
                assert(tr.randomize);
                tr.op = readData; //overwrite to readData oper mode
                finish_item(tr); 
            end
    endtask

endclass
//////////////////////////////////////////////////////////////////////////////
//WRITE_READ SEQ
//WRITE SEQ
class write_read extends uvm_sequence#(transaction);
    `uvm_object_utils(write_read)

    transaction tr;

    //constructor
    function new(input string path = "write_read");
        super.new(path);
    endfunction

    //body task to perform randomization
    virtual task body();
        repeat(15) //write & read for 15 times 
            begin
                tr = transaction::type_id::create("tr");
                tr.addr_c.constraint_mode(1); //enable addr constr to keep addr < 32
                tr.addr_c_err.constraint_mode(0); //disable addr > 32
                //first call WRITE SEQ
                start_item(tr);
                assert(tr.randomize);
                tr.op = writeData; //overwrite to writeData oper mode
                finish_item(tr); 

                //second call READ SEQ
                start_item(tr);
                assert(tr.randomize);
                tr.op = readData; //overwrite to readData oper mode
                finish_item(tr); 
            end
    endtask

endclass
/////////////////////////////////////////////////////////////////////
//WRITE BULK READ BULK SEQ
class writeb_readb extends uvm_sequence#(transaction);
    `uvm_object_utils(writeb_readb)

    transaction tr;

    //constructor
    function new(input string path = "writeb_readb");
        super.new(path);
    endfunction

    //body task to perform randomization
    virtual task body();
        repeat(15)  
            begin
                tr = transaction::type_id::create("tr");
                tr.addr_c.constraint_mode(1); //enable addr constr to keep addr < 32
                tr.addr_c_err.constraint_mode(0); //disable addr > 32
                //write bulk
                start_item(tr);
                assert(tr.randomize);
                tr.op = writeData; //overwrite to writeData oper mode
                finish_item(tr); 
            end

        repeat(15)
            begin
                tr = transaction::type_id::create("tr");
                tr.addr_c.constraint_mode(1); //enable addr constr to keep addr < 32
                tr.addr_c_err.constraint_mode(0); //disable addr > 32
                //read bulk
                start_item(tr);
                assert(tr.randomize);
                tr.op = readData; //overwrite to readData oper mode
                finish_item(tr); 
            end
    endtask

endclass
/////////////////////////////////////////////////////////////////////
//SLV ERROR WRITE SEQ    
class write_err extends uvm_sequence#(transaction);
    `uvm_object_utils(write_err)

    transaction tr;

    //constructor
    function new(input string path = "write_err");
        super.new(path);
    endfunction

    //body task to perform randomization
    virtual task body();
        repeat(15)
            begin
                tr = transaction::type_id::create("tr");
                tr.addr_c.constraint_mode(0); //disable
                tr.addr_c_err.constraint_mode(1); //want addr > 32 so that error comes
                start_item(tr);
                assert(tr.randomize);
                tr.op = writeData; //overwrite to readData oper mode
                finish_item(tr); 
            end
    endtask
endclass
///////////////////////////////////////////////////////////////////
//READ ERR
class read_err extends uvm_sequence#(transaction);
    `uvm_object_utils(read_err)

    transaction tr;

    //constructor
    function new(input string path = "read_err");
        super.new(path);
    endfunction

    //body task to perform randomization
    virtual task body();
        repeat(15)
            begin
                tr = transaction::type_id::create("tr");
                tr.addr_c.constraint_mode(0); //disable
                tr.addr_c_err.constraint_mode(1); //want addr > 32 so that error comes
                start_item(tr);
                assert(tr.randomize);
                tr.op = readData; //overwrite to readData oper mode
                finish_item(tr); 
            end
    endtask
endclass
////////////////////////////////////////////////////////////////////////
//RESET DUT SEQ
class reset_dut extends uvm_sequence#(transaction);
    `uvm_object_utils(reset_dut)

    transaction tr;

    function new(input string path = "reset_dut");
        super.new(path);
    endfunction

    virtual task body();
        repeat(15)
            begin
                tr = transaction::type_id::create("tr");
                tr.addr_c.constraint_mode(1);
                tr.addr_c_err.constraint_mode(0);
                start_item(tr);
                assert(tr.randomize);
                tr.op = rst; //overwrite to rst operation
                finish_item(tr);
            end
    endtask
endclass
/////////////////////////////////////////////////////////////////////
//4.DRIVER
class driver extends uvm_driver#(transaction);
    `uvm_component_utils(driver)

    transaction tr; //data container to hold values sent by sequencer
    virtual apb_if vif; //access of intf to driver
    //no port creation for driver and seqr, by default : seq_item_port and seq_item_export

    //constructor 
    function new(input string path = "driver", uvm_component parent = null); //2 args as uvm_component
        super.new(path, parent);
    endfunction 

    //build phase
    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        tr = transaction::type_id::create("tr");

        //config_db and get method to give access of intf to driver
        if(!uvm_config_db#(virtual apb_if)::get(this, "", "vif", vif)) //uvm_test_top.agent.drv.aif
         `uvm_error("DRV", "Unable to access interface");
    endfunction

    //reset dut task
    task reset_dut();
        repeat(5)  //apply reset for 5 clk ticks
            begin
                vif.presetn <= 1'b0;  
                vif.pwrite <= 1'b0;
                vif.paddr <= 32'h000000000;
                vif.pwdata <= 32'h00000000;
                vif.psel <= 1'b0;
                vif.penable <= 1'b0;
                `uvm_info("DRV", "System Reset : Start of Simulation", UVM_MEDIUM);
                @(posedge vif.pclk);
            end
    endtask

    //task to drive the input signals to the DUT
    task drive();
        reset_dut(); //apply reset
        forever begin //forever block as driver has to be ready all the time 
            seq_item_port.get_next_item(tr); //tell the sequencer to send the packet
            //drive the inputs to DUT according to oper_mode
            if(tr.op == rst)
                begin
                    vif.presetn <= 0;
                    vif.pwrite <= 0;
                    vif.paddr <= 0;
                    vif.pwdata <= 32'h00000000;
                    vif.psel <= 0;
                    vif.penable <= 0;
                    //wait for 1 clk tick
                    @(posedge vif.pclk);
                end
            else if(tr.op == writeData)
                begin
                    vif.presetn <= 1'b1; //remove reset
                    vif.psel <= 1'b1; 
                    vif.pwrite <= 1'b1; //for write
                    vif.paddr <= tr.PADDR;
                    vif.pwdata <= tr.PWDATA;
                    //wait for 1 clk tick, make penable high
                    @(posedge vif.pclk);
                    vif.penable <= 1'b1;
                    `uvm_info("DRV", $sformatf("OP: %0s, addr: %0d, wdata: %0d, rdata: %0d, slverr: %0d", tr.op.name(), tr.PADDR, tr.PWDATA, tr.PRDATA, tr.PSLVERR), UVM_NONE);
                    //wait for slave to give pready 
                    @(negedge vif.pready); //S to M
                    vif.penable <= 1'b0;
                    tr.PSLVERR = vif.pslverr; //we r collecting output of DUT here in the driver
                end
            else if(tr.op == readData) 
                begin
                    vif.presetn <= 1'b1; //remove reset
                    vif.psel <= 1'b1;
                    vif.pwrite <= 1'b0; //for read
                    vif.paddr <= tr.PADDR;
                    //wait of 1 tick
                    @(posedge vif.pclk);
                    vif.penable <= 1'b1;
                    `uvm_info("DRV", $sformatf("OP: %0s, addr: %0d, wdata: %0d, rdata: %0d, slverr: %0d", tr.op.name(), tr.PADDR, tr.PWDATA, tr.PRDATA, tr.PSLVERR), UVM_NONE);
                    //wait for slave to give pready
                    @(negedge vif.pready);
                    vif.penable <= 1'b0;
                    tr.PRDATA = vif.prdata;
                    tr.PSLVERR = vif.pslverr;
                end
            seq_item_port.item_done(); //send item_done to sequencer
        end 
    endtask 

    //run phase 
    virtual task run_phase(uvm_phase phase);
        drive();    //calling the tasks 
    endtask

endclass
//////////////////////////////////////////////////////////////////////////
//5.MONITOR
class mon extends uvm_monitor;
    `uvm_component_utils(mon)

    transaction tr; //data container to store values sent by DUT and send to scoreboard
    virtual apb_if vif; //access of intf to monitor
    uvm_analysis_port #(transaction) send; //port to connect mon and sco

    //constructor 
    function new(input string path = "mon", uvm_component parent = null); //2 args as uvm_component
        super.new(path, parent);
        //create the send port here itself
        send = new("send", this); //2 args
    endfunction 

    //build phase
    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        tr = transaction::type_id::create("tr");
        //config_db and get method to give access of intf to monitor
        if(!uvm_config_db#(virtual apb_if)::get(this,"","vif", vif)) //uvm_test_top.agent.mon.aif
         `uvm_error("MON", "Unable to access interface");
    endfunction

    //run phase to receive data from DUT
    virtual task run_phase(uvm_phase phase);
        forever  //forever block used because monitor needs to be ready all the time to get response from DUT 
            begin    
                @(posedge vif.pclk);
                if(!vif.presetn) //rst 
                    begin
//tr.op is not a signal to or from the DUT , so we have to overwrite or handle it ourselve 
                        tr.op = rst; //overwrite op to rst as resetn is applied
                        `uvm_info("MON", "SYSTEM RESET DETECTED", UVM_NONE);
                        send.write(tr); //calling write fucntion inside scoreboard
                    end
                else if(vif.presetn && vif.pwrite) //write operation coming from DUT
                    begin
                        @(negedge vif.pready) //S to M
                            tr.op = writeData; //have to set by self
                            tr.PWDATA = vif.pwdata;
                            tr.PADDR = vif.paddr;
                            tr.PSLVERR = vif.pslverr;
                            `uvm_info("MON", $sformatf("DATA WRITE addr:%0d, wdata:%0d, slverr:%0d", tr.PADDR, tr.PWDATA, tr.PSLVERR), UVM_NONE);
                            send.write(tr); //send to scoreboard
                    end 
                else if(vif.presetn && !vif.pwrite) //read operation data coming from DUT
                    begin
                        @(negedge vif.pready) //S to M
                            tr.op = readData; //have to set by self
                            tr.PADDR = vif.paddr;
                            tr.PRDATA = vif.prdata;
                            tr.PSLVERR = vif.pslverr;
                            `uvm_info("MON", $sformatf("DATA READ addr:%0d, rdata: %0d, slverr:%0d", tr.PADDR, tr.PRDATA, tr.PSLVERR), UVM_NONE);
                            send.write(tr); //send to scoreboard
                    end
           end
    endtask
    
endclass
///////////////////////////////////////////////////////////////////////////////////////////////
//6.SCOREBOARD
class sco extends uvm_scoreboard;
    `uvm_component_utils(sco)

    uvm_analysis_imp #(transaction, sco) recv; //port to connect mon and sco
    //temp variables to store the data from mon
    bit [31:0] arr[32] = '{default:0}; //arr[tr.PADDR] to store tr.PWDATA of write operation
    bit [31:0] addr = 0;
    bit [31:0] data_read = 0;

    //constructor 
    function new(input string path = "sco", uvm_component parent = null); //2 args as uvm_component
        super.new(path, parent);
    endfunction 

    //build phase
    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        recv = new("recv", this); //2 args
    endfunction

    //write function to get packet form monitor
    virtual function void write(input transaction tr);
        if(tr.op == rst) //check for rst operation
            begin
                `uvm_info("SCO", "SYSTEM RESET DETECTED", UVM_NONE);
            end
        else if(tr.op == writeData)
            begin
                if(tr.PSLVERR == 1'b1)
                    begin
                        `uvm_info("SCO", "SLV ERR during WRITE OP", UVM_NONE);
                    end
                else
                    begin
                        //store the data written into mem into a temp variable 32-bit arr[32]
                        arr[tr.PADDR] = tr.PWDATA; 
                        `uvm_info("SCO", $sformatf("DATA WRITE OP addr:%0d, wdata:%0d, arr_wr:%0d", tr.PADDR, tr.PWDATA, arr[tr.PADDR]), UVM_NONE);
                    end
            end
        else if(tr.op == readData)
            begin       
                if(tr.PSLVERR == 1'b1)
                    begin
                        `uvm_info("SCO", "SLV ERR during READ OP", UVM_NONE);
                    end    
                else  //check if read operation gives correct data from memory
                    begin
//store the data from the temp memory that we stored during Write oper and compare it to the rdata that the mon got from the DUT
                        data_read = arr[tr.PADDR]; 
                        if(data_read == tr.PRDATA)
                            `uvm_info("SCO", $sformatf("DATA MATCHED : addr: %0d, rdata:%0d data_read_arr:%0d", tr.PADDR, tr.PRDATA, data_read), UVM_NONE)
                        else 
                            `uvm_info("SCO", $sformatf("TEST FAILED : addr: %0d, rdata: %0d data_read_addr:%0d", tr.PADDR, tr.PRDATA, data_read), UVM_NONE)
                     end
            end
    $display("-------------------------------------------------------------------------------------");
    endfunction
endclass
///////////////////////////////////////////////////////////////////////////////////////////////
//8.AGENT 
class agent extends uvm_agent;
    `uvm_component_utils(agent)

    apb_config cfg; //instance to get acces of configuration

    //constructor new()
    function new(input string path = "agent", uvm_component parent = null);
        super.new(path, parent);
    endfunction
//agent contains sequencer , driver and monitor 
    uvm_sequencer#(transaction) seqr;
    driver d;
    mon m;

    //build_phase
    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        cfg = apb_config::type_id::create("cfg"); //1 arg as belongs to uvm_config
        m = mon::type_id::create("m", this);

        //check if is_active = UVM_ACTIVE then only create active components
        if(cfg.is_active == UVM_ACTIVE) 
            begin
                d = driver::type_id::create("d", this);
                seqr = uvm_sequencer#(transaction)::type_id::create("seqr", this); //2 args as uvm_component    
            end    
    endfunction

    //connect phase 
    virtual function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        if(cfg.is_active == UVM_ACTIVE) begin
            d.seq_item_port.connect(seqr.seq_item_export); //connecting drv and seqr
        end
    endfunction
endclass
////////////////////////////////////////////////////////////////////////////////////////////////////
//9.ENV
class env extends uvm_env;
    `uvm_component_utils(env)

    //env contains agent and scoreboard
    agent a;
    sco s;
    
    //constructor new()
    function new(input string path = "env", uvm_component parent = null);
        super.new(path, parent);
    endfunction

    //build_phase
    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        a = agent::type_id::create("a", this); //2 args
        s = sco::type_id::create("s", this); //2 args   
    endfunction

    //connect phase 
    virtual function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        a.m.send.connect(s.recv); //connecting mon and sco inside environment     
    endfunction
endclass
/////////////////////////////////////////////////////////////////////////////////////////////////
//10.TEST
class test extends uvm_test;
    `uvm_component_utils(test)

    //test contains env and we start the sequences from test 
    env e;
    //sequence instances
    write_data wdata;
    read_data rdata;
    write_read wrrd;
    writeb_readb wrrdb;
    write_err werr;
    read_err rerr;
    reset_dut rstdut;

    //constructor new()
    function new(input string path = "agent", uvm_component parent = null);
        super.new(path, parent);
    endfunction

    //build_phase
    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        e = env::type_id::create("e", this); //2 args as uvm_component
        //sequences
        wdata = write_data::type_id::create("wdata");
        rdata = read_data::type_id::create("rdata");
        wrrd = write_read::type_id::create("wrrd");
        wrrdb = writeb_readb::type_id::create("wrrdb");
        werr = write_err::type_id::create("werr");
        rerr = read_err::type_id::create("reer");
        rstdut = reset_dut::type_id::create("rstdut");
    endfunction

    //run phase 
    virtual task run_phase(uvm_phase phase);
        phase.raise_objection(this);  //to hold the simulator 
            werr.start(e.a.seqr);
            #20;
        phase.drop_objection(this);   
    endtask
endclass
/////////////////////////////////////////////////////////////////////////////////////////////////////
//11.TB_TOP
module tb;

    //interface instance
    apb_if vif(); //need to use () for interface inside tb
    //DUT instance
    apb_ram dut(
        //M to S
        .presetn(vif.presetn),
        .pclk(vif.pclk),
        .psel(vif.psel),
        .pwrite(vif.pwrite),
        .paddr(vif.paddr),
        .pwdata(vif.pwdata),
        .penable(vif.penable),
        //S to M
        .pready(vif.pready),
        .prdata(vif.prdata),
        .pslverr(vif.pslverr)
    );
    //initialization
    initial 
        begin
            vif.pclk <= 0;
        end 
    //clk generation
    always #10 vif.pclk = ~vif.pclk; //50MHz
    
    //no need to create test instance, use run_test
    initial 
        begin
            //give intf access to driver and monitor
            uvm_config_db#(virtual apb_if)::set(null, "*", "vif", vif);
            run_test("test");
        end

    //for waveform
    initial
        begin
            $dumpfile("dump.vcd");
            $dumpvars;
        end
        
endmodule
```
</details>

__________________________________________________________________

<details><summary>Simulation</summary>

### Write and Read in Bulk 

![alt text](<sim/wrrd_bulk seq P1.png>)

![alt text](<sim/wrrd_bulk seq P2.png>)

### Write Error

![alt text](<sim/werr seq .png>)


</details>

__________________________________________________________________