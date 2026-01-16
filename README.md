# AMBA APB Protocol: Technical Specification

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

---

