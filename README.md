# 🚀 SPI to AXI4-Lite Bridge on FPGA

> Protocol bridge converting SPI transactions into AXI4-Lite operations, implemented and validated on Nexys A7 FPGA.

---

## 🎯 Overview

This project implements a complete data path inside FPGA:

SPI Master → SPI Slave → AXI4-Lite Master → AXI4-Lite Slave → 7-Segment Display

- SPI generates read/write commands  
- Bridge converts SPI → AXI transactions  
- AXI slave stores data in registers  
- Output is displayed on LEDs and 7-segment  

---

## ⚙️ Key Features

- SPI (Mode 0) to AXI4-Lite protocol conversion  
- 8-bit to 32-bit data width adaptation  
- Address alignment using addr << 2  
- FSM-based control logic for SPI and AXI  
- AXI VALID/READY handshake implementation  
- 8 × 32-bit memory-mapped register file  
- Implemented on Nexys A7 FPGA  

---

## 🏗️ Architecture

SPI Master → SPI Slave → AXI Master → AXI Slave → Display

---

## 🔄 Data Flow

Write Operation:  
SPI → Decode → AXI Write → Register Update  

Read Operation:  
SPI → Decode → AXI Read → Data returned via MISO  

---

## 📂 File Structure

rtl/
  spi_master.v  
  spi_slave.v  
  axi4_lite_master.v  
  axi4_lite_slave.v  
  seg7_driver.v  
  top_spi_axi.v  

tb/
  tb_top_spi_axi.v  

constraints/
  nexys_a7.xdc 

## 🔧 FPGA Implementation

- Board: Nexys A7-100T  
- Tool: Vivado  
- Clock: 100 MHz  

Steps:
1. Add RTL files and constraints  
2. Run synthesis and implementation  
3. Generate bitstream  
4. Program FPGA  

---

## 👀 Output

- LEDs indicate transaction status  
- 7-segment display shows data value  
- Successful read/write verification on hardware  

---

## 💡 Applications

- SoC protocol bridging  
- Embedded systems  
- Peripheral interfacing  

---

## 📌 Summary

This project demonstrates protocol conversion from SPI to AXI4-Lite with complete hardware validation on FPGA.
