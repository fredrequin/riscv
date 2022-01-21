`timescale 1 ns / 1 ps
//-----------------------------------------------------------------
//                         RISC-V Top
//                            V0.6
//                     Ultra-Embedded.com
//                     Copyright 2014-2019
//
//                   admin@ultra-embedded.com
//
//                       License: BSD
//-----------------------------------------------------------------
//
// Copyright (c) 2014, Ultra-Embedded.com
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions 
// are met:
//   - Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   - Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer 
//     in the documentation and/or other materials provided with the 
//     distribution.
//   - Neither the name of the author nor the names of its contributors 
//     may be used to endorse or promote products derived from this 
//     software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR 
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF 
// THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
// SUCH DAMAGE.
//-----------------------------------------------------------------

//-----------------------------------------------------------------
//                          Generated File
//-----------------------------------------------------------------
module riscv_tcm_top
//-----------------------------------------------------------------
// Params
//-----------------------------------------------------------------
#(
     parameter BOOT_VECTOR        = 32'h00002000
    ,parameter CORE_ID            = 0
    ,parameter TCM_MEM_BASE       = 0
    ,parameter MEM_CACHE_ADDR_MIN = 0
    ,parameter MEM_CACHE_ADDR_MAX = 32'hffffffff
    ,parameter MEM_INIT_FILE      = "NONE"
)
//-----------------------------------------------------------------
// Ports
//-----------------------------------------------------------------
(
    // Inputs
    input           clk_i,
    input           rst_i,
    input           rst_cpu_i,
    input           axi_i_awready_i,
    input           axi_i_wready_i,
    input           axi_i_bvalid_i,
    input  [  1:0]  axi_i_bresp_i,
    input           axi_i_arready_i,
    input           axi_i_rvalid_i,
    input  [ 31:0]  axi_i_rdata_i,
    input  [  1:0]  axi_i_rresp_i,
    input           axi_t_awvalid_i,
    input  [ 31:0]  axi_t_awaddr_i,
    input  [  3:0]  axi_t_awid_i,
    input  [  7:0]  axi_t_awlen_i,
    input  [  1:0]  axi_t_awburst_i,
    input           axi_t_wvalid_i,
    input  [ 31:0]  axi_t_wdata_i,
    input  [  3:0]  axi_t_wstrb_i,
    input           axi_t_wlast_i,
    input           axi_t_bready_i,
    input           axi_t_arvalid_i,
    input  [ 31:0]  axi_t_araddr_i,
    input  [  3:0]  axi_t_arid_i,
    input  [  7:0]  axi_t_arlen_i,
    input  [  1:0]  axi_t_arburst_i,
    input           axi_t_rready_i,
    input  [ 31:0]  intr_i,

    // Outputs
    output          axi_i_awvalid_o,
    output [ 31:0]  axi_i_awaddr_o,
    output          axi_i_wvalid_o,
    output [ 31:0]  axi_i_wdata_o,
    output [  3:0]  axi_i_wstrb_o,
    output          axi_i_bready_o,
    output          axi_i_arvalid_o,
    output [ 31:0]  axi_i_araddr_o,
    output          axi_i_rready_o,
    output          axi_t_awready_o,
    output          axi_t_wready_o,
    output          axi_t_bvalid_o,
    output [  1:0]  axi_t_bresp_o,
    output [  3:0]  axi_t_bid_o,
    output          axi_t_arready_o,
    output          axi_t_rvalid_o,
    output [ 31:0]  axi_t_rdata_o,
    output [  1:0]  axi_t_rresp_o,
    output [  3:0]  axi_t_rid_o,
    output          axi_t_rlast_o
);

wire  [ 31:0]  w_ifetch_pc;
wire           w_ifetch_rd;
wire           w_ifetch_flush;
wire           w_ifetch_invalidate;
wire  [ 31:0]  w_ifetch_inst;
wire           w_ifetch_valid;
wire           w_ifetch_accept;
wire           w_ifetch_error;

wire  [ 31:0]  w_dport_addr;
wire  [ 31:0]  w_dport_data_wr;
wire           w_dport_rd;
wire  [  3:0]  w_dport_wr;
wire           w_dport_cacheable;
wire  [ 10:0]  w_dport_req_tag;
wire           w_dport_invalidate;
wire           w_dport_writeback;
wire           w_dport_flush;
wire  [ 31:0]  w_dport_data_rd;
wire           w_dport_accept;
wire           w_dport_ack;
wire           w_dport_error;
wire  [ 10:0]  w_dport_resp_tag;


wire  [ 31:0]  w_dport_tcm_addr;
wire  [ 31:0]  w_dport_tcm_data_wr;
wire           w_dport_tcm_rd;
wire  [  3:0]  w_dport_tcm_wr;
wire           w_dport_tcm_cacheable;
wire  [ 10:0]  w_dport_tcm_req_tag;
wire           w_dport_tcm_invalidate;
wire           w_dport_tcm_writeback;
wire           w_dport_tcm_flush;
wire  [ 31:0]  w_dport_tcm_data_rd;
wire           w_dport_tcm_accept;
wire           w_dport_tcm_ack;
wire           w_dport_tcm_error;
wire  [ 10:0]  w_dport_tcm_resp_tag;

wire           w_dport_axi_accept;
wire  [ 10:0]  w_dport_axi_resp_tag;
wire  [ 10:0]  w_dport_axi_req_tag;
wire           w_dport_axi_error;
wire  [ 31:0]  w_cpu_id = CORE_ID;
wire           w_dport_axi_ack;
wire           w_dport_axi_rd;
wire  [ 31:0]  w_dport_axi_data_rd;
wire           w_dport_axi_invalidate;
wire  [ 31:0]  w_boot_vector = BOOT_VECTOR;
wire  [ 31:0]  w_dport_axi_addr;
wire           w_dport_axi_writeback;
wire  [ 31:0]  w_dport_axi_data_wr;
wire           w_dport_axi_cacheable;
wire  [  3:0]  w_dport_axi_wr;
wire           w_dport_axi_flush;

// RISC-V core
riscv_core
#(
    .MEM_CACHE_ADDR_MIN   (MEM_CACHE_ADDR_MIN),
    .MEM_CACHE_ADDR_MAX   (MEM_CACHE_ADDR_MAX)
)
u_core
(
    // Clock and reset
    .rst_i                (rst_cpu_i),
    .clk_i                (clk_i),
    // Misc.
    .reset_vector_i       (w_boot_vector),
    .cpu_id_i             (w_cpu_id),
    .intr_i               (intr_i[0:0]),
    // Instruction fetch
    .mem_i_pc_o           (w_ifetch_pc),
    .mem_i_rd_o           (w_ifetch_rd),
    .mem_i_flush_o        (w_ifetch_flush),
    .mem_i_invalidate_o   (w_ifetch_invalidate),
    .mem_i_inst_i         (w_ifetch_inst),
    .mem_i_valid_i        (w_ifetch_valid),
    .mem_i_accept_i       (w_ifetch_accept),
    .mem_i_error_i        (w_ifetch_error),
    // Data access
    .mem_d_addr_o         (w_dport_addr),
    .mem_d_data_wr_o      (w_dport_data_wr),
    .mem_d_rd_o           (w_dport_rd),
    .mem_d_wr_o           (w_dport_wr),
    .mem_d_cacheable_o    (w_dport_cacheable),
    .mem_d_req_tag_o      (w_dport_req_tag),
    .mem_d_invalidate_o   (w_dport_invalidate),
    .mem_d_writeback_o    (w_dport_writeback),
    .mem_d_flush_o        (w_dport_flush),
    .mem_d_data_rd_i      (w_dport_data_rd),
    .mem_d_accept_i       (w_dport_accept),
    .mem_d_ack_i          (w_dport_ack),
    .mem_d_error_i        (w_dport_error),
    .mem_d_resp_tag_i     (w_dport_resp_tag)
);


dport_mux
#(
    .TCM_MEM_BASE         (TCM_MEM_BASE)
)
u_dmux
(
    // Clock and reset
    .rst_i                (rst_cpu_i),
    .clk_i                (clk_i),
    // Data access : CPU side
    .mem_addr_i           (w_dport_addr),
    .mem_data_wr_i        (w_dport_data_wr),
    .mem_rd_i             (w_dport_rd),
    .mem_wr_i             (w_dport_wr),
    .mem_cacheable_i      (w_dport_cacheable),
    .mem_req_tag_i        (w_dport_req_tag),
    .mem_invalidate_i     (w_dport_invalidate),
    .mem_writeback_i      (w_dport_writeback),
    .mem_flush_i          (w_dport_flush),
    .mem_data_rd_o        (w_dport_data_rd),
    .mem_accept_o         (w_dport_accept),
    .mem_ack_o            (w_dport_ack),
    .mem_error_o          (w_dport_error),
    .mem_resp_tag_o       (w_dport_resp_tag),
    // Data access : TCM side
    .mem_tcm_addr_o       (w_dport_tcm_addr),
    .mem_tcm_data_wr_o    (w_dport_tcm_data_wr),
    .mem_tcm_rd_o         (w_dport_tcm_rd),
    .mem_tcm_wr_o         (w_dport_tcm_wr),
    .mem_tcm_cacheable_o  (w_dport_tcm_cacheable),
    .mem_tcm_req_tag_o    (w_dport_tcm_req_tag),
    .mem_tcm_invalidate_o (w_dport_tcm_invalidate),
    .mem_tcm_writeback_o  (w_dport_tcm_writeback),
    .mem_tcm_flush_o      (w_dport_tcm_flush),
    .mem_tcm_data_rd_i    (w_dport_tcm_data_rd),
    .mem_tcm_accept_i     (w_dport_tcm_accept),
    .mem_tcm_ack_i        (w_dport_tcm_ack),
    .mem_tcm_error_i      (w_dport_tcm_error),
    .mem_tcm_resp_tag_i   (w_dport_tcm_resp_tag),
    // Data access : AXI side
    .mem_ext_addr_o       (w_dport_axi_addr),
    .mem_ext_data_wr_o    (w_dport_axi_data_wr),
    .mem_ext_rd_o         (w_dport_axi_rd),
    .mem_ext_wr_o         (w_dport_axi_wr),
    .mem_ext_cacheable_o  (w_dport_axi_cacheable),
    .mem_ext_req_tag_o    (w_dport_axi_req_tag),
    .mem_ext_invalidate_o (w_dport_axi_invalidate),
    .mem_ext_writeback_o  (w_dport_axi_writeback),
    .mem_ext_flush_o      (w_dport_axi_flush),
    .mem_ext_data_rd_i    (w_dport_axi_data_rd),
    .mem_ext_accept_i     (w_dport_axi_accept),
    .mem_ext_ack_i        (w_dport_axi_ack),
    .mem_ext_error_i      (w_dport_axi_error),
    .mem_ext_resp_tag_i   (w_dport_axi_resp_tag)
);

// Tightly Coupled Memory
tcm_mem
#(
    .MEM_INIT_FILE (MEM_INIT_FILE)
)
u_tcm
(
    // Clock and reset
    .rst_i                (rst_cpu_i),
    .clk_i                (clk_i),
    // Instruction fetch
    .mem_i_pc_i           (w_ifetch_pc),
    .mem_i_rd_i           (w_ifetch_rd),
    .mem_i_flush_i        (w_ifetch_flush),
    .mem_i_invalidate_i   (w_ifetch_invalidate),
    .mem_i_accept_o       (w_ifetch_accept),
    .mem_i_valid_o        (w_ifetch_valid),
    .mem_i_error_o        (w_ifetch_error),
    .mem_i_inst_o         (w_ifetch_inst),
    // Data access
    .mem_d_addr_i         (w_dport_tcm_addr),
    .mem_d_data_wr_i      (w_dport_tcm_data_wr),
    .mem_d_rd_i           (w_dport_tcm_rd),
    .mem_d_wr_i           (w_dport_tcm_wr),
    .mem_d_cacheable_i    (w_dport_tcm_cacheable),
    .mem_d_req_tag_i      (w_dport_tcm_req_tag),
    .mem_d_invalidate_i   (w_dport_tcm_invalidate),
    .mem_d_writeback_i    (w_dport_tcm_writeback),
    .mem_d_flush_i        (w_dport_tcm_flush),
    .mem_d_data_rd_o      (w_dport_tcm_data_rd),
    .mem_d_accept_o       (w_dport_tcm_accept),
    .mem_d_ack_o          (w_dport_tcm_ack),
    .mem_d_error_o        (w_dport_tcm_error),
    .mem_d_resp_tag_o     (w_dport_tcm_resp_tag),
    // AXI slave access
    .axi_awvalid_i        (axi_t_awvalid_i),
    .axi_awaddr_i         (axi_t_awaddr_i),
    .axi_awid_i           (axi_t_awid_i),
    .axi_awlen_i          (axi_t_awlen_i),
    .axi_awburst_i        (axi_t_awburst_i),
    .axi_wvalid_i         (axi_t_wvalid_i),
    .axi_wdata_i          (axi_t_wdata_i),
    .axi_wstrb_i          (axi_t_wstrb_i),
    .axi_wlast_i          (axi_t_wlast_i),
    .axi_bready_i         (axi_t_bready_i),
    .axi_arvalid_i        (axi_t_arvalid_i),
    .axi_araddr_i         (axi_t_araddr_i),
    .axi_arid_i           (axi_t_arid_i),
    .axi_arlen_i          (axi_t_arlen_i),
    .axi_arburst_i        (axi_t_arburst_i),
    .axi_rready_i         (axi_t_rready_i),
    .axi_awready_o        (axi_t_awready_o),
    .axi_wready_o         (axi_t_wready_o),
    .axi_bvalid_o         (axi_t_bvalid_o),
    .axi_bresp_o          (axi_t_bresp_o),
    .axi_bid_o            (axi_t_bid_o),
    .axi_arready_o        (axi_t_arready_o),
    .axi_rvalid_o         (axi_t_rvalid_o),
    .axi_rdata_o          (axi_t_rdata_o),
    .axi_rresp_o          (axi_t_rresp_o),
    .axi_rid_o            (axi_t_rid_o),
    .axi_rlast_o          (axi_t_rlast_o)
);

// AXI-4 master interface for CPU data access
dport_axi
u_axi
(
    // Clock and reset
    .rst_i                (rst_cpu_i),
    .clk_i                (clk_i),
    // Data access : CPU side
    .mem_addr_i           (w_dport_axi_addr),
    .mem_data_wr_i        (w_dport_axi_data_wr),
    .mem_rd_i             (w_dport_axi_rd),
    .mem_wr_i             (w_dport_axi_wr),
    .mem_cacheable_i      (w_dport_axi_cacheable),
    .mem_req_tag_i        (w_dport_axi_req_tag),
    .mem_invalidate_i     (w_dport_axi_invalidate),
    .mem_writeback_i      (w_dport_axi_writeback),
    .mem_flush_i          (w_dport_axi_flush),
    .mem_data_rd_o        (w_dport_axi_data_rd),
    .mem_accept_o         (w_dport_axi_accept),
    .mem_ack_o            (w_dport_axi_ack),
    .mem_error_o          (w_dport_axi_error),
    .mem_resp_tag_o       (w_dport_axi_resp_tag),
    // Data access : AXI master side
    .axi_awvalid_o        (axi_i_awvalid_o),
    .axi_awaddr_o         (axi_i_awaddr_o),
    .axi_wvalid_o         (axi_i_wvalid_o),
    .axi_wdata_o          (axi_i_wdata_o),
    .axi_wstrb_o          (axi_i_wstrb_o),
    .axi_bready_o         (axi_i_bready_o),
    .axi_arvalid_o        (axi_i_arvalid_o),
    .axi_araddr_o         (axi_i_araddr_o),
    .axi_rready_o         (axi_i_rready_o),
    .axi_awready_i        (axi_i_awready_i),
    .axi_wready_i         (axi_i_wready_i),
    .axi_bvalid_i         (axi_i_bvalid_i),
    .axi_bresp_i          (axi_i_bresp_i),
    .axi_arready_i        (axi_i_arready_i),
    .axi_rvalid_i         (axi_i_rvalid_i),
    .axi_rdata_i          (axi_i_rdata_i),
    .axi_rresp_i          (axi_i_rresp_i)
);

`ifdef verilator3
    always @ (posedge clk_i) begin : UART_DEBUG
    
        // Last word of the TCM is the debug output
        if (w_dport_addr == 32'h0001FFFC) begin
            // ASCII output
            if (w_dport_wr == 4'b0001) begin
                $write("%c", w_dport_data_wr[7:0]);
            end
            // Hexadecimal output
            if (w_dport_wr == 4'b0011) begin
                $write("%x", w_dport_data_wr[15:0]);
            end
            if (w_dport_wr == 4'b1111) begin
                $write("%x", w_dport_data_wr[31:0]);
            end
        end
    end
`endif

endmodule
