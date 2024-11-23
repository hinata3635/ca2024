/********************************************************************************************/
/* RVCore_Base, RISC-V RV32I (without some LD/ST) v2024-11-09a       ArchLab, Science Tokyo */
/********************************************************************************************/
`default_nettype none

// `define TRACE_VCD
`define READ_DELAY   5
`define CLK_FREQ_MHZ 40
`define IMEM_SIZE    (1024*4)  // 4KB (1024 word) memory
`define PROGRAM      "sample1.mem"
`define MAX_INSN     100000000

/********************************************************************************************/
module vio_0  (
    input wire w_clk,
    input wire [31:0] input1
);
endmodule

/********************************************************************************************/
module top;
    reg r_clk = 0; always  #6 r_clk <= ~r_clk;

    reg [31:0] r_cc = 0;
    always @(posedge r_clk) r_cc <= r_cc + 1;

    reg r_rst_n = 0;
    always @(posedge r_clk) if (r_cc==10) r_rst_n = 1;

`ifdef TRACE_VCD
    initial begin
        $dumpfile("dump.vcd");
        $dumpvars(0);
    end
`endif

    // Send program using UART
    reg [31:0] rom [0:(`IMEM_SIZE/4)-1];
    initial begin
        $readmemh(`PROGRAM, rom);
    end

    reg                      [31:0] r_wait_cnt = 0                               ;
    reg                             r_valid    = 1'b0                            ;
    reg                      [31:0] r_addr     = 0                               ;
    reg                       [7:0] r_data     = 8'h0                            ;
    wire [$clog2(`IMEM_SIZE/4)-1:0] w_index    = r_addr[$clog2(`IMEM_SIZE/4)+1:2];
    wire                      [1:0] w_offset   = r_addr[1:0]                     ;
    always @(posedge r_clk) begin
        r_wait_cnt <= (r_wait_cnt==`CLK_FREQ_MHZ*11) ? 0 : r_wait_cnt+1;
        r_valid    <= (r_wait_cnt==0 && (r_addr<`IMEM_SIZE));
        r_data     <= rom[w_index][w_offset*8+:8];
        r_addr     <= (r_valid) ? r_addr+1 : r_addr;
    end

    wire w_rxd;
    wire w_txd;
    m_uart_tx uart2 (r_clk, r_valid, r_data, w_txd);

    wire  [13:0] ddr3_addr;
    wire   [2:0] ddr3_ba;
    wire         ddr3_cas_n;
    wire   [0:0] ddr3_ck_n;
    wire   [0:0] ddr3_ck_p;
    wire   [0:0] ddr3_cke;
    wire         ddr3_ras_n;
    wire         ddr3_reset_n;
    wire         ddr3_we_n;
    wire  [15:0] ddr3_dq;
    wire   [1:0] ddr3_dqs_n;
    wire   [1:0] ddr3_dqs_p;
    wire   [0:0] ddr3_cs_n;
    wire   [1:0] ddr3_dm;
    wire   [0:0] ddr3_odt;
    m_main m0 (
        .w_clk_i     (r_clk  ),
        .w_rxd_i     (w_txd  ),
        .w_txd_o     (w_rxd  ),
        .ddr3_addr   (ddr3_addr     ),
        .ddr3_ba     (ddr3_ba       ),
        .ddr3_cas_n  (ddr3_cas_n    ),
        .ddr3_ck_n   (ddr3_ck_n     ),
        .ddr3_ck_p   (ddr3_ck_p     ),
        .ddr3_cke    (ddr3_cke      ),
        .ddr3_ras_n  (ddr3_ras_n    ),
        .ddr3_reset_n(ddr3_reset_n  ),
        .ddr3_we_n   (ddr3_we_n     ),
        .ddr3_dq     (ddr3_dq       ),
        .ddr3_dqs_n  (ddr3_dqs_n    ),
        .ddr3_dqs_p  (ddr3_dqs_p    ),
        .ddr3_cs_n   (ddr3_cs_n     ),
        .ddr3_dm     (ddr3_dm       ),
        .ddr3_odt    (ddr3_odt      )
    );

    reg [31:0] r_insn = 0;
    always @(posedge r_clk) begin
        if (m0.r_init_done && m0.w_stall==0 && m0.m0.r_state==1) begin
            r_insn <= r_insn + 1;
        end
    end

    always @(posedge r_clk) begin
        if (r_insn>`MAX_INSN) begin
            $write("\n== executed max insns (%0d) and finish simulation.\n\n", `MAX_INSN);
            $finish;
        end
    end
endmodule

/********************************************************************************************/
module mig_simulator #(
    parameter DRAM_SIZE = (32*1024*1024) // 32 MiB
) (
    // Memory interface ports
    output wire  [13:0] ddr3_addr          ,
    output wire   [2:0] ddr3_ba            ,
    output wire         ddr3_cas_n         ,
    output wire   [0:0] ddr3_ck_n          ,
    output wire   [0:0] ddr3_ck_p          ,
    output wire   [0:0] ddr3_cke           ,
    output wire         ddr3_ras_n         ,
    output wire         ddr3_reset_n       ,
    output wire         ddr3_we_n          ,
    inout  wire  [15:0] ddr3_dq            ,
    inout  wire   [1:0] ddr3_dqs_n         ,
    inout  wire   [1:0] ddr3_dqs_p         ,
    output reg          init_calib_complete = 1'b0,
    output wire   [0:0] ddr3_cs_n          ,
    output wire   [1:0] ddr3_dm            ,
    output wire   [0:0] ddr3_odt           ,
    // Application interface ports
    input  wire  [27:0] app_addr_i         ,
    input  wire   [2:0] app_cmd_i          , // 3'b000 -> Write, 3'b001 -> Read
    input  wire         app_en_i           ,
    input  wire [127:0] app_wdf_data_i     ,
    input  wire         app_wdf_end_i      ,
    input  wire         app_wdf_wren_i     ,
    output reg  [127:0] app_rd_data_o       = 0   ,
    output reg          app_rd_data_end_o   = 1'b0,
    output reg          app_rd_data_valid_o = 1'b0,
    output reg          app_rdy_o           = 1'b0,
    output reg          app_wdf_rdy_o       = 1'b0,
    input  wire         app_sr_req_i       ,
    input  wire         app_ref_req_i      ,
    input  wire         app_zq_req_i       ,
    output wire         app_sr_active_o    ,
    output wire         app_ref_ack_o      ,
    output wire         app_zq_ack_o       ,
    output wire         ui_clk_o           , //  80 MHz
    output wire         ui_clk_sync_rst_o  ,
    input  wire  [15:0] app_wdf_mask_i     ,
    // System Clock Ports
    input  wire         sys_clk_i          , // 160 MHz
    // Reference Clock Ports
    input  wire         clk_ref_i          , // 200 MHz
    input  wire         sys_rst_i
);

    reg clk_80_mhz = 0; always #6 clk_80_mhz <= ~clk_80_mhz;
    assign ui_clk_o = clk_80_mhz;

    reg rst1, rst2;
    always @(posedge ui_clk_o or posedge sys_rst_i) begin
        if (sys_rst_i) begin
            rst1 <= 1'b1;
            rst2 <= 1'b1;
        end else begin
            rst1 <= 1'b0;
            rst2 <= rst1;
        end
    end
    assign ui_clk_sync_rst_o = rst2;

    // DRAM
    reg [127:0] dram [0:(DRAM_SIZE/16)-1];
    integer i; initial for (i=0; i<(DRAM_SIZE/16); i=i+1) dram[i]=0;

    localparam VALID_ADDR_WIDTH = $clog2(DRAM_SIZE/16)-3;
    wire [VALID_ADDR_WIDTH-1:0] valid_addr = {2'b0, app_addr_i[VALID_ADDR_WIDTH:3]};

    // CMD
    reg  [VALID_ADDR_WIDTH-1:0] app_addr = 0, n_app_addr           ;
    reg                                       n_app_rd_data_end_o  ;
    reg                                       n_app_rd_data_valid_o;
    reg                                       n_app_rdy_o          ;

    // CMD FSM
    localparam CMD_IDLE  = 2'b00;
    localparam CMD_WRITE = 2'b01;
    localparam CMD_READ  = 2'b10;
    reg [1:0]  cmd_state = CMD_IDLE, n_cmd_state;

    reg [7:0] read_wait_cntr = 0, n_read_wait_cntr;

    always @(*) begin
        n_app_addr            = app_addr        ;
        n_app_rd_data_end_o   = 1'b0            ;
        n_app_rd_data_valid_o = 1'b0            ;
        n_app_rdy_o           = 1'b0            ;
        n_read_wait_cntr      = read_wait_cntr+1;
        n_cmd_state           = cmd_state       ;
        casez (cmd_state)
            CMD_IDLE: begin
                if (app_en_i) begin
                    if (app_cmd_i==3'b000) begin // write
                        n_app_addr  = valid_addr;
                        n_app_rdy_o = 1'b1      ;
                        n_cmd_state = CMD_WRITE ;
                    end
                    if (app_cmd_i==3'b001) begin // read
                        n_app_addr       = valid_addr;
                        n_app_rdy_o      = 1'b1      ;
                        n_read_wait_cntr = 0         ;
                        n_cmd_state      = CMD_READ  ;
                    end
                end
            end
            CMD_WRITE: begin
                if (wdf_state==WDF_WDATA) begin
                    n_cmd_state = CMD_IDLE;
                end
            end
            CMD_READ : begin
                if (read_wait_cntr==`READ_DELAY-1) begin
                    n_app_rd_data_end_o   = 1'b1    ;
                    n_app_rd_data_valid_o = 1'b1    ;
                    n_cmd_state           = CMD_IDLE;
                end
            end
            default  : begin
                n_cmd_state = CMD_IDLE;
            end
        endcase
    end

    always @(posedge ui_clk_o) begin
        if (ui_clk_sync_rst_o) begin
            app_rd_data_end_o   <= 1'b0    ;
            app_rd_data_valid_o <= 1'b0    ;
            app_rdy_o           <= 1'b0    ;
            cmd_state           <= CMD_IDLE;
        end else begin
            app_addr            <= n_app_addr           ;
            app_rd_data_end_o   <= n_app_rd_data_end_o  ;
            app_rd_data_valid_o <= n_app_rd_data_valid_o;
            app_rdy_o           <= n_app_rdy_o          ;
            read_wait_cntr      <= n_read_wait_cntr     ;
            cmd_state           <= n_cmd_state          ;
        end
    end

    // WDF
    reg  [127:0] app_wdf_data = 0, n_app_wdf_data ;
    reg                            n_app_wdf_rdy_o;
    reg   [15:0] app_wdf_mask = 0, n_app_wdf_mask ;

    // WDF FSM
    localparam WDF_IDLE  = 1'b0;
    localparam WDF_WDATA = 1'b1;
    reg        wdf_state = WDF_IDLE, n_wdf_state;

    always @(*) begin
        n_app_wdf_data  = app_wdf_data;
        n_app_wdf_rdy_o = 1'b0        ;
        n_app_wdf_mask  = app_wdf_mask;
        n_wdf_state     = wdf_state   ;
        casez (wdf_state)
            WDF_IDLE : begin
                if (app_wdf_wren_i && app_wdf_end_i) begin
                    n_app_wdf_data  = app_wdf_data_i;
                    n_app_wdf_rdy_o = 1'b1          ;
                    n_app_wdf_mask  = app_wdf_mask_i;
                    n_wdf_state     = WDF_WDATA     ;
                end
            end
            WDF_WDATA: begin
                if (cmd_state==CMD_WRITE) begin
                    n_wdf_state = WDF_IDLE;
                end
            end
            default  : begin
                n_wdf_state = WDF_IDLE;
            end
        endcase
    end

    always @(posedge ui_clk_o) begin
        if (ui_clk_sync_rst_o) begin
            app_wdf_rdy_o <= 1'b0    ;
            wdf_state     <= WDF_IDLE;
        end else begin
            app_wdf_data  <= n_app_wdf_data ;
            app_wdf_rdy_o <= n_app_wdf_rdy_o;
            app_wdf_mask  <= n_app_wdf_mask ;
            wdf_state     <= n_wdf_state    ;
        end
    end

    // DRAM Read/Write
    always @(posedge ui_clk_o) begin
        if ((cmd_state==CMD_READ) && (read_wait_cntr==`READ_DELAY-1)) begin
            app_rd_data_o <= dram[app_addr];
        end
        if ((cmd_state==CMD_WRITE) && (wdf_state==WDF_WDATA)) begin
            for (i=0; i<16; i=i+1) begin
                if (!app_wdf_mask[i]) dram[app_addr][i*8+:8] <= app_wdf_data[i*8+:8];
            end
        end
    end

    assign app_sr_active_o = app_sr_req_i ;
    assign app_ref_ack_o   = app_ref_req_i;
    assign app_zq_ack_o    = app_zq_req_i ;

    always @(posedge ui_clk_o) begin
        if (ui_clk_sync_rst_o) begin
            init_calib_complete <= 1'b0;
        end else begin
            init_calib_complete <= 1'b1;
        end
    end

endmodule
/********************************************************************************************/
