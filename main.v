/********************************************************************************************/
/* RVCore_Base, RISC-V RV32I (without some LD/ST) v2024-11-14a       ArchLab, Science Tokyo */
/********************************************************************************************/
`default_nettype none
`define UART_CNT     40        // UART wait count, 40MHz / 40 = 1Mbaud
`define IMEM_SIZE    (1024*4)  // 4KB (1024 word) memory

/********************************************************************************************/
module m_main (
    input  wire         w_clk_i     ,
    input  wire         w_rxd_i     ,
    output wire         w_txd_o     ,
    // Memory interface ports
    output wire  [13:0] ddr3_addr   ,
    output wire   [2:0] ddr3_ba     ,
    output wire         ddr3_cas_n  ,
    output wire   [0:0] ddr3_ck_n   ,
    output wire   [0:0] ddr3_ck_p   ,
    output wire   [0:0] ddr3_cke    ,
    output wire         ddr3_ras_n  ,
    output wire         ddr3_reset_n,
    output wire         ddr3_we_n   ,
    inout  wire  [15:0] ddr3_dq     ,
    inout  wire   [1:0] ddr3_dqs_n  ,
    inout  wire   [1:0] ddr3_dqs_p  ,
    output wire   [0:0] ddr3_cs_n   ,
    output wire   [1:0] ddr3_dm     ,
    output wire   [0:0] ddr3_odt
);
    wire w_u_clk;

    /**** Program loader ****/
    /****************************************************************************************/
    wire       w_valid;
    wire [7:0] w_data ;
    m_uart_rx uart_rx1 (w_u_clk, w_rxd_i, w_data, w_valid);

    reg  [31:0] r_byte_cnt       = 0;
    reg         r_init_v    = 0;
    reg  [31:0] r_init_addr = 0;
    reg  [31:0] r_init_data = 0;
    reg         r_init_done = 0;
    always @(posedge w_u_clk) begin
        r_init_v    <= (w_valid && (r_byte_cnt[1:0]==2'b11));
        r_init_addr <= (r_init_v) ? r_init_addr+4 : r_init_addr;
        r_init_data <= (w_valid) ? {w_data, r_init_data[31:8]} : r_init_data;
        if (r_byte_cnt>=`IMEM_SIZE) begin
            r_init_done <= 1'b1;
        end
        if (w_valid) begin
            r_byte_cnt <= r_byte_cnt+1;
        end
    end

    /***** Instruction Memory *****/
    /****************************************************************************************/
    wire [31:0] w_imem_raddr;
    wire [31:0] w_imem_rdata;
    m_imem imem1 (w_u_clk, r_init_done==0 && r_init_v,
                  (r_init_done ?  w_imem_raddr : r_init_addr), r_init_data, w_imem_rdata);

    /***** Data Memory *****/
    /****************************************************************************************/
    wire        w_dmem_busy   ;
    wire [31:0] w_dmem_addr   ;
    wire        w_dmem_wvalid ;
    wire [31:0] w_dmem_wdata  ;
    wire        w_dmem_arvalid;
    wire        w_dmem_rvalid ;
    wire [31:0] w_dmem_rdata  ;
    wire        w_dram_rst_o  ;

    wire        w_dmem_we = (w_dmem_addr[31:30]==0) ? w_dmem_wvalid  : 0;
    wire        w_dmem_re = (w_dmem_addr[31:30]==0) ? w_dmem_arvalid : 0;
    wire [31:0] w_dmem_a  = w_dmem_addr;
    wire [31:0] w_dmem_d  = w_dmem_wdata;

    dram dram1 (
        .clk_i       (w_clk_i       ),
        .rst_ni      (1'b1          ),
        // User application ports
        .clk_o       (w_u_clk       ), // user clock (default 40MHz)
        .rst_o       (w_dram_rst_o  ), //
        .busy_o      (w_dmem_busy   ), //
        .addr_i      (w_dmem_a      ), // address
        .wvalid_i    (w_dmem_we     ), // write enable
        .wdata_i     (w_dmem_d      ), // write data
        .arvalid_i   (w_dmem_re     ), // read  enable
        .rvalid_o    (w_dmem_rvalid ), //
        .rdata_o     (w_dmem_rdata  ), //
        // Memory interface ports
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

    /***** processor *****/
    /****************************************************************************************/
    wire w_stall = (!r_init_done || w_dmem_busy);
    m_rvcore m0 (
        .w_clk         (w_u_clk       ),  // input  w_clk
        .w_rst         (!r_init_done  ),  //
        .w_stall       (w_stall       ),  // input  w_stall
        .I_ADDR        (w_imem_raddr  ),  // output I_ADDR
        .I_IN          (w_imem_rdata  ),  // input  I_IN
        .D_ADDR        (w_dmem_addr   ),  // output D_ADDR
        .D_WE          (w_dmem_wvalid ),  // output D_WE
        .D_OUT         (w_dmem_wdata  ),  // output D_OUT
        .D_RE          (w_dmem_arvalid),  // output D_RE
        .D_IN          (w_dmem_rdata  )   // input  D_IN
    );

    wire [1:0] tohost_cmd  = ((w_dmem_addr==32'h40008000)
                              && w_dmem_wvalid) ? w_dmem_wdata[17:16] : 0;
    wire [7:0] tohost_char = w_dmem_wdata[7:0];

    reg r_halt = 0;
    always @(posedge w_u_clk) if (r_init_done && w_stall==0) begin
        if (tohost_cmd==2) begin
            r_halt <= 1;
        end
        if (r_halt && r_head==r_tail) begin
            $write("== simulation finished.\n\n");
            $finish();
        end
    end

    /****************************************************************************************/
    reg [31:0] r_tc = 0;
    always @(posedge w_u_clk) if (r_init_done && r_halt==0) begin
        r_tc <= r_tc + 1;
    end
    vio_0 vio0 (w_u_clk, r_tc);

    /****************************************************************************************/
    reg [ 7:0] ubuf [0:4095];
    reg [11:0] r_head=0, r_tail=0;
    always @(posedge w_u_clk) if (r_init_done && w_stall==0 && tohost_cmd==1) begin
        ubuf[r_tail] <= tohost_char;
        r_tail <= r_tail + 1;
    end

    reg [31:0] r_cnt = 1;
    always @(posedge w_u_clk) r_cnt <= (r_cnt>=(`UART_CNT*12)) ? 1 : r_cnt + 1;

    wire w_uart_we = (r_init_done && (r_cnt==1) && (r_head!=r_tail));
    always @(posedge w_u_clk) if(w_uart_we) r_head <= r_head + 1;
    m_uart_tx uart_tx1 (w_u_clk, w_uart_we, ubuf[r_head], w_txd_o);

    always @(posedge w_u_clk) if (w_uart_we) $write("%c", ubuf[r_head]);
endmodule

/********************************************************************************************/
module m_imem (
    input  wire        w_clk,
    input  wire        w_we,
    input  wire [31:0] w_adr_in,
    input  wire [31:0] w_din,
    output wire [31:0] w_dout
);
    reg [31:0] mem [0:1023]; // 1024 word (4KB) memory
    wire [9:0] w_adr = w_adr_in[11:2];
    always @(posedge w_clk) if (w_we) mem[w_adr] <= w_din;
    assign w_dout = mem[w_adr];
endmodule

/********************************************************************************************/
module m_uart_tx (
    input  wire       w_clk,     // clock signal (default 40MHz)
    input  wire       w_we,      // write enable
    input  wire [7:0] w_din,     // data in
    output wire       w_uart_tx  // UART tx, data line from FPGA to PC
);
    reg [8:0] r_buf = 9'b1_1111_1111;
    reg [7:0] r_cnt = 1;
    always @(posedge w_clk) begin
        r_cnt <= (w_we) ? 1 : (r_cnt==`UART_CNT) ? 1 : r_cnt + 1;
        r_buf <= (w_we) ? {w_din, 1'b0} : (r_cnt==`UART_CNT) ? {1'b1, r_buf[8:1]} : r_buf;
    end
    assign w_uart_tx = r_buf[0];
endmodule

/********************************************************************************************/
module m_uart_rx (
    input  wire       w_clk,   // clock signal (default 40MHz)
    input  wire       w_rxd,   // UART rx, data line from PC to FPGA
    output wire [7:0] w_char,  // 8-bit data received
    output reg        r_en = 0 // data enable
);
    reg [2:0] r_detect_cnt = 0; /* to detect the start bit */
    always @(posedge w_clk) r_detect_cnt <= (w_rxd) ? 0 : r_detect_cnt + 1;
    wire w_detected = (r_detect_cnt>2);

    reg       r_busy = 0; // r_busy is set while receiving 9-bits data
    reg [3:0] r_bit  = 0; // the number of received bits
    reg [7:0] r_cnt  = 0; // wait count for 1Mbaud
    always@(posedge w_clk) r_cnt <= (r_busy==0) ? 1 : (r_cnt==`UART_CNT) ? 1 : r_cnt + 1;

    reg [8:0] r_data = 0;
    always@(posedge w_clk) begin
        if (r_busy==0) begin
            {r_data, r_bit, r_en} <= 0;
            if (w_detected) r_busy <= 1;
        end
        else if (r_cnt>= `UART_CNT) begin
            r_bit <= r_bit + 1;
            r_data <= {w_rxd, r_data[8:1]};
            if (r_bit==8) begin r_en <= 1; r_busy <= 0; end
        end
    end
    assign w_char = r_data[7:0];
endmodule

/********************************************************************************************/