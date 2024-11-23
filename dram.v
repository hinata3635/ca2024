/********************************************************************************************/
/* RVCore_Base, RISC-V RV32I (without some LD/ST) v2024-11-09a       ArchLab, Science Tokyo */
/********************************************************************************************/
`resetall
`default_nettype none

/********************************************************************************************/
module mig_ui (
    input  wire         ui_clk             ,
    input  wire         ui_rst             ,
    // User application ports
    input  wire  [31:0] addr_i             ,
    input  wire         wvalid_i           ,
    output wire         wready_o           ,
    input  wire [127:0] wdata_i            ,
    input  wire  [15:0] wstrb_i            ,
    input  wire         arvalid_i          ,
    output wire         arready_o          ,
    output wire         rvalid_o           ,
    output wire [127:0] rdata_o            ,
    // Application interface ports
    input  wire         init_calib_complete,
    output reg   [27:0] app_addr           ,
    output reg    [2:0] app_cmd            ,
    output reg          app_en             ,
    output reg  [127:0] app_wdf_data       ,
    output reg          app_wdf_end        ,
    output reg          app_wdf_wren       ,
    input  wire [127:0] app_rd_data        ,
    input  wire         app_rd_data_end    ,
    input  wire         app_rd_data_valid  ,
    input  wire         app_rdy            ,
    input  wire         app_wdf_rdy        ,
    output wire         app_sr_req         ,
    output wire         app_ref_req        ,
    output wire         app_zq_req         ,
    input  wire         app_sr_active      ,
    input  wire         app_ref_ack        ,
    input  wire         app_zq_ack         ,
    output reg  [15:0]  app_wdf_mask
);

    reg   [27:0] n_app_addr     = 0;
    reg    [2:0] n_app_cmd      = 0; // 3'b000 -> Write, 3'b001 -> Read
    reg          n_app_en       = 0;
    reg  [127:0] n_app_wdf_data = 0;
    reg          n_app_wdf_end  = 0;
    reg          n_app_wdf_wren = 0;
    reg   [15:0] n_app_wdf_mask = 0;

    // FSM
    localparam CALIB     = 3'b000;
    localparam IDLE      = 3'b001;
    localparam CMD_WDATA = 3'b010;
    localparam CMD       = 3'b011;
    localparam WDATA     = 3'b100;
    reg [2:0] state = CALIB, n_state;

    assign app_sr_req  = 1'b0;
    assign app_ref_req = 1'b0;
    assign app_zq_req  = 1'b0;

    wire ready = (state==IDLE);
    assign arready_o = ready;
    assign wready_o  = ready;

    assign rvalid_o = app_rd_data_valid;
    assign rdata_o  = app_rd_data      ;

    always @(*) begin
        n_state        = state       ;
        n_app_addr     = app_addr    ;
        n_app_cmd      = app_cmd     ;
        n_app_en       = app_en      ;
        n_app_wdf_data = app_wdf_data;
        n_app_wdf_end  = app_wdf_end ;
        n_app_wdf_wren = app_wdf_wren;
        n_app_wdf_mask = app_wdf_mask;
        casez (state)
            CALIB: begin
                if (init_calib_complete) begin
                    n_state = IDLE;
                end
            end
            IDLE: begin
                if (arvalid_i) begin // read
                    n_app_addr     = addr_i[27:0];
                    n_app_cmd      = 3'b001      ; // read command
                    n_app_en       = 1'b1        ;
                    n_state        = CMD         ;
                end else if (wvalid_i) begin // write
                    n_app_addr     = addr_i[27:0];
                    n_app_cmd      = 3'b000      ; // write command
                    n_app_en       = 1'b1        ;
                    n_app_wdf_data = wdata_i     ;
                    n_app_wdf_end  = 1'b1        ;
                    n_app_wdf_wren = 1'b1        ;
                    n_app_wdf_mask = ~wstrb_i    ;
                    n_state        = CMD_WDATA   ;
                end
            end
            CMD_WDATA: begin
                casez ({app_rdy, app_wdf_rdy})
                    2'b11  : begin
                        n_app_en       = 1'b0 ;
                        n_app_wdf_end  = 1'b0 ;
                        n_app_wdf_wren = 1'b0 ;
                        n_state        = IDLE ;
                    end
                    2'b01  : begin
                        n_app_wdf_end  = 1'b0 ;
                        n_app_wdf_wren = 1'b0 ;
                        n_state        = CMD  ;
                    end
                    2'b10  : begin
                        n_app_en       = 1'b0 ;
                        n_state        = WDATA;
                    end
                    default: ;
                endcase
            end
            CMD: begin
                if (app_rdy) begin
                    n_app_en = 1'b0;
                    n_state  = IDLE;
                end
            end
            WDATA: begin
                if (app_wdf_rdy) begin
                    n_app_wdf_end  = 1'b0;
                    n_app_wdf_wren = 1'b0;
                    n_state        = IDLE;
                end
            end
            default: begin
                n_app_en       = 1'b0 ;
                n_app_wdf_end  = 1'b0 ;
                n_app_wdf_wren = 1'b0 ;
                n_state        = CALIB;
            end
        endcase
    end

    always @(posedge ui_clk) begin
        if (ui_rst) begin
            app_en       <= 1'b0 ;
            app_wdf_end  <= 1'b0 ;
            app_wdf_wren <= 1'b0 ;
            state        <= CALIB;
        end else begin
            app_addr     <= n_app_addr    ;
            app_cmd      <= n_app_cmd     ;
            app_en       <= n_app_en      ;
            app_wdf_data <= n_app_wdf_data;
            app_wdf_end  <= n_app_wdf_end ;
            app_wdf_wren <= n_app_wdf_wren;
            app_wdf_mask <= n_app_wdf_mask;
            state        <= n_state       ;
        end
    end

endmodule

/********************************************************************************************/
module dram_controller (
    input  wire         clk_i       , // 100.00    MHz
    input  wire         rst_ni      ,
    // User application ports
    output wire         clk_o       , //  83.33333 MHz
    output wire         rst_o       ,
    input  wire  [31:0] addr_i      ,
    input  wire         wvalid_i    ,
    output wire         wready_o    ,
    input  wire [127:0] wdata_i     ,
    input  wire  [15:0] wstrb_i     ,
    input  wire         arvalid_i   ,
    output wire         arready_o   ,
    output wire         rvalid_o    ,
    output wire [127:0] rdata_o     ,
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

    // Clock and Reset Signals
    wire sys_clk_i;
    wire clk_ref_i;
    wire sys_rst  ;

`ifdef SYNTHESIS
    wire locked;
`else
    wire locked  = 1 ;
`endif

`ifdef SYNTHESIS
    clk_wiz_0 dram_clk_gen (
        .clk_out1(sys_clk_i), // 160 MHz
        .clk_out2(clk_ref_i), // 200 MHz
        .locked  (locked   ), // output locked
        .clk_in1 (clk_i    )  // 100 MHz
    );
`else // Simulation
    reg clk_166_66667_mhz = 1'b0; always #6 clk_166_66667_mhz <= ~clk_166_66667_mhz;
    reg clk_200_mhz       = 1'b0; always #5 clk_200_mhz       <= ~clk_200_mhz      ;
    assign sys_clk_i = clk_166_66667_mhz;
    assign clk_ref_i = clk_200_mhz      ;
`endif

    wire rst_n_async = (rst_ni && locked);
    reg  rst1, rst2;
    always @(posedge sys_clk_i or negedge rst_n_async) begin
        if (!rst_n_async) begin
            rst1 <= 1'b1;
            rst2 <= 1'b1;
        end else begin
            rst1 <= 1'b0;
            rst2 <= rst1;
        end
    end
    assign sys_rst = rst2;

    // Application interface ports
    wire         ui_clk             ; // 83.33333 MHz (333.33/4)
    wire         ui_clk_sync_rst    ; // active high
    wire         init_calib_complete;
    wire  [27:0] app_addr           ;
    wire   [2:0] app_cmd            ; // 3'b000 -> Write, 3'b001 -> Read
    wire         app_en             ;
    wire [127:0] app_wdf_data       ;
    wire         app_wdf_end        ;
    wire         app_wdf_wren       ;
    wire [127:0] app_rd_data        ;
    wire         app_rd_data_end    ;
    wire         app_rd_data_valid  ;
    wire         app_rdy            ;
    wire         app_wdf_rdy        ;
    wire         app_sr_req         ;
    wire         app_ref_req        ;
    wire         app_zq_req         ;
    wire         app_sr_active      ;
    wire         app_ref_ack        ;
    wire         app_zq_ack         ;
    wire  [15:0] app_wdf_mask       ;

    assign clk_o = ui_clk           ;
    assign rst_o = ui_clk_sync_rst  ;

    mig_ui mig_ui0 (
        .ui_clk             (ui_clk             ), // input          ui_clk
        .ui_rst             (ui_clk_sync_rst    ), // input          ui_rst
        // User application ports
        .addr_i             (addr_i             ), // input   [31:0] addr_i
        .wvalid_i           (wvalid_i           ), // input          wvalid_i
        .wready_o           (wready_o           ), // output         wready_o
        .wdata_i            (wdata_i            ), // input  [127:0] wdata_i
        .wstrb_i            (wstrb_i            ), // input   [15:0] wstrb_i
        .arvalid_i          (arvalid_i          ), // input          arvalid_i
        .arready_o          (arready_o          ), // output         arready_o
        .rvalid_o           (rvalid_o           ), // output         rvalid_o
        .rdata_o            (rdata_o            ), // output [127:0] rdata_o
        // Application interface ports
        .init_calib_complete(init_calib_complete), // input          init_calib_complete
        .app_addr           (app_addr           ), // output  [27:0] app_addr
        .app_cmd            (app_cmd            ), // output   [2:0] app_cmd
        .app_en             (app_en             ), // output         app_en
        .app_wdf_data       (app_wdf_data       ), // output [127:0] app_wdf_data
        .app_wdf_end        (app_wdf_end        ), // output         app_wdf_end
        .app_wdf_wren       (app_wdf_wren       ), // output         app_wdf_wren
        .app_rd_data        (app_rd_data        ), // input  [127:0] app_rd_data
        .app_rd_data_end    (app_rd_data_end    ), // input          app_rd_data_end
        .app_rd_data_valid  (app_rd_data_valid  ), // input          app_rd_data_valid
        .app_rdy            (app_rdy            ), // input          app_rdy
        .app_wdf_rdy        (app_wdf_rdy        ), // input          app_wdf_rdy
        .app_sr_req         (app_sr_req         ), // output         app_sr_req
        .app_ref_req        (app_ref_req        ), // output         app_ref_req
        .app_zq_req         (app_zq_req         ), // output         app_zq_req
        .app_sr_active      (app_sr_active      ), // input          app_sr_active
        .app_ref_ack        (app_ref_ack        ), // input          app_ref_ack
        .app_zq_ack         (app_zq_ack         ), // input          app_zq_ack
        .app_wdf_mask       (app_wdf_mask       )  // output [15:0]  app_wdf_mask
    );

`ifdef SYNTHESIS
    mig_7series_0 u_mig_7series_0 (
        // Memory interface ports
        .ddr3_addr          (ddr3_addr          ), // output [13:0]  ddr3_addr
        .ddr3_ba            (ddr3_ba            ), // output [2:0]   ddr3_ba
        .ddr3_cas_n         (ddr3_cas_n         ), // output         ddr3_cas_n
        .ddr3_ck_n          (ddr3_ck_n          ), // output [0:0]   ddr3_ck_n
        .ddr3_ck_p          (ddr3_ck_p          ), // output [0:0]   ddr3_ck_p
        .ddr3_cke           (ddr3_cke           ), // output [0:0]   ddr3_cke
        .ddr3_ras_n         (ddr3_ras_n         ), // output         ddr3_ras_n
        .ddr3_reset_n       (ddr3_reset_n       ), // output         ddr3_reset_n
        .ddr3_we_n          (ddr3_we_n          ), // output         ddr3_we_n
        .ddr3_dq            (ddr3_dq            ), // inout  [15:0]  ddr3_dq
        .ddr3_dqs_n         (ddr3_dqs_n         ), // inout  [1:0]   ddr3_dqs_n
        .ddr3_dqs_p         (ddr3_dqs_p         ), // inout  [1:0]   ddr3_dqs_p
        .init_calib_complete(init_calib_complete), // output         init_calib_complete
        .ddr3_cs_n          (ddr3_cs_n          ), // output [0:0]   ddr3_cs_n
        .ddr3_dm            (ddr3_dm            ), // output [1:0]   ddr3_dm
        .ddr3_odt           (ddr3_odt           ), // output [0:0]   ddr3_odt
        // Application interface ports
        .app_addr           (app_addr           ), // input  [27:0]  app_addr
        .app_cmd            (app_cmd            ), // input  [2:0]   app_cmd
        .app_en             (app_en             ), // input          app_en
        .app_wdf_data       (app_wdf_data       ), // input  [127:0] app_wdf_data
        .app_wdf_end        (app_wdf_end        ), // input          app_wdf_end
        .app_wdf_wren       (app_wdf_wren       ), // input          app_wdf_wren
        .app_rd_data        (app_rd_data        ), // output [127:0] app_rd_data
        .app_rd_data_end    (app_rd_data_end    ), // output         app_rd_data_end
        .app_rd_data_valid  (app_rd_data_valid  ), // output         app_rd_data_valid
        .app_rdy            (app_rdy            ), // output         app_rdy
        .app_wdf_rdy        (app_wdf_rdy        ), // output         app_wdf_rdy
        .app_sr_req         (app_sr_req         ), // input          app_sr_req
        .app_ref_req        (app_ref_req        ), // input          app_ref_req
        .app_zq_req         (app_zq_req         ), // input          app_zq_req
        .app_sr_active      (app_sr_active      ), // output         app_sr_active
        .app_ref_ack        (app_ref_ack        ), // output         app_ref_ack
        .app_zq_ack         (app_zq_ack         ), // output         app_zq_ack
        .ui_clk             (ui_clk             ), // output         ui_clk
        .ui_clk_sync_rst    (ui_clk_sync_rst    ), // output         ui_clk_sync_rst
        .app_wdf_mask       (app_wdf_mask       ), // input  [15:0]  app_wdf_mask
        // System Clock Ports
        .sys_clk_i          (sys_clk_i          ), // 166.66667 MHz
        // Reference Clock Ports
        .clk_ref_i          (clk_ref_i          ), // 200.00000 MHz
        .sys_rst            (sys_rst            )  // input          sys_rst
    );
`else // Simulation
    mig_simulator mig_simulator0 (
        // Memory interface ports
        .ddr3_addr          (ddr3_addr          ), // output [13:0]  ddr3_addr
        .ddr3_ba            (ddr3_ba            ), // output [2:0]   ddr3_ba
        .ddr3_cas_n         (ddr3_cas_n         ), // output         ddr3_cas_n
        .ddr3_ck_n          (ddr3_ck_n          ), // output [0:0]   ddr3_ck_n
        .ddr3_ck_p          (ddr3_ck_p          ), // output [0:0]   ddr3_ck_p
        .ddr3_cke           (ddr3_cke           ), // output [0:0]   ddr3_cke
        .ddr3_ras_n         (ddr3_ras_n         ), // output         ddr3_ras_n
        .ddr3_reset_n       (ddr3_reset_n       ), // output         ddr3_reset_n
        .ddr3_we_n          (ddr3_we_n          ), // output         ddr3_we_n
        .ddr3_dq            (ddr3_dq            ), // inout  [15:0]  ddr3_dq
        .ddr3_dqs_n         (ddr3_dqs_n         ), // inout  [1:0]   ddr3_dqs_n
        .ddr3_dqs_p         (ddr3_dqs_p         ), // inout  [1:0]   ddr3_dqs_p
        .init_calib_complete(init_calib_complete), // output         init_calib_complete
        .ddr3_cs_n          (ddr3_cs_n          ), // output [0:0]   ddr3_cs_n
        .ddr3_dm            (ddr3_dm            ), // output [1:0]   ddr3_dm
        .ddr3_odt           (ddr3_odt           ), // output [0:0]   ddr3_odt
        // Application interface ports
        .app_addr_i         (app_addr           ), // input  [27:0]  app_addr
        .app_cmd_i          (app_cmd            ), // input  [2:0]   app_cmd
        .app_en_i           (app_en             ), // input          app_en
        .app_wdf_data_i     (app_wdf_data       ), // input  [127:0] app_wdf_data
        .app_wdf_end_i      (app_wdf_end        ), // input          app_wdf_end
        .app_wdf_wren_i     (app_wdf_wren       ), // input          app_wdf_wren
        .app_rd_data_o      (app_rd_data        ), // output [127:0] app_rd_data
        .app_rd_data_end_o  (app_rd_data_end    ), // output         app_rd_data_end
        .app_rd_data_valid_o(app_rd_data_valid  ), // output         app_rd_data_valid
        .app_rdy_o          (app_rdy            ), // output         app_rdy
        .app_wdf_rdy_o      (app_wdf_rdy        ), // output         app_wdf_rdy
        .app_sr_req_i       (app_sr_req         ), // input          app_sr_req
        .app_ref_req_i      (app_ref_req        ), // input          app_ref_req
        .app_zq_req_i       (app_zq_req         ), // input          app_zq_req
        .app_sr_active_o    (app_sr_active      ), // output         app_sr_active
        .app_ref_ack_o      (app_ref_ack        ), // output         app_ref_ack
        .app_zq_ack_o       (app_zq_ack         ), // output         app_zq_ack
        .ui_clk_o           (ui_clk             ), // output         ui_clk
        .ui_clk_sync_rst_o  (ui_clk_sync_rst    ), // output         ui_clk_sync_rst
        .app_wdf_mask_i     (app_wdf_mask       ), // input  [15:0]  app_wdf_mask
        // System Clock Ports
        .sys_clk_i          (sys_clk_i          ), // 166.66667 MHz
        // Reference Clock Ports
        .clk_ref_i          (clk_ref_i          ), // 200.00000 MHz
        .sys_rst_i          (sys_rst            )  // input          sys_rst
    );
`endif // SYNTHESIS

endmodule


/********************************************************************************************/
module async_fifo #(
    parameter ADDR_WIDTH = 9 ,
    parameter DATA_WIDTH = 64
) (
    input  wire                  wclk_i  ,
    input  wire                  rclk_i  ,
    input  wire                  wrst_i  ,
    input  wire                  rrst_i  ,
    input  wire                  wvalid_i,
    output wire                  wready_o,
    input  wire [DATA_WIDTH-1:0] wdata_i ,
    output reg                   rvalid_o,
    input  wire                  rready_i,
    output reg  [DATA_WIDTH-1:0] rdata_o
);

    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] ram [0:(2**ADDR_WIDTH)-1];
    reg   [ADDR_WIDTH:0] waddr = 0, gray_waddr1 = 0, gray_waddr2 = 0;
    reg   [ADDR_WIDTH:0] raddr = 0, gray_raddr1 = 0, gray_raddr2 = 0;
    wire  [ADDR_WIDTH:0] gray_waddr = waddr ^ {1'b0, waddr[ADDR_WIDTH:1]};
    wire  [ADDR_WIDTH:0] gray_raddr = raddr ^ {1'b0, raddr[ADDR_WIDTH:1]};

    always @(posedge rclk_i) begin
        if (rrst_i) begin
            gray_waddr1 <= 0;
            gray_waddr2 <= 0;
        end else begin
            gray_waddr1 <= gray_waddr ;
            gray_waddr2 <= gray_waddr1;
        end
    end

    always @(posedge wclk_i) begin
        if (wrst_i) begin
            gray_raddr1 <= 0;
            gray_raddr2 <= 0;
        end else begin
            gray_raddr1 <= gray_raddr ;
            gray_raddr2 <= gray_raddr1;
        end
    end

    wire empty_n = (gray_raddr!=gray_waddr2);
    wire full_n  = (gray_waddr!={~gray_raddr2[ADDR_WIDTH:ADDR_WIDTH-1], gray_raddr2[ADDR_WIDTH-2:0]});

    // FIFO Write
    assign wready_o = full_n;
    always @(posedge wclk_i) begin
        if (wvalid_i && wready_o) begin
            ram[waddr[ADDR_WIDTH-1:0]] <= wdata_i;
        end
        if (wrst_i) begin
            waddr <= 0;
        end else begin
            if (wvalid_i && wready_o) begin
                waddr <= waddr+1;
            end
        end
    end

    // FIFO Read
    always @(posedge rclk_i) begin
        rdata_o <= ram[raddr[ADDR_WIDTH-1:0]];
        if (rrst_i) begin
            rvalid_o <= 1'b0;
            raddr    <= 0   ;
        end else begin
            if (rvalid_o && rready_i) begin
                rvalid_o <= 1'b0   ;
                raddr    <= raddr+1;
            end else begin
                rvalid_o <= empty_n;
            end
        end
    end

endmodule

/********************************************************************************************/
module dram (
    input  wire                  clk_i       ,
    input  wire                  rst_ni      ,
    // User application ports
    output wire                  clk_o       ,
    output wire                  rst_o       ,
    output wire                  busy_o      ,
    input  wire           [31:0] addr_i      ,
    input  wire                  wvalid_i    ,
    input  wire           [31:0] wdata_i     ,
    input  wire                  arvalid_i   ,
    output reg                   rvalid_o    ,
    output reg            [31:0] rdata_o     ,
    // Memory interface ports
    output wire           [13:0] ddr3_addr   ,
    output wire            [2:0] ddr3_ba     ,
    output wire                  ddr3_cas_n  ,
    output wire            [0:0] ddr3_ck_n   ,
    output wire            [0:0] ddr3_ck_p   ,
    output wire            [0:0] ddr3_cke    ,
    output wire                  ddr3_ras_n  ,
    output wire                  ddr3_reset_n,
    output wire                  ddr3_we_n   ,
    inout  wire           [15:0] ddr3_dq     ,
    inout  wire            [1:0] ddr3_dqs_n  ,
    inout  wire            [1:0] ddr3_dqs_p  ,
    output wire            [0:0] ddr3_cs_n   ,
    output wire            [1:0] ddr3_dm     ,
    output wire            [0:0] ddr3_odt
);

    reg                                n_rvalid_o;

    reg            [31:0] addr_t  = 0, n_addr_t  ;
    reg            [31:0] wdata_t = 0, n_wdata_t ;
    reg           [127:0] rdata_t = 0, n_rdata_t ;

    // Processor <-> Async FIFO
    reg                   wvalid  = 0, n_wvalid  ;
    wire                  wready                 ;
    reg           [127:0] wdata                  ;
    reg            [15:0] wstrb                  ;
    reg                   arvalid = 0, n_arvalid ;
    wire                  arready                ;
    wire                  rvalid                 ;
    wire          [127:0] rdata                  ;

    // FSM
    localparam IDLE  = 2'b00;
    localparam READ  = 2'b01;
    localparam WRITE = 2'b10;
    reg [1:0]  state = IDLE, n_state;

    assign busy_o = (state!=IDLE);

    always @(*) begin
        n_addr_t   = addr_t ;
        n_wvalid   = wvalid ;
        n_wdata_t  = wdata_t;
        n_arvalid  = arvalid;
        n_rvalid_o = 1'b0   ;
        n_rdata_t  = rdata_t;
        n_state    = state  ;
        casez (state)
            IDLE: begin
                if (arvalid_i) begin
                    n_arvalid = 1'b1   ;
                    n_addr_t  = addr_i ;
                    n_state   = READ   ;
                end else if (wvalid_i) begin
                    n_wvalid  = 1'b1   ;
                    n_addr_t  = addr_i ;
                    n_wdata_t = wdata_i;
                    n_state   = WRITE  ;
                end
            end
            READ: begin
                if (arready) begin
                    n_arvalid = 1'b0;
                end
                if (rvalid) begin
                    n_rvalid_o = 1'b1 ;
                    n_rdata_t  = rdata;
                    n_state    = IDLE ;
                end
            end
            WRITE: begin
                if (wready) begin
                    n_wvalid = 1'b0;
                    n_state  = IDLE;
                end
            end
            default: begin
                n_arvalid = 1'b0;
                n_wvalid  = 1'b0;
                n_state   = IDLE;
            end
        endcase
    end

    always @(posedge clk_o) begin
        if (rst_o) begin
            wvalid   <= 1'b0;
            arvalid  <= 1'b0;
            rvalid_o <= 1'b0;
            state    <= IDLE;
        end else begin
            addr_t   <= n_addr_t  ;
            wvalid   <= n_wvalid  ;
            wdata_t  <= n_wdata_t ;
            arvalid  <= n_arvalid ;
            rvalid_o <= n_rvalid_o;
            rdata_t  <= n_rdata_t ;
            state    <= n_state   ;
        end
    end

    integer i;
    wire [31:0] addr   = {addr_t[31:4], 3'b000};
    wire  [3:0] offset =            addr_t[3:0];

    // Write Data
    always @(*) begin
        wdata = 0;
        for (i=0; i<4; i=i+1) begin
            if (offset[3:2]==i) begin
                wdata[i*32+:32] = wdata_t;
            end
        end
    end

    // Write Strobe
    always @(*) begin
        wstrb = 16'h0;
        for (i=0; i<4; i=i+1) begin
            if (offset[3:2]==i) begin
                wstrb[i*4+:4] = 4'b1111;
            end
        end
    end

    // Read Data
    always @(*) begin
        rdata_o = 0;
        for (i=0; i<4; i=i+1) begin
            if (offset[3:2]==i) begin
                rdata_o = rdata_t[i*32+:32];
            end
        end
    end

/********************************************************************************************/
    // Clock and Reset Signals
    wire ui_clk;
    wire ui_rst;

`ifdef SYNTHESIS
    wire locked;
`else
    wire locked  = 1 ;
`endif

`ifdef SYNTHESIS
    clk_wiz_1 user_clk_gen (
        .clk_out1(clk_o ), // output 40 MHz user clock, please select this freq.
        .locked  (locked), // output locked
        .clk_in1 (ui_clk)  // input 80 MHz clock signal
    );
`else // Simulation
    reg clk = 1'b0; always #6 clk <= ~clk;
    assign clk_o = clk;
`endif // SYNTHESIS

    wire rst_async = (ui_rst || !locked);
    reg  rst1, rst2;
    always @(posedge clk_o or posedge rst_async) begin
        if (rst_async) begin
            rst1 <= 1'b1;
            rst2 <= 1'b1;
        end else begin
            rst1 <= 1'b0;
            rst2 <= rst1;
        end
    end
    assign rst_o = rst2;

    // Processor -> Async FIFO -> DRAM Controller
    wire         tx_fifo_wvalid;
    wire         tx_fifo_wready;
    wire [176:0] tx_fifo_wdata ; // {arvalid, addr, wdata, wstrb}
    wire         tx_fifo_rvalid;
    wire         tx_fifo_rready;
    wire [176:0] tx_fifo_rdata ; // {arvalid, addr, wdata, wstrb}

    assign tx_fifo_wvalid = (wvalid || arvalid)          ;
    assign tx_fifo_wdata  = {arvalid, addr, wdata, wstrb};
    assign wready         = tx_fifo_wready               ;
    assign arready        = tx_fifo_wready               ;

    async_fifo #(
        .ADDR_WIDTH(9  ),
        .DATA_WIDTH(177)
    ) req_async_fifo0 (
        .wclk_i  (clk_o         ),
        .rclk_i  (ui_clk        ),
        .wrst_i  (rst_o         ),
        .rrst_i  (ui_rst        ),
        .wvalid_i(tx_fifo_wvalid),
        .wready_o(tx_fifo_wready),
        .wdata_i (tx_fifo_wdata ),
        .rvalid_o(tx_fifo_rvalid),
        .rready_i(tx_fifo_rready),
        .rdata_o (tx_fifo_rdata )
    );

    // DRAM Controller -> Async FIFO -> Processor
    wire         rx_fifo_wvalid ;
    wire         rx_fifo_wready ;
    wire [127:0] rx_fifo_wdata  ;
    wire         rx_fifo_arvalid;
    wire         rx_fifo_arready;
    wire         rx_fifo_rvalid ;
    wire         rx_fifo_rready ;
    wire [127:0] rx_fifo_rdata  ;

    assign rvalid         = rx_fifo_rvalid;
    assign rx_fifo_rready = 1'b1          ;
    assign rdata          = rx_fifo_rdata ;

    async_fifo #(
        .ADDR_WIDTH(9  ),
        .DATA_WIDTH(128)
    ) rsp_async_fifo0 (
        .wclk_i  (ui_clk        ),
        .rclk_i  (clk_o         ),
        .wrst_i  (ui_rst        ),
        .rrst_i  (rst_o         ),
        .wvalid_i(rx_fifo_wvalid),
        .wready_o(rx_fifo_wready),
        .wdata_i (rx_fifo_wdata ),
        .rvalid_o(rx_fifo_rvalid),
        .rready_i(rx_fifo_rready),
        .rdata_o (rx_fifo_rdata )
    );

    // Async FIFO <-> DRAM Conrroller
    wire  [31:0] dc_addr   ;
    wire         dc_wvalid ;
    wire         dc_wready ;
    wire [127:0] dc_wdata  ;
    wire  [15:0] dc_wstrb  ;
    wire         dc_arvalid;
    wire         dc_arready;
    wire         dc_rvalid ;
    wire [127:0] dc_rdata  ;

    wire   dc_cmd         = tx_fifo_rdata[176]         ; // 1'b0 -> write, 1'b1 -> read
    assign dc_wvalid      = (tx_fifo_rvalid && !dc_cmd);
    assign dc_arvalid     = (tx_fifo_rvalid &&  dc_cmd);
    assign tx_fifo_rready = (dc_wready && dc_arready)  ;
    assign {dc_addr, dc_wdata, dc_wstrb} = tx_fifo_rdata[175:0];

    assign rx_fifo_wvalid = dc_rvalid;
    assign rx_fifo_wdata  = dc_rdata ;


    dram_controller dram_controller0 (
        .clk_i       (clk_i       ), // input          clk_i    // 100 MHz
        .rst_ni      (rst_ni      ), // input          rst_ni
        // User application ports
        .clk_o       (ui_clk      ), // output         clk_o    //  80 MHz
        .rst_o       (ui_rst      ), // output         rst_o
        .addr_i      (dc_addr     ), // input   [31:0] addr_i
        .wvalid_i    (dc_wvalid   ), // input          wvalid_i
        .wready_o    (dc_wready   ), // output         wready_o
        .wdata_i     (dc_wdata    ), // input  [127:0] wdata_i
        .wstrb_i     (dc_wstrb    ), // input   [15:0] wstrb_i
        .arvalid_i   (dc_arvalid  ), // input          arvalid_i
        .arready_o   (dc_arready  ), // output         arready_o
        .rvalid_o    (dc_rvalid   ), // output         rvalid_o
        .rdata_o     (dc_rdata    ), // output [127:0] rdata_o
        // Memory interface ports
        .ddr3_addr   (ddr3_addr   ), // output [13:0]  ddr3_addr
        .ddr3_ba     (ddr3_ba     ), // output [2:0]   ddr3_ba
        .ddr3_cas_n  (ddr3_cas_n  ), // output         ddr3_cas_n
        .ddr3_ck_n   (ddr3_ck_n   ), // output [0:0]   ddr3_ck_n
        .ddr3_ck_p   (ddr3_ck_p   ), // output [0:0]   ddr3_ck_p
        .ddr3_cke    (ddr3_cke    ), // output [0:0]   ddr3_cke
        .ddr3_ras_n  (ddr3_ras_n  ), // output         ddr3_ras_n
        .ddr3_reset_n(ddr3_reset_n), // output         ddr3_reset_n
        .ddr3_we_n   (ddr3_we_n   ), // output         ddr3_we_n
        .ddr3_dq     (ddr3_dq     ), // inout  [15:0]  ddr3_dq
        .ddr3_dqs_n  (ddr3_dqs_n  ), // inout  [1:0]   ddr3_dqs_n
        .ddr3_dqs_p  (ddr3_dqs_p  ), // inout  [1:0]   ddr3_dqs_p
        .ddr3_cs_n   (ddr3_cs_n   ), // output [0:0]   ddr3_cs_n
        .ddr3_dm     (ddr3_dm     ), // output [1:0]   ddr3_dm
        .ddr3_odt    (ddr3_odt    )  // output [0:0]   ddr3_odt
    );

endmodule

`resetall
/********************************************************************************************/
