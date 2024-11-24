/********************************************************************************************/
/* RVCore_Base, RISC-V RV32I (without some LD/ST) v2024-11-09a       ArchLab, Science Tokyo */
/********************************************************************************************/
`default_nettype none
// `define VERIFY                   // define this to generate verify.txt
`define START_PC  0              // initial PC value
`define D_I_TYPE  0
`define D_R_TYPE  1
`define D_S_TYPE  2
`define D_B_TYPE  3
`define D_J_TYPE  4
`define D_U_TYPE  5
`define D_JA__IS  6
`define D_LD__IS  7
`define D_MUL_IS  8
`define D_DIV_IS  9

/********************************************************************************************/
module m_rvcore ( ///// RVCore Simple Version
    input  wire        w_clk, w_rst, w_stall, // Note
    input  wire [31:0] I_IN, D_IN,
    output wire [31:0] I_ADDR, D_ADDR,
    output wire [31:0] D_OUT,
    output wire        D_RE, // Read Enable
    output wire        D_WE  // Write Enable
);
    reg r_rst = 1;
    always @(posedge w_clk) r_rst <= w_rst;

    reg r_state = 0;
    always @(posedge w_clk) if (r_rst==0 && w_stall==0) r_state <= r_state + 1;

    reg [31:0] r_pc = `START_PC;
    assign I_ADDR = r_pc;
    always @(posedge w_clk) if (w_stall==0 && r_state==1)
      r_pc <= (r_rst) ? `START_PC : (w_b_rslt) ? w_tkn_pc : r_pc+4;

    wire [31:0] w_ir = I_IN;
    wire [31:0] w_rrs1, w_rrs2_t, w_imm_t, w_rslt;
    wire [ 4:0] w_rs1, w_rs2, w_rd;
    wire        w_op_im;
    wire [9:0]  w_itype;
    wire [10:0] w_alu_c;
    wire [6:0]  w_bru_c;
    wire        w_b_rslt;
    wire        w_ja   = w_itype[6];
    wire [4:0]  w_op   = w_ir[6:2];

    m_decode decode1 (w_ir, w_rd, w_rs1, w_rs2, w_op_im, w_itype, w_alu_c, w_bru_c, w_imm_t);

    m_regfile regfile1 (w_clk, w_rs1, w_rs2, w_rrs1, w_rrs2_t, w_ma_rd, w_ma_rslt);
    wire [31:0] w_imm  = (w_ja) ? r_pc+4 : (w_op==5'b00101) ? r_pc+w_imm_t : w_imm_t;
    wire [31:0] w_rrs2 = (w_op_im) ? w_imm : w_rrs2_t;
    wire [31:0] w_tkn_pc = (w_ir[6:2]==5'b11001) ? w_rrs1+w_imm_t : r_pc+w_imm_t;

    reg [31:0] r_rrs1=0, r_rrs2=0, r_imm=0;
    reg [9:0]  r_itype=0;
    reg [10:0] r_alu_c=0;
    reg [6:0]  r_bru_c=0;
    reg [4:0]  r_rd=0;
    always @(posedge w_clk) if(r_state==0) begin
        {r_rrs1, r_rrs2, r_imm} <= {w_rrs1, w_rrs2, w_imm};
        {r_itype, r_alu_c, r_bru_c, r_rd} <= {w_itype, w_alu_c, w_bru_c, w_rd};
    end

    m_alu alu1 (r_rrs1, r_rrs2, r_alu_c, w_rslt);
    m_bru bru0 (r_rrs1, r_rrs2, r_bru_c, w_b_rslt);

    /***** for load and store instructions *****/
    assign D_RE   = (r_state==1) ? 0 : w_itype[`D_LD__IS];
    assign D_WE   = (r_state==1) ? 0 : w_itype[`D_S_TYPE]  ? 1 : 0; //4'b1111 : 0;
    assign D_ADDR = w_rrs1 + w_imm;
    assign D_OUT  = w_rrs2;

    wire [31:0] w_ma_rslt_t = (r_itype[`D_LD__IS]) ? D_IN : w_rslt;
    wire [4:0]  w_ma_rd     = (r_state==0) ? 0 : (w_stall) ? 0 : r_rd ;
    wire [31:0] w_ma_rslt   = (r_state==0) ? 0 : (w_stall) ? 0 : (r_rd!=0) ? w_ma_rslt_t : 0;

    /****************************************************************************************/
`ifdef VERIFY
    initial $write("\n== VERIFY is defined and generate verify.txt\n");
    integer fd;
    initial fd = $fopen("verify.txt", "w");
    reg [31:0] r_tc = 1;
    integer i, j;
    reg r_halt = 0;
    always @(posedge w_clk) if (!r_rst && w_stall==0 && r_state==1 && r_halt==0) begin
        r_tc <= r_tc + 1;
        $fwrite(fd, "%03x\n", r_pc);
        if (I_IN==32'h0000006f) r_halt = 1;
        $fflush();
    end
`endif
endmodule

/********************************************************************************************/
module m_regfile ( ///// register file
    input  wire        w_clk,
    input  wire [ 4:0] rs1, rs2,
    output wire [31:0] rdata1, rdata2,
    input  wire [ 4:0] rd,
    input  wire [31:0] wdata
);
    reg [31:0] mem [0:31];
    integer i; initial begin for(i=0; i<32; i=i+1) mem[i]=0; end
    assign rdata1 = mem[rs1];
    assign rdata2 = mem[rs2];
    always @(posedge w_clk) mem[rd] <= wdata;
endmodule

/********************************************************************************************/
module m_bru ( ///// Branch Resolution Unit
    input  wire [31:0] in1, in2,
    input  wire [ 6:0] s,
    output wire        rslt
);
    wire signed [31:0] sin1 = in1;
    wire signed [31:0] sin2 = in2;
    wire ex0 = s[0] & ( in1 ==  in2);  // BEQ
    wire ex1 = s[1] & ( in1 !=  in2);  // BNE
    wire ex2 = s[2] & (sin1 <  sin2);  // BLT
    wire ex3 = s[3] & (sin1 >= sin2);  // BGE
    wire ex4 = s[4] & ( in1 <   in2);  // BLTU
    wire ex5 = s[5] & ( in1 >=  in2);  // BGEU
    wire ex6 = s[6];                   // JAL, JALR  -> always taken
    assign rslt = ex0 ^ ex1 ^ ex2 ^ ex3 ^ ex4 ^ ex5 ^ ex6;
endmodule

/********************************************************************************************/
module m_alu ( ///// Arithmetic and Logic Unit
    input  wire [31:0] in1, in2,
    input  wire [10:0] s,
    output wire [31:0] rslt
);
    wire signed [31:0] sin1 = in1;
    wire signed [31:0] sin2 = in2;
    wire        ex0  = (s[0])  ?  in1 <  in2      : 0;
    wire        ex1  = (s[1])  ? sin1 < sin2      : 0;
    wire [31:0] ex2  = (s[2])  ?  in1 +  in2      : 0;
    wire [31:0] ex3  = (s[3])  ?  in1 -  in2      : 0;
    wire [31:0] ex4  = (s[4])  ?  in1 ^  in2      : 0;
    wire [31:0] ex5  = (s[5])  ?  in1 |  in2      : 0;
    wire [31:0] ex6  = (s[6])  ?  in1 &  in2      : 0;
    wire [31:0] ex7  = (s[7])  ?  in1 << in2[4:0] : 0;
    wire [31:0] ex8  = (s[8])  ?  in1 >> in2[4:0] : 0;
    wire [31:0] ex9  = (s[9])  ? sin1 >>>in2[4:0] : 0;
    wire [31:0] ex10 = (s[10]) ?         in2      : 0;
    assign rslt = {31'd0, ex0} ^ {31'd0, ex1} ^
                  ex2 ^ ex3 ^ ex4 ^ ex5 ^ ex6 ^ ex7 ^ ex8 ^ ex9 ^ ex10;
endmodule

/********************************************************************************************/
module m_get_imm(ir, i, s, b, u, j, imm);
  input wire [31:0] ir;
  input wire i, s, b, u, j;
  output wire [31:0] imm;
  assign imm= (i) ? {{20{ir[31]}},ir[31:20]} :
              (s) ? {{20{ir[31]}},ir[31:25],ir[11:7]} :
              (b) ? {{20{ir[31]}},ir[7],ir[30:25],ir[11:8],1'b0} :
              (u) ? {ir[31:12],12'b0} :
              (j) ? {{12{ir[31]}},ir[19:12],ir[20],ir[30:21],1'b0} : 0;
endmodule

/********************************************************************************************/
module m_decode (
    input  wire [31:0] ir,
    output wire [ 4:0] rd, rs1, rs2,
    output wire        op_imm,
    output wire [ 9:0] itype,
    output wire [10:0] alu_c,
    output wire [ 6:0] bru_c,
    output wire [31:0] w_imm
);
    wire [4:0] opcode5 = ir[6:2]; // use 5-bit, cause lower 2-bit are always 2'b11
    wire j = (opcode5==5'b11011);                       // J-type
    wire b = (opcode5==5'b11000);                       // B-type
    wire s = (opcode5==5'b01000);                       // S-type
    wire r = (opcode5==5'b01100);                       // R-type
    wire u = (opcode5==5'b01101 || opcode5==5'b00101);  // U-type
    wire i = ~(j | b | s | r | u);                      // I-type

    wire ja = (opcode5==5'b11001 || opcode5==5'b11011); // JAL, JALR insn
    wire ld = (opcode5==5'b00000);                      // LB, LH, LW, LBU, LHU insn
    wire mul= r & ir[25] & !ir[14];                     // MUL, MULH, MULHSU, MULHU insn
    wire div= r & ir[25] &  ir[14];                     // DIV, DIVU, REM, REMU insn
    wire reg_we = (ir[11:7]!=0) && (!s && !b);          // rd!=0 & (not store & not branch)
    assign op_imm = (opcode5==5'b00100 || u | ja);      // use immediate for rrs2, Note
    assign rd     = (reg_we)    ? ir[11:7]  : 5'd0;
    assign rs1    = (!u && !j)  ? ir[19:15] : 5'd0;
    assign rs2    = (b | s | r) ? ir[24:20] : 5'd0;
    assign itype = {div, mul, ld, ja, u, j, b, s, r, i};

    wire [2:0] fct3   = ir[14:12]; // funct3
    wire [3:0] alu_op = (opcode5==5'b01100) ? {ir[30], fct3} :
                        (opcode5==5'b00100) ? {(fct3==3'h5 & ir[30]), fct3} : 4'hf;
    /********** one-hot encoding of ALU control ***********************/
    assign alu_c[0] = (alu_op==4'h3);      // `ALU_CTRL_SLTU
    assign alu_c[1] = (alu_op==4'h2);      // `ALU_CTRL_SLT
    assign alu_c[2] = (alu_op==4'h0 || u); // `ALU_CTRL_ADD
    assign alu_c[3] = (alu_op==4'h8);      // `ALU_CTRL_SUB
    assign alu_c[4] = (alu_op==4'h4);      // `ALU_CTRL_XOR
    assign alu_c[5] = (alu_op==4'h6);      // `ALU_CTRL_OR
    assign alu_c[6] = (alu_op==4'h7);      // `ALU_CTRL_AND
    assign alu_c[7] = (alu_op==4'h1);      // `ALU_CTRL_SLL
    assign alu_c[8] = (alu_op==4'h5);      // `ALU_CTRL_SRL
    assign alu_c[9] = (alu_op==4'hd);      // `ALU_CTRL_SRA
    assign alu_c[10]= ja;                  // `JAL, JALR
    /********** one-hot encoding of branch control ********************/
    assign bru_c[0] = (b & fct3==3'b000);  // BEQ
    assign bru_c[1] = (b & fct3==3'b001);  // BNE
    assign bru_c[2] = (b & fct3==3'b100);  // BLT
    assign bru_c[3] = (b & fct3==3'b101);  // BGE
    assign bru_c[4] = (b & fct3==3'b110);  // BLTU
    assign bru_c[5] = (b & fct3==3'b111);  // BGEU
    assign bru_c[6] = ja;                  // JAL, JALR  -> always taken

    m_get_imm m_get_imm1 (ir, i, s, b, u, j, w_imm);
endmodule
/********************************************************************************************/