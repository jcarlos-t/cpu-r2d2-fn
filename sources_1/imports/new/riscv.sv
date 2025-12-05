module top(
  input  logic        clk, reset,
  output logic [31:0] WriteDataM, DataAdrM,
  output logic        MemWriteM,
  input logic [127:0] file_name
);
  logic [31:0] PCF, InstrF, ReadDataM;
  
  // Señales para MATMUL microcodigo
  logic [31:0] pc_backup;
  logic [31:0] pc_to_imem;
  logic        save_pc;
  logic        im_sel;
  logic        toggle_fsm;
  logic [31:0] instr_normal, instr_matmul2;
  logic [31:0] target_pc_val;

  // Instancia del procesador con señales MATMUL
  riscv riscv(
    .clk(clk),
    .reset(reset),
    .PCF(PCF),
    .InstrF(InstrF),
    .MemWriteM(MemWriteM),
    .ALUResultM(DataAdrM),
    .WriteDataM(WriteDataM),
    .ReadDataM(ReadDataM),
    .ToggleFSM(toggle_fsm),
    .TargetPC(target_pc_val)
  );
  
  // PC backup register - guarda el PC cuando inicia MATMUL
  flopenr #(32) pc_backup_reg(
    .clk(clk), 
    .reset(reset), 
    .en(save_pc), 
    .d(PCF + 4),  // Guardar PC+4 para retornar a la siguiente instrucción
    .q(pc_backup)
  );

  // FSM de control de MATMUL
  matmul_fsm fsm(
    .clk(clk),
    .reset(reset),
    .toggle_fsm(toggle_fsm),
    .im_sel(im_sel),
    .save_pc(save_pc)
  );

  // Lógica para determinar el Target PC
  // Si estamos en MATMUL2 y ocurre Toggle -> Vamos a NORMAL -> Target = pc_backup
  // Si estamos en NORMAL y ocurre Toggle -> Vamos a MATMUL -> Target = 0
  assign target_pc_val = (fsm.state == 1'b1) ? pc_backup : 32'b0;
  


  // Tres instruction memories
  imem imem_normal(PCF, instr_normal, file_name);
  imem imem_matmul2(PCF, instr_matmul2, "matmul2.mem");

  // Mux de instrucciones: selecciona de qué memoria leer
  // Mux de instrucciones: selecciona de qué memoria leer
  mux2 #(32) instr_mux(
    .d0(instr_normal),
    .d1(instr_matmul2),
    .s(im_sel),
    .y(InstrF)
  );
  
  // Data memory
  dmem dmem(clk, MemWriteM, DataAdrM, WriteDataM, ReadDataM);
endmodule

// procesador pipeline
module riscv(
  input  logic        clk, reset,
  output logic [31:0] PCF,
  input  logic [31:0] InstrF,
  output logic        MemWriteM,
  output logic [31:0] ALUResultM, WriteDataM,
  input  logic [31:0] ReadDataM,
  // Señales de control para MATMUL
  output logic        ToggleFSM,
  input  logic [31:0] TargetPC
);
  logic [6:0]  opD;
  logic [2:0]  funct3D;
  logic [6:0]  funct7D;
  logic [1:0]  ImmSrcD;
  logic        ZeroE;
  logic        PCSrcE;
  logic [2:0]  ALUControlE;
  logic        ALUSrcE;
  logic        ResultSrcEb0;
  logic        RegWriteM;
  logic [1:0]  ResultSrcW;
  logic        RegWriteW;
  logic [1:0]  ForwardAE, ForwardBE;
  logic        StallF, StallD, FlushD, FlushE;
  logic [4:0]  Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW, RdD;
  logic        isFPD, isFPE, isFPM, isFPW; // FPU ALU usage en cada etapa
  logic        useFP_RF_D, useFP_RF_E, useFP_RF_M, useFP_RF_W; // FP register file usage
  logic        round_modeE; // modo de redondeo para FPU

  logic [31:0] RD1D_int, RD2D_int;
  logic [31:0] RD1D_fp, RD2D_fp;
  
  logic StartMatmul2, EndMatmul, ResetPC;
  logic RegWriteE; // Exposed from controller

  controller c(
    clk, reset,
    opD, funct3D, funct7D, ImmSrcD,
    FlushE, ZeroE, PCSrcE, ALUControlE, ALUSrcE, ResultSrcEb0,
    isFPD, isFPE, round_modeE, useFP_RF_D, useFP_RF_E,
    MemWriteM, RegWriteM, isFPM, useFP_RF_M,
    RegWriteW, RegWriteE, ResultSrcW, isFPW, useFP_RF_W,
    StartMatmul2, EndMatmul, ResetPC, ToggleFSM,
    StallD
  );

  logic [31:0] ALUResultE, ResultW; // Exposed from datapath

  datapath dp(
    clk, reset,
    StallF, PCF, InstrF,
    opD, funct3D, funct7D, StallD, FlushD, ImmSrcD,
    FlushE, ForwardAE, ForwardBE, PCSrcE, ALUControlE,
    ALUSrcE, isFPD, isFPE, round_modeE, ZeroE,
    MemWriteM, WriteDataM, ALUResultM, ALUResultE, ResultW, ReadDataM,
    RegWriteW, useFP_RF_W, ResultSrcW,
    Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW, RdD,
    RD1D_int, RD2D_int, RD1D_fp, RD2D_fp,
    ToggleFSM, TargetPC
  );

  hazard hu(
    Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW,
    PCSrcE, ResultSrcEb0, RegWriteM, RegWriteW,
    isFPD, useFP_RF_D, useFP_RF_E, useFP_RF_M, useFP_RF_W,
    ForwardAE, ForwardBE, StallF, StallD, FlushD, FlushE,
    ToggleFSM
  );

  // ===========================================================================
  // Shadow Write Logic & Register Files (Moved here for signal visibility)
  // ===========================================================================
  
  // Señales para shadow write de MATMUL
  logic we_matmul_int, we_matmul_fp;
  logic [31:0] matmul_addr_A, matmul_addr_B, matmul_addr_C;
  
  // Generar señal de shadow write cuando detecta MATMUL
  assign we_matmul_int = (StartMatmul2) & ~isFPD;
  assign we_matmul_fp  = (StartMatmul2) & isFPD;
  
  // Capturar direcciones desde los valores leídos del regfile
  // Forwarding logic for Shadow Write (Decode Stage)
  // Prioridad: Execute > Memory > Writeback > RegFile
  
  // Forward A (Rs1D)
  always_comb begin
    if ((Rs1D != 0) && (Rs1D == RdE) && RegWriteE) matmul_addr_A = ALUResultE;
    else if ((Rs1D != 0) && (Rs1D == RdM) && RegWriteM) matmul_addr_A = ALUResultM;
    else if ((Rs1D != 0) && (Rs1D == RdW) && RegWriteW) matmul_addr_A = ResultW;
    else matmul_addr_A = RD1D_int;
  end
  
  // Forward B (Rs2D)
  always_comb begin
    if ((Rs2D != 0) && (Rs2D == RdE) && RegWriteE) matmul_addr_B = ALUResultE;
    else if ((Rs2D != 0) && (Rs2D == RdM) && RegWriteM) matmul_addr_B = ALUResultM;
    else if ((Rs2D != 0) && (Rs2D == RdW) && RegWriteW) matmul_addr_B = ResultW;
    else matmul_addr_B = RD2D_int;
  end
  
  // Para rd necesitamos leer su valor actual
  logic [31:0] rd_value;
  assign rd_value = (RdD == Rs1D) ? RD1D_int : 
                    (RdD == Rs2D) ? RD2D_int : 
                    32'h00000060; // Default: dirección de C hardcodeada
  assign matmul_addr_C = rd_value;
  
  // Register files separados: Int (x0=0) y FP (f0 puede ser != 0)
  regfile #(1) rf_int(
    .clk(clk), 
    .we3(RegWriteW & ~useFP_RF_W), 
    .a1(Rs1D), .a2(Rs2D), .a3(RdW), 
    .wd3(ResultW), 
    .rd1(RD1D_int), .rd2(RD2D_int),
    // Shadow write para MATMUL
    .we_matmul(we_matmul_int),
    .addr_A(matmul_addr_A),
    .addr_B(matmul_addr_B),
    .addr_C(matmul_addr_C)
  );
  
  regfile #(0) rf_fp(
    .clk(clk), 
    .we3(RegWriteW & useFP_RF_W), 
    .a1(Rs1D), .a2(Rs2D), .a3(RdW), 
    .wd3(ResultW), 
    .rd1(RD1D_fp), .rd2(RD2D_fp),
    // Shadow write para MATMUL (FP también puede necesitarlo)
    .we_matmul(we_matmul_fp),
    .addr_A(matmul_addr_A),
    .addr_B(matmul_addr_B),
    .addr_C(matmul_addr_C)
  );

endmodule

module controller(
  input  logic clk, reset,
  // Señales de control de Decode
  input  logic [6:0] opD,
  input  logic [2:0] funct3D,
  input  logic [6:0] funct7D,
  output logic [1:0] ImmSrcD,
  // Señales de control de Execute
  input  logic       FlushE,
  input  logic       ZeroE,
  output logic       PCSrcE, // pc next: 1-> branching, 0 -> pc+4
  // Hazard Unit
  output logic [2:0] ALUControlE,
  output logic       ALUSrcE,
  output logic       ResultSrcEb0, // para Hazard Unit
  output logic       isFPD, isFPE, // usa FPU ALU
  output logic       round_modeE, // modo de redondeo para FPU
  output logic       useFP_RF_D, useFP_RF_E, // usa FP register file
  output logic       MemWriteM,
  output logic       RegWriteM, isFPM, useFP_RF_M, // para Hazard Unit
  output logic       RegWriteW, // para datapath and
  output logic       RegWriteE, // Exposed for Shadow Write forwarding
  output logic [1:0] ResultSrcW,
  output logic       isFPW, useFP_RF_W,
  // Señales para MATMUL microcodigo
  output logic       StartMatmul2, EndMatmul, ResetPC, ToggleFSM,
  input  logic       StallD
);
  // señales de pipelined
  logic       RegWriteD; // RegWriteE is now an output
  logic [1:0] ResultSrcD, ResultSrcE, ResultSrcM;
  logic       MemWriteD, MemWriteE;
  logic       JumpD, JumpE;
  logic       BranchD, BranchE;
  logic [1:0] ALUOpD;
  logic [2:0] ALUControlD;
  logic       ALUSrcD;
  logic       round_modeD;

  // useFP_RF_D: usa FP register file (FPU ALU ops + FLW + FSW)
  assign useFP_RF_D = (opD[6:5] == 2'b10) | (opD == 7'b0000111) | (opD == 7'b0100111);
  assign isFPD = useFP_RF_D;  // alias para compatibilidad
  
  assign round_modeD = funct3D[0]; // bit 0 de funct3, modo de redondeo

  // Detección de instrucciones MATMUL (Tipo R con opcode compartido)
  // opcode = 7'b1111010, diferenciadas por funct7 y funct3
  logic is_matmul_op;
  assign is_matmul_op = (opD == 7'b1111010);
  
  assign StartMatmul2 = is_matmul_op && (funct7D == 7'b0000000) && (funct3D == 3'b000);
  assign EndMatmul    = is_matmul_op && (funct7D == 7'b1111111) && (funct3D == 3'b111);
  assign ResetPC      = StartMatmul2 & ~StallD;
  assign ToggleFSM    = (StartMatmul2 | EndMatmul) & ~StallD;

  // Decode stage
  maindec md(
    opD, ResultSrcD, MemWriteD, BranchD,
    ALUSrcD, RegWriteD, JumpD, ImmSrcD, ALUOpD
  );
  aludec ad(opD[5], funct3D, funct7D, ALUOpD, ALUControlD);

  // Registro de señales de control -> Execute
  // isFPE: solo para FPU ALU (opcode[6:5]=10), NO para FLW/FSW
  // useFP_RF_E: para FPU ALU + FLW/FSW (marca uso de FP register file)
  logic isFPD_useALU;  // usa FPU ALU (no FLW/FSW)
  assign isFPD_useALU = (opD[6:5] == 2'b10);
  
  floprc #(13) controlregE(
    clk, reset, FlushE,
    {RegWriteD, ResultSrcD, MemWriteD, JumpD, BranchD, ALUControlD, ALUSrcD, isFPD_useALU, round_modeD, useFP_RF_D},
    {RegWriteE, ResultSrcE, MemWriteE, JumpE, BranchE, ALUControlE, ALUSrcE, isFPE, round_modeE, useFP_RF_E}
  );

  assign PCSrcE      = (BranchE & ZeroE) | JumpE;
  assign ResultSrcEb0 = ResultSrcE[0];

  // Registro de señales de control -> Memory
  flopr #(6) controlregM(
    clk, reset,
    {RegWriteE, ResultSrcE, MemWriteE, isFPE, useFP_RF_E},
    {RegWriteM, ResultSrcM, MemWriteM, isFPM, useFP_RF_M}
  );

  // Registro de señales de control -> Writeback
  flopr #(5) controlregW(
    clk, reset,
    {RegWriteM, ResultSrcM, isFPM, useFP_RF_M},
    {RegWriteW, ResultSrcW, isFPW, useFP_RF_W}
  );
endmodule

module maindec(
  input  logic [6:0] op,
  output logic [1:0] ResultSrc,
  output logic       MemWrite,
  output logic       Branch, ALUSrc,
  output logic       RegWrite, Jump,
  output logic [1:0] ImmSrc,
  output logic [1:0] ALUOp
);
  logic [10:0] controls;

  assign {RegWrite, ImmSrc, ALUSrc, MemWrite,
          ResultSrc, Branch, ALUOp, Jump} = controls;

  always_comb
    case (op)
      // RegWrite_ImmSrc_ALUSrc_MemWrite_ResultSrc_Branch_ALUOp_Jump
      7'b0000011: controls = 11'b1_00_1_0_01_0_00_0; // lw (integer)
      7'b0100011: controls = 11'b0_01_1_1_00_0_00_0; // sw (integer)
      7'b0110011: controls = 11'b1_xx_0_0_00_0_10_0; // R-type (integer)
      7'b1100011: controls = 11'b0_10_0_0_00_1_01_0; // beq
      7'b0010011: controls = 11'b1_00_1_0_00_0_10_0; // I-type ALU (integer)
      7'b1101111: controls = 11'b1_11_0_0_10_0_00_1; // jal
      7'b0000111: controls = 11'b1_00_1_0_01_0_00_0; // flw (FP load)
      7'b0100111: controls = 11'b0_01_1_1_00_0_00_0; // fsw (FP store)
      7'b0000000: controls = 11'b0_00_0_0_00_0_00_0; // señales para el reset
      7'b1010011: controls = 11'b1_xx_0_0_00_0_11_0; // R-type FPU
      // Instrucciones MATMUL (R-type, opcode único, diferenciadas por funct7/funct3)
      // Formato: matmul2/matmul3/endmatmul rd, rs1, rs2
      7'b1111010: controls = 11'b1_00_0_0_00_0_00_0; // MATMUL (tipo R, puede escribir en rd)
      default:    controls = 11'bx_xx_x_x_xx_x_xx_x; 
    endcase
endmodule

module aludec(
  input  logic       opb5,
  input  logic [2:0] funct3,
  input  logic [6:0] funct7,
  input  logic [1:0] ALUOp,
  output logic [2:0] ALUControl
);
  logic RtypeSub;

  assign RtypeSub = funct7[5] & opb5; // 1 -> resta R-type

  always_comb
  case (ALUOp)
    2'b00: ALUControl = 3'b000; // adicion
    2'b01: ALUControl = 3'b001; // sustraccion
    2'b10: begin  // R-type or I-type ALU (Integer)
      case (funct3)
        3'b000: begin
          if (RtypeSub)
            ALUControl = 3'b001;  // SUB
          else
            ALUControl = 3'b000;  // ADD, ADDI
        end
        3'b010: ALUControl = 3'b101;  // SLT, SLTI
        3'b110: ALUControl = 3'b011;  // OR, ORI
        3'b111: ALUControl = 3'b010;  // AND, ANDI
        default: ALUControl = 3'bxxx;
      endcase
    end
    2'b11: begin // FP operations
      if (funct7[4]) begin
        case (funct3)
          3'b000: ALUControl = 3'b100; // MIN
          3'b001: ALUControl = 3'b101; // MAX
          default: ALUControl = 3'bxxx;
        endcase
      end else begin
        case (funct7[3:2])
          2'b00: ALUControl = 3'b000; // addf
          2'b01: ALUControl = 3'b001; // subf
          2'b10: ALUControl = 3'b010; // mulf
          2'b11: ALUControl = 3'b011; // divf
          default: ALUControl = 3'bxxx;
        endcase
      end
    end
    default: ALUControl = 3'bxxx;
  endcase
endmodule


module datapath(
  input  logic        clk, reset,
  // Señales de Fetch 
  input  logic        StallF,
  output logic [31:0] PCF,
  input  logic [31:0] InstrF,
  // Señales de Decode
  output logic [6:0]  opD,
  output logic [2:0]  funct3D,
  output logic [6:0]  funct7D,
  input  logic        StallD, FlushD,
  input  logic [1:0]  ImmSrcD,
  // Señales de Execute
  input  logic        FlushE,
  input  logic [1:0]  ForwardAE, ForwardBE,
  input  logic        PCSrcE,
  input  logic [2:0]  ALUControlE,
  input  logic        ALUSrcE,
  input  logic        isFPD, isFPE, // floating point (Decode y Execute)
  input  logic        round_modeE, // modo de redondeo
  output logic        ZeroE,
  // Señales de Memory
  input  logic        MemWriteM,
  output logic [31:0] WriteDataM, ALUResultM,
  output logic [31:0] ALUResultE, // Exposed for Shadow Write forwarding
  output logic [31:0] ResultW,    // Exposed for Shadow Write forwarding
  input  logic [31:0] ReadDataM,
  // Señales de Writeback
  input  logic        RegWriteW, useFP_RF_W,
  input  logic [1:0]  ResultSrcW,
  // Señales de Hazard Unit
  output logic [4:0]  Rs1D, Rs2D, Rs1E, Rs2E,
  output logic [4:0]  RdE, RdM, RdW,
  output logic [4:0]  RdD, // Necesario para shadow write logic
  // Entradas de Register Files (desde módulo riscv)
  input  logic [31:0] RD1D_int, RD2D_int,
  input  logic [31:0] RD1D_fp, RD2D_fp,
  input  logic        ForcePC, // Fuerza salto a TargetPC
  input  logic [31:0] TargetPC // Dirección de salto forzado
);
  // Fetch
  logic [31:0] PCNextF, PCPlus4F, PCBranchF;

  // Decode
  logic [31:0] InstrD;
  logic [31:0] PCD, PCPlus4D;
  // RD1D_int, RD2D_int, RD1D_fp, RD2D_fp son inputs ahora
  logic [31:0] ImmExtD;
  // RdD es output ahora

  // Execute
  logic [31:0] RD1E_int, RD2E_int;  // Integer operands
  logic [31:0] RD1E_fp, RD2E_fp;    // FP operands
  logic [31:0] PCE, ImmExtE;
  logic [31:0] SrcAE_int, SrcBE_int;  // Integer ALU inputs
  logic [31:0] SrcAE_fp, SrcBE_fp;    // FP ALU inputs
  // logic [31:0] ALUResultE; // Now an output port
  logic [31:0] WriteDataE_int, WriteDataE_fp;
  logic [31:0] PCPlus4E;
  logic [31:0] PCTargetE;

  // Memory
  logic [31:0] PCPlus4M;

  // Writeback
  logic [31:0] ALUResultW;
  logic [31:0] ReadDataW;
  logic [31:0] PCPlus4W;
  // logic [31:0] ResultW; // Now an output port

  // Etapa de Fetch
  // Lógica de PC Next: ResetPC > Branch/Jump > PC+4
  mux2 #(32)    pcmux_branch(PCPlus4F, PCTargetE, PCSrcE, PCBranchF);
  mux2 #(32)    pcmux_force(PCBranchF, TargetPC, ForcePC, PCNextF);
  
  flopenr #(32) pcreg(clk, reset, ~StallF, PCNextF, PCF);
  adder         pcadd(PCF, 32'h4, PCPlus4F);

  // Etapa de Decode
  flopenrc #(96) regD(
    clk, reset, FlushD, ~StallD,
    {InstrF, PCF, PCPlus4F},
    {InstrD, PCD, PCPlus4D}
  ); // 1 registro pipeline

  assign opD        = InstrD[6:0];
  assign funct3D    = InstrD[14:12];
  assign funct7D    = InstrD[31:25];
  assign Rs1D       = InstrD[19:15];
  assign Rs2D       = InstrD[24:20];
  assign RdD        = InstrD[11:7];
  
  // Shadow write logic and regfiles moved to end of module
  
  extend  ext(InstrD[31:7], ImmSrcD, ImmExtD);

  // Execute - pipeline register con operandos Int y FP separados
  floprc #(239) regE(
    clk, reset, FlushE,
    {RD1D_int, RD2D_int, RD1D_fp, RD2D_fp, PCD, Rs1D, Rs2D, RdD, ImmExtD, PCPlus4D},
    {RD1E_int, RD2E_int, RD1E_fp, RD2E_fp, PCE, Rs1E, Rs2E, RdE, ImmExtE, PCPlus4E}
  ); // 2 registro pipeline: 32*4 + 32 + 5*3 + 32*3 = 128+32+15+96 = 271? Corregir: 32+32+32+32+32+5+5+5+32+32=239

  // Forwarding muxes para Integer path
  mux3 #(32) faemux_int(RD1E_int, ResultW, ALUResultM, ForwardAE, SrcAE_int);
  mux3 #(32) fbemux_int(RD2E_int, ResultW, ALUResultM, ForwardBE, WriteDataE_int);
  mux2 #(32) srcbmux_int(WriteDataE_int, ImmExtE, ALUSrcE, SrcBE_int);
  
  // Forwarding muxes para FP path
  mux3 #(32) faemux_fp(RD1E_fp, ResultW, ALUResultM, ForwardAE, SrcAE_fp);
  mux3 #(32) fbemux_fp(RD2E_fp, ResultW, ALUResultM, ForwardBE, WriteDataE_fp);
  mux2 #(32) srcbmux_fp(WriteDataE_fp, ImmExtE, ALUSrcE, SrcBE_fp);
  
  // ALU entera
  wire [31:0] intResult;
  wire        intZero;
  alu alu_u(SrcAE_int, SrcBE_int, ALUControlE, intResult, intZero);

  // FPU ALU
  wire [31:0] fpuResult;
  fpu_alu fpu_u(SrcAE_fp, SrcBE_fp, ALUControlE, round_modeE, fpuResult);

  // seleccionar resultado (Zero solo de Int ALU, FPU no genera Zero)
  assign ALUResultE = isFPE ? fpuResult : intResult;
  assign ZeroE      = intZero;

  adder      branchadd(ImmExtE, PCE, PCTargetE);

  // Memory - WriteData puede ser Int o FP, se selecciona según isFPE
  wire [31:0] WriteDataE = isFPE ? WriteDataE_fp : WriteDataE_int;
  flopr #(101) regM(
    clk, reset,
    {ALUResultE, WriteDataE, RdE, PCPlus4E},
    {ALUResultM, WriteDataM, RdM, PCPlus4M}
  ); // 3 registro pipeline

  // Writeback
  flopr #(101) regW(
    clk, reset,
    {ALUResultM, ReadDataM, RdM, PCPlus4M},
    {ALUResultW, ReadDataW, RdW, PCPlus4W}
  ); // 4 registro pipeline

  mux3 #(32) resultmux(
    ALUResultW, ReadDataW, PCPlus4W, ResultSrcW, ResultW
  );
endmodule


// Hazard Unit: forward, stall y flush
module hazard(
  input  logic [4:0] Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW,
  input  logic       PCSrcE, ResultSrcEb0,
  input  logic       RegWriteM, RegWriteW,
  input  logic       isFPD, useFP_RF_D, useFP_RF_E, useFP_RF_M, useFP_RF_W,
  output logic [1:0] ForwardAE, ForwardBE,
  output logic       StallF, StallD, FlushD, FlushE,
  input  logic       ToggleFSM
);
  logic lwStallD;

  // forwarding logic - solo forward entre instrucciones que usan el MISMO register file
  always_comb begin
    ForwardAE = 2'b00;
    ForwardBE = 2'b00;

    // Para Int: Rs1E != 0, para FP: siempre puede ser 0
    if (useFP_RF_E || Rs1E != 5'b0) begin
      if ((Rs1E == RdM) & RegWriteM & (useFP_RF_E == useFP_RF_M))      ForwardAE = 2'b10;
      else if ((Rs1E == RdW) & RegWriteW & (useFP_RF_E == useFP_RF_W)) ForwardAE = 2'b01;
    end

    if (useFP_RF_E || Rs2E != 5'b0) begin
      if ((Rs2E == RdM) & RegWriteM & (useFP_RF_E == useFP_RF_M))      ForwardBE = 2'b10;
      else if ((Rs2E == RdW) & RegWriteW & (useFP_RF_E == useFP_RF_W)) ForwardBE = 2'b01;
    end
  end

  // stalls y flushes - solo stall si ambas inst usan el mismo register file
  assign lwStallD = ResultSrcEb0 & ((Rs1D == RdE) | (Rs2D == RdE)) & (isFPD == useFP_RF_E);
  assign StallD   = lwStallD;
  assign StallF   = lwStallD;
  assign FlushD   = PCSrcE | ToggleFSM;
  assign FlushE   = lwStallD | PCSrcE;
endmodule


// FSM para control de MATMUL microcodigo
module matmul_fsm(
  input  logic       clk, reset,
  input  logic       toggle_fsm,
  output logic       im_sel,
  output logic       save_pc
);
  typedef enum logic {
    NORMAL  = 1'b0,
    MATMUL2 = 1'b1
  } state_t;
  
  state_t state, next_state;
  
  // State register
  always_ff @(posedge clk or posedge reset) begin
    if (reset)
      state <= NORMAL;
    else
      state <= next_state;
  end
  
  // Next state logic
  always_comb begin
    next_state = state;
    case (state)
      NORMAL: begin
        if (toggle_fsm)
          next_state = MATMUL2;
      end
      MATMUL2: begin
        if (toggle_fsm)
          next_state = NORMAL;
      end
      default: next_state = NORMAL;
    endcase
  end
  
  // Output logic
  always_comb begin
    case (state)
      NORMAL: begin
        im_sel = 1'b0;      // Instruction memory normal
        save_pc = toggle_fsm; // Save PC when starting MATMUL (toggle from NORMAL)
      end
      MATMUL2: begin
        im_sel = 1'b1;      // Instruction memory matmul2
        save_pc = 1'b0;
      end
      default: begin
        im_sel = 1'b0;
        save_pc = 1'b0;
      end
    endcase
  end
endmodule


module regfile #(parameter ZERO_REG_0 = 1) (
  input  logic        clk,
  input  logic        we3,
  input  logic [ 4:0] a1, a2, a3,
  input  logic [31:0] wd3,
  output logic [31:0] rd1, rd2,
  // Shadow write para MATMUL
  input  logic        we_matmul,
  input  logic [31:0] addr_A, addr_B, addr_C
);
  // Regfile estándar de 32 registros (5 bits de direccionamiento)
  logic [31:0] rf[31:0];

  // a1, a2, a3 -> direcciones
  // wd3 -> valor de escritura
  // se escribe en el negedge del reloj, permite write|read
  always_ff @(negedge clk) begin
    // Escritura normal
    if (we3) rf[a3] <= wd3;
    
    // Shadow write para MATMUL en registros reservados
    // Usa últimos 3 registros: x29, x30, x31
    if (we_matmul) begin
      rf[29] <= addr_C;  // x29 = MATMUL_C (dirección resultado)
      rf[30] <= addr_A;  // x30 = MATMUL_A (dirección matriz A)
      rf[31] <= addr_B;  // x31 = MATMUL_B (dirección matriz B)
    end
  end

  assign rd1 = (ZERO_REG_0 && a1 == 0) ? 32'h0 : rf[a1];
  assign rd2 = (ZERO_REG_0 && a2 == 0) ? 32'h0 : rf[a2];
endmodule


module adder(
  input  [31:0] a, b,
  output [31:0] y
);
  assign y = a + b;
endmodule


module extend(
  input  logic [31:7] instr,
  input  logic [1:0]  immsrc,
  output logic [31:0] immext
);
  always_comb
    case (immsrc)
      // I-type
      2'b00: immext = {{20{instr[31]}}, instr[31:20]};
      // S-type (stores)
      2'b01: immext = {{20{instr[31]}}, instr[31:25], instr[11:7]};
      // B-type (branches)
      2'b10: immext = {{20{instr[31]}}, instr[7], instr[30:25],
                        instr[11:8], 1'b0};
      // J-type (jal)
      2'b11: immext = {{12{instr[31]}}, instr[19:12], instr[20],
                        instr[30:21], 1'b0};
      default: immext = 32'bx;
    endcase
endmodule


module flopr #(parameter WIDTH = 8) (
  input  logic              clk, reset,
  input  logic [WIDTH-1:0]  d,
  output logic [WIDTH-1:0]  q
);
  always_ff @(posedge clk, posedge reset)
    if (reset) q <= 0;
    else       q <= d;
endmodule


module flopenr #(parameter WIDTH = 8) (
  input  logic              clk, reset, en,
  input  logic [WIDTH-1:0]  d,
  output logic [WIDTH-1:0]  q
);
  always_ff @(posedge clk, posedge reset)
    if (reset) q <= 0;
    else if (en) q <= d;
endmodule


module flopenrc #(parameter WIDTH = 8) (
  input  logic              clk, reset, clear, en,
  input  logic [WIDTH-1:0]  d,
  output logic [WIDTH-1:0]  q
);
  always_ff @(posedge clk, posedge reset)
    if (reset) q <= 0;
    else if (en)
      if (clear) q <= 0;
      else       q <= d;
endmodule


module floprc #(parameter WIDTH = 8) (
  input  logic              clk,
  input  logic              reset,
  input  logic              clear,
  input  logic [WIDTH-1:0]  d,
  output logic [WIDTH-1:0]  q
);
  always_ff @(posedge clk, posedge reset)
    if (reset) q <= 0;
    else if (clear) q <= 0;
    else            q <= d;
endmodule


module mux2 #(parameter WIDTH = 8) (
  input  logic [WIDTH-1:0] d0, d1,
  input  logic             s,
  output logic [WIDTH-1:0] y
);
  assign y = s ? d1 : d0;
endmodule


module mux3 #(parameter WIDTH = 8) (
  input  logic [WIDTH-1:0] d0, d1, d2,
  input  logic [1:0]       s,
  output logic [WIDTH-1:0] y
);
  assign y = s[1] ? d2 : (s[0] ? d1 : d0);
endmodule


module imem(
  input  logic [31:0] a,
  output logic [31:0] rd,
  input  logic [127:0] file_name
);
  logic [31:0] RAM[63:0];

initial begin
    if (file_name != 0)
      $readmemh(file_name, RAM);
    else
      $display("No file provided for instruction memory.");
  end

  assign rd = RAM[a[31:2]]; // word aligned
endmodule


module dmem(
  input  logic        clk, we,
  input  logic [31:0] a, wd,
  output logic [31:0] rd
);
  logic [31:0] RAM[63:0];
  
  // Inicialización con matrices de prueba para MATMUL
  initial begin
    // Inicializar todo a 0
    for (int i = 0; i < 64; i = i + 1) begin
      RAM[i] = 32'h00000000;
    end
    
    // ========== Matriz A (2x2) - Base dirección 0x40 (word 16) ==========
    // A = [[1.5, 2.3],
    //      [3.7, 4.2]]
    RAM[16] = 32'h3FC00000;  // A[0][0] = 1.5 (addr 0x40)
    RAM[17] = 32'h40133333;  // A[0][1] = 2.3 (addr 0x44)
    RAM[18] = 32'h406CCCCD;  // A[1][0] = 3.7 (addr 0x48)
    RAM[19] = 32'h40866666;  // A[1][1] = 4.2 (addr 0x4C)
    
    // ========== Matriz B (2x2) - Base dirección 0x50 (word 20) ==========
    // B = [[5.1, 6.8],
    //      [7.4, 8.9]]
    RAM[20] = 32'h40A33333;  // B[0][0] = 5.1 (addr 0x50)
    RAM[21] = 32'h40D9999A;  // B[0][1] = 6.8 (addr 0x54)
    RAM[22] = 32'h40ECCCCD;  // B[1][0] = 7.4 (addr 0x58)
    RAM[23] = 32'h410E6666;  // B[1][1] = 8.9 (addr 0x5C)
    
  end

  assign rd = RAM[a[31:2]]; // word aligned

  always_ff @(posedge clk)
    if (we) RAM[a[31:2]] <= wd;
endmodule


module alu(
  input  logic [31:0] a, b,
  input  logic [2:0]  alucontrol,
  output logic [31:0] result,
  output logic        zero
);
  logic [31:0] condinvb, sum;
  logic        v;          // overflow
  logic        isAddSub;   // 1 -> es add or sub

  assign condinvb = alucontrol[0] ? ~b : b;
  assign sum      = a + condinvb + alucontrol[0];
  assign isAddSub = ~alucontrol[2] & ~alucontrol[1] |
                    ~alucontrol[1] & alucontrol[0];

  always_comb
    case (alucontrol)
      3'b000: result = sum;          // add
      3'b001: result = sum;          // subtract
      3'b010: result = a & b;        // and
      3'b011: result = a | b;        // or
      3'b100: result = a ^ b;        // xor
      3'b101: result = sum[31] ^ v;  // slt
      3'b110: result = a << b[4:0];  // sll
      3'b111: result = a >> b[4:0];  // srl
      default: result = 32'bx;
    endcase

  assign zero = (result == 32'b0);
  assign v    = ~(alucontrol[0] ^ a[31] ^ b[31]) & (a[31] ^ sum[31]) & isAddSub;
endmodule

module fpu_alu (
  input  logic [31:0] a, b,
  input  logic [2:0]  alucontrol,  // 000=add, 001=sub, 010=mul, 011=div, 100=MIN, 101=MAX
  input  logic        round_mode,  // 0=truncate, 1=nearest-even
  output logic [31:0] result
);
  wire [31:0] y_add, y_mul, y_div;
  wire [4:0]  flags_add, flags_mul, flags_div;

  wire [31:0] b_sub = { b[31] ^ alucontrol[0], b[30:0] };

  fadd_core  fadd_u (.round_mode(round_mode), .a(a), .b(b_sub), .y(y_add), .flags(flags_add));
  fmul_core  fmul_u (.round_mode(round_mode), .a(a), .b(b),     .y(y_mul), .flags(flags_mul));
  fdiv_core  fdiv_u (.round_mode(round_mode), .a(a), .b(b),     .y(y_div), .flags(flags_div));

  // Unpack IEEE-754 single (NEXP=8, NSIG=23)
  logic signed [9:0] aExp, bExp;
  logic [23:0]       aSig, bSig;
  wire  [5:0]        aFlags, bFlags; // indices: 0=NORMAL,1=SUBNORMAL,2=ZERO,3=INFINITY,4=QNAN,5=SNAN

  unpacker #(8,23) unpa(a, aExp, aSig, aFlags);
  unpacker #(8,23) unpb(b, bExp, bSig, bFlags);

  // helpers
  function logic anyNaN(input [5:0] f);
    anyNaN = f[4] | f[5];
  endfunction

  // devuelve 1 si x < y (orden total ignorando NaN, para iguales devuelve 0)
  function logic fp_less_unpacked(
    input logic sx, input logic sy,
    input signed [9:0] ex, input signed [9:0] ey,
    input logic [23:0] sxmant, input logic [23:0] symant
  );
    begin
      // si signos distintos: negativo < positivo
      if (sx != sy) begin
        fp_less_unpacked = sx; // 1 si x negativo
        return fp_less_unpacked;
      end

      // mismos signos
      if (ex != ey) begin
        if (sx == 1'b0) // positivos: menor exponente -> menor
          fp_less_unpacked = (ex < ey);
        else // negativos: mayor exponente -> menor valor
          fp_less_unpacked = (ex > ey);
        return fp_less_unpacked;
      end

      // mismo exponente: comparar mantisas (incluye subnormals)
      if (sx == 1'b0)
        fp_less_unpacked = (sxmant < symant);
      else
        fp_less_unpacked = (sxmant > symant);
    end
  endfunction

  // comparar magnitud igual (sin signo): true si magnitudes idénticas
  function logic mag_equal(input logic [23:0] m1, input logic [23:0] m2, input signed [9:0] e1, input signed [9:0] e2);
    mag_equal = (e1 == e2) && (m1 == m2);
  endfunction

  always_comb begin
    case (alucontrol)
      3'b000: result = y_add;
      3'b001: result = y_add; // sub via b_sub
      3'b010: result = y_mul;
      3'b011: result = y_div;

      // MIN / MAX con manejo NaN y +0/-0 según convención (si un operando NaN retorna el otro; ambos NaN -> a)
      3'b100: begin // MIN
        if (anyNaN(aFlags) && anyNaN(bFlags)) result = a;
        else if (anyNaN(aFlags)) result = b;
        else if (anyNaN(bFlags)) result = a;
        else begin
          // ambos no-NaN: comparar
          if (mag_equal(aSig, bSig, aExp, bExp)) begin
            // igual valor/magnitud: preferir negativo (MIN -> -0 si existe)
            result = a[31] ? a : b;
          end else if (fp_less_unpacked(a[31], b[31], aExp, bExp, aSig, bSig))
            result = a;
          else
            result = b;
        end
      end

      3'b101: begin // MAX
        if (anyNaN(aFlags) && anyNaN(bFlags)) result = a;
        else if (anyNaN(aFlags)) result = b;
        else if (anyNaN(bFlags)) result = a;
        else begin
          if (mag_equal(aSig, bSig, aExp, bExp)) begin
            // igual valor/magnitud: preferir positivo (MAX -> +0 si existe)
            result = a[31] ? b : a;
          end else if (fp_less_unpacked(a[31], b[31], aExp, bExp, aSig, bSig))
            result = b;
          else
            result = a;
        end
      end

      default: result = 32'bx;
    endcase
  end
endmodule