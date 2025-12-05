`timescale 1ns / 1ps

module testbench_matmul();
  // Señales del DUT (Device Under Test)
  logic        clk;
  logic        reset;
  logic [31:0] WriteDataM, DataAdrM;
  logic        MemWriteM;
  
  logic [31:0] PCF;
  logic [31:0] InstrF;
  logic [31:0] pc_backup;
  logic        save_pc;
  logic        pc_mux_sel;
  logic [1:0]  im_sel;
  logic        start_matmul2;
  logic        end_matmul;
  logic [1:0]  fsm_state;
  
  // Variables para control de test
  integer cycle_count;
  
  // Instancia del DUT
  top dut(
    .clk(clk),
    .reset(reset),
    .WriteDataM(WriteDataM),
    .DataAdrM(DataAdrM),
    .MemWriteM(MemWriteM),
    .file_name("program.mem")  // Archivo de programa principal
  );
  
  // Generación de reloj: 10ns periodo (100MHz)
  initial begin
    clk = 0;
    forever #5 clk = ~clk;
  end
  
  // Asignación de señales internas mediante hierarchical reference
  assign PCF = dut.PCF_or_zero;
  assign InstrF = dut.InstrF;
  
  // Señales de MATMUL - ahora con las rutas correctas
  assign pc_backup = dut.pc_backup;
  assign save_pc = dut.save_pc;
  assign pc_mux_sel = dut.pc_mux_sel;
  assign im_sel = dut.im_sel;
  assign start_matmul2 = dut.start_matmul2;
  assign end_matmul = dut.end_matmul;
  assign fsm_state = dut.fsm.state;
  
  // Secuencia de reset y test
  initial begin
    $display("================================================================================");
    $display("  TESTBENCH MATMUL - Verificación de Microcodigo");
    $display("================================================================================");
    $display("\n[HEADER] Señales principales de control:");
    $display("  FSM_STATE: Estado actual del FSM (00=NORMAL, 01=MATMUL2)");
    $display("  IM_SEL: Selección de instruction memory (00=normal, 01=matmul2)");
    $display("  PC_MUX_SEL: Mux de PC (0=backup, 1=normal)");
    $display("  SAVE_PC: Habilita guardado de PC");
    $display("  RESET_PC: Resetea PC a 0");
    $display("================================================================================\n");
    
    // Inicialización
    cycle_count = 0;
    reset = 1;
    
    // Mantener reset por 2 ciclos
    #20;
    reset = 0;
    
    $display("[INFO] Reset liberado, iniciando ejecución...\n");
    
    // Ejecutar por 200 ciclos de reloj
    repeat(200) begin
      @(posedge clk);
      cycle_count = cycle_count + 1;
    end
    
    $display("\n================================================================================");
    $display("  Test completado después de %0d ciclos", cycle_count);
    $display("================================================================================");
    $finish;
  end
  
  // Monitor detallado de señales en cada ciclo
  always @(posedge clk) begin
    if (!reset) begin
      $display("[CICLO %3d] PC=%08h | Instr=%08h | FSM=%b | IM_SEL=%b | PC_MUX=%b | WB: RegWr=%b Rd=%2d Val=%08h",
               cycle_count, PCF, InstrF, fsm_state, im_sel, pc_mux_sel, 
               dut.riscv.RegWriteW, dut.riscv.RdW, dut.riscv.ResultW);
    end
  end
  
  
  // Monitor de señales de control MATMUL con estado detallado
  always @(posedge clk) begin
    if (!reset) begin
      // Detectar inicio de MATMUL
      if (start_matmul2) begin
        $display("\n╔═══════════════════════════════════════════════════════════════════╗");
        $display("║ >>> [MATMUL START] Ciclo %0d: STARTMATMUL2 detectado", 
                 cycle_count);
        $display("║     PC actual guardado: %08h", PCF);
        $display("║     PC+4 a guardar: %08h", PCF + 4);
        $display("║     Control: save_pc=%b, pc_mux_sel=%b, im_sel=%b, reset_pc=%b", 
                 save_pc, pc_mux_sel, im_sel, dut.reset_pc);
        $display("║     FSM: Estado actual=%b, próximo estado=MATMUL2(01)",
                 fsm_state);
        $display("╚═══════════════════════════════════════════════════════════════════╝\n");
      end
      
      // Detectar fin de MATMUL
      if (end_matmul) begin
        $display("\n╔═══════════════════════════════════════════════════════════════════╗");
        $display("║ <<< [MATMUL END] Ciclo %0d: ENDMATMUL detectado", cycle_count);
        $display("║     Estado FSM actual: %b (%s)", fsm_state,
                 fsm_state == 2'b01 ? "MATMUL2" : "NORMAL");
        $display("║     PC backup a restaurar: %08h", pc_backup);
        $display("║     Control: pc_mux_sel=%b (debe cambiar a 0)", pc_mux_sel);
        $display("║     Retornando a flujo normal...");
        $display("╚═══════════════════════════════════════════════════════════════════╝\n");
      end
    end
  end
  
  
  // Monitor de escrituras a memoria
  always @(posedge clk) begin
    if (!reset && MemWriteM) begin
      $display("\n    [MEM WRITE] Dirección: %08h, Dato: %08h (Ciclo %0d)",
               DataAdrM, WriteDataM, cycle_count);
    end
  end
  
  // Monitor de cambios en el PC
  logic [31:0] prev_pc;
  initial prev_pc = 0;
  
  always @(posedge clk) begin
    if (!reset) begin
      if (PCF != prev_pc + 4 && cycle_count > 0) begin
        $display("\n    [PC JUMP] De %08h a %08h (diferencia: %0d) en ciclo %0d",
                 prev_pc, PCF, $signed(PCF - prev_pc), cycle_count);
      end
      prev_pc = PCF;
    end
  end
  
  // Generación de archivo VCD para GTKWave (opcional)
  initial begin
    $dumpfile("testbench_matmul.vcd");
    $dumpvars(0, testbench_matmul);
    $dumpvars(0, dut);
  end
  
  // Monitor de instrucciones decodificadas
  logic [6:0] opcode;
  assign opcode = InstrF[6:0];
  
  always @(posedge clk) begin
    if (!reset && cycle_count > 0) begin
      case (opcode)
        7'b1111010: $display("\n>>> [DECODE] STARTMATMUL2 detectado en ciclo %0d", cycle_count);
        7'b1111100: $display("\n<<< [DECODE] ENDMATMUL detectado en ciclo %0d", cycle_count);
        7'b0110011: if (cycle_count % 10 == 0) $display("    [DECODE] R-type (Integer)");
        7'b1010011: if (cycle_count % 10 == 0) $display("    [DECODE] R-type (FPU)");
        7'b0000011: if (cycle_count % 10 == 0) $display("    [DECODE] LW");
        7'b0100011: if (cycle_count % 10 == 0) $display("    [DECODE] SW");
        7'b1100011: if (cycle_count % 10 == 0) $display("    [DECODE] BEQ");
        7'b1101111: $display("    [DECODE] JAL en ciclo %0d", cycle_count);
      endcase
    end
  end
  
  // Timeout de seguridad
  initial begin
    #100000; // 100us timeout
    $display("\n[ERROR] TIMEOUT - El test excedió el tiempo máximo de ejecución");
    $finish;
  end

endmodule
