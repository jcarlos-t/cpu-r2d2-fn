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
  logic        save_pc;
  logic        im_sel;
  logic        toggle_fsm;
  logic        fsm_state;
  
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
  assign PCF = dut.PCF;
  assign InstrF = dut.InstrF;
  
  // Señales de MATMUL - ahora con las rutas correctas
  assign pc_backup = dut.pc_backup;
  assign save_pc = dut.save_pc;
  assign im_sel = dut.im_sel;
  assign toggle_fsm = dut.toggle_fsm;
  assign fsm_state = dut.fsm.state;
  
  // Secuencia de reset y test
  initial begin
    $display("================================================================================");
    $display("  TESTBENCH MATMUL - Verificación de Microcodigo");
    $display("================================================================================");
    $display("\n[HEADER] Señales principales de control:");
    $display("  FSM_STATE: Estado actual del FSM (0=NORMAL, 1=MATMUL2)");
    $display("  IM_SEL: Selección de instruction memory (0=normal, 1=matmul2)");
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
    // Ejecutar por 43 ciclos de reloj
    repeat(43) begin
      @(posedge clk);
      cycle_count = cycle_count + 1;
    end
    
    $display("\n================================================================================");
    $display("  VERIFICACIÓN FINAL DE MEMORIA");
    $display("================================================================================");
    $display("  C[0][0] (0x60): %h %s", dut.dmem.RAM[24], (dut.dmem.RAM[24] == exp_C00) ? "✅" : "❌");
    $display("  C[0][1] (0x64): %h %s", dut.dmem.RAM[25], (dut.dmem.RAM[25] == exp_C01) ? "✅" : "❌");
    $display("  C[1][0] (0x68): %h %s", dut.dmem.RAM[26], (dut.dmem.RAM[26] == exp_C10) ? "✅" : "❌");
    $display("  C[1][1] (0x6C): %h %s", dut.dmem.RAM[27], (dut.dmem.RAM[27] == exp_C11) ? "✅" : "❌");
    
    $display("\n================================================================================");
    $display("  Test completado después de %0d ciclos", cycle_count);
    $display("================================================================================");
    $finish;
  end
  
  // Monitor detallado de señales en cada ciclo
  always @(posedge clk) begin
    if (!reset) begin
      $display("[CICLO %3d] PC=%08h | Instr=%08h | FSM=%b | WB: RegWr=%b Rd=%2d Val=%08h | MemWr=%b DataM=%08h",
               cycle_count, PCF, InstrF, fsm_state, 
               dut.riscv.RegWriteW, dut.riscv.RdW, dut.riscv.ResultW,
               MemWriteM, WriteDataM);
      
      // Monitor adicional para debug de datos eliminado
    end
  end
  
  
  // Monitor de señales de control MATMUL con estado detallado
  always @(posedge clk) begin
    if (!reset) begin
      // Detectar inicio de MATMUL (Toggle en estado NORMAL)
      if (toggle_fsm && fsm_state == 1'b0) begin
        $display("\n╔═══════════════════════════════════════════════════════════════════╗");
        $display("║ >>> [MATMUL START] Ciclo %0d: STARTMATMUL2 detectado", 
                 cycle_count);
        $display("║     PC actual guardado: %08h", PCF);
        $display("║     PC+4 a guardar: %08h", PCF + 4);
        $display("║     Control: save_pc=%b, im_sel=%b, toggle_fsm=%b", 
                 save_pc, im_sel, toggle_fsm);
        $display("║     FSM: Estado actual=%b, próximo estado=MATMUL2(1)",
                 fsm_state);
        $display("╚═══════════════════════════════════════════════════════════════════╝\n");
      end
      
      // Detectar fin de MATMUL (Toggle en estado MATMUL2)
      if (toggle_fsm && fsm_state == 1'b1) begin
        $display("\n╔═══════════════════════════════════════════════════════════════════╗");
        $display("║ <<< [MATMUL END] Ciclo %0d: ENDMATMUL detectado", cycle_count);
        $display("║     Estado FSM actual: %b (%s)", fsm_state,
                 fsm_state == 1'b1 ? "MATMUL2" : "NORMAL");
        $display("║     PC backup a restaurar: %08h", pc_backup);
        $display("║     Retornando a flujo normal...");
        $display("╚═══════════════════════════════════════════════════════════════════╝\n");
      end
    end
  end
  
  
  // Monitor de escrituras a memoria
  // Verificación de resultados esperados (C = A x B)
  // A = [[1.5, 2.3], [3.7, 4.2]]
  // B = [[5.1, 6.8], [7.4, 8.9]]
  // C = [[24.67, 30.67], [49.95, 62.54]]
  logic [31:0] exp_C00 = 32'h41c55c28; // 24.67
  logic [31:0] exp_C01 = 32'h41f55c27; // 30.67
  logic [31:0] exp_C10 = 32'h4247cccc; // 49.95
  logic [31:0] exp_C11 = 32'h427a28f4; // 62.54

  always @(posedge clk) begin
    if (!reset && MemWriteM) begin
      $display("\n    [MEM WRITE] Dirección: %08h, Dato: %08h (Ciclo %0d)",
               DataAdrM, WriteDataM, cycle_count);
               
      case (DataAdrM)
        32'h00000060: begin
          if (WriteDataM == exp_C00) $display("    [PASS] C[0][0] Correcto: %h (24.67)", WriteDataM);
          else $display("     [FAIL] C[0][0] Incorrecto: Esperado %h, Obtenido %h", exp_C00, WriteDataM);
        end
        32'h00000064: begin
          if (WriteDataM == exp_C01) $display("     [PASS] C[0][1] Correcto: %h (30.67)", WriteDataM);
          else $display("     [FAIL] C[0][1] Incorrecto: Esperado %h, Obtenido %h", exp_C01, WriteDataM);
        end
        32'h00000068: begin
          if (WriteDataM == exp_C10) $display("     [PASS] C[1][0] Correcto: %h (49.95)", WriteDataM);
          else $display("     [FAIL] C[1][0] Incorrecto: Esperado %h, Obtenido %h", exp_C10, WriteDataM);
        end
        32'h0000006C: begin
          if (WriteDataM == exp_C11) $display("     [PASS] C[1][1] Correcto: %h (62.54)", WriteDataM);
          else $display("     [FAIL] C[1][1] Incorrecto: Esperado %h, Obtenido %h", exp_C11, WriteDataM);
        end
      endcase
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
        7'b1111010: begin
          if (InstrF[14:12] == 3'b000) $display("\n>>> [DECODE] STARTMATMUL2 detectado en ciclo %0d", cycle_count);
          else if (InstrF[14:12] == 3'b111) $display("\n<<< [DECODE] ENDMATMUL detectado en ciclo %0d", cycle_count);
        end
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
