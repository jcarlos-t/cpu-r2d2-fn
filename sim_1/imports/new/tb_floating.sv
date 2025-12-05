// Testbench para verificar:
// 1. FLW/FSW (load/store FP)
// 2. Register files separados (Int x0-x31, FP f0-f31)
// 3. Hazard forwarding entre FP
// 4. No-forwarding entre Int y FP
// 5. Stall logic (FP load-use)

module tb_floating();
  logic clk;
  logic reset;
  logic [31:0] WriteData, DataAdr;
  logic MemWrite;

  // Nombre de archivo
  logic [127:0] mem_file = "fptest.mem";

  // pipeline
  top dut(clk, reset, WriteData, DataAdr, MemWrite, mem_file);

  // Contadores
  int cycle = 0;
  
  initial begin
    // Preload data memory con valores FP
    #1;
    dut.dmem.RAM[0] = 32'h3f400000; // 0.75 en FP32
    dut.dmem.RAM[1] = 32'hbfc00000; // -1.5 en FP32
    dut.dmem.RAM[2] = 32'h40000000; // 2.0 en FP32
    $display("================================================");
    $display("TB: Preloaded Data Memory");
    $display("  dmem[0] = 0x%08h (0.75)", dut.dmem.RAM[0]);
    $display("  dmem[1] = 0x%08h (-1.5)", dut.dmem.RAM[1]);
    $display("  dmem[2] = 0x%08h (2.0)", dut.dmem.RAM[2]);
    $display("================================================\n");
  end

  // initialize test
  initial begin
    reset <= 1; #22; reset <= 0;
    #200; // Más tiempo para ver todas las instrucciones
    $display("\n================================================");
    $display("TB: Test completado en ciclo %0d", cycle);
    $display("================================================");
    $finish;
  end

  // generate clock
  always begin
    clk <= 1; #5; clk <= 0; #5;
  end

  always @(negedge clk) begin
  if (!reset) begin
    cycle++;
    $display("\n--- Ciclo %0d (tiempo %0t) ---", cycle, $time);
    
    // Fetch stage
    $display("  [F] PCF=0x%08h  InstrF=0x%08h", dut.riscv.PCF, dut.riscv.InstrF);

    // Decode stage
    if (dut.riscv.dp.InstrD != 0) begin
      $display("  [D] PC=0x%08h  Instr=0x%08h  opcode=0x%02h  isFPD=%b", 
               dut.riscv.dp.PCD, dut.riscv.dp.InstrD, dut.riscv.opD, dut.riscv.isFPD);
      $display("      Rs1D=%0d Rs2D=%0d RdD=%0d  ImmExt=0x%08h", 
               dut.riscv.dp.Rs1D, dut.riscv.dp.Rs2D, dut.riscv.dp.RdD, dut.riscv.dp.ImmExtD);
      $display("      Int RF: RD1D_int=0x%08h  RD2D_int=0x%08h", dut.riscv.dp.RD1D_int, dut.riscv.dp.RD2D_int);
      $display("      FP  RF: RD1D_fp =0x%08h  RD2D_fp =0x%08h", dut.riscv.dp.RD1D_fp, dut.riscv.dp.RD2D_fp);
    end

    // Execute stage
    if (dut.riscv.dp.RdE != 0) begin
      $display("  [E] RdE=%0d  isFPE=%b  ALUControl=0x%01h  ZeroE=%b", 
               dut.riscv.dp.RdE, dut.riscv.isFPE, dut.riscv.ALUControlE, dut.riscv.ZeroE);
      $display("      ForwardAE=%b  ForwardBE=%b", dut.riscv.hu.ForwardAE, dut.riscv.hu.ForwardBE);
      $display("      SrcAE_int=0x%08h  SrcBE_int=0x%08h", dut.riscv.dp.SrcAE_int, dut.riscv.dp.SrcBE_int);
      $display("      SrcAE_fp =0x%08h  SrcBE_fp =0x%08h", dut.riscv.dp.SrcAE_fp, dut.riscv.dp.SrcBE_fp);
      $display("      ALUResultE=0x%08h", dut.riscv.dp.ALUResultE);
    end

    // Memory stage
    if (dut.riscv.dp.RdM != 0) begin
      $display("  [M] RdM=%0d  isFPM=%b  ALUResultM=0x%08h  MemWrite=%b", 
               dut.riscv.dp.RdM, dut.riscv.isFPM, dut.riscv.ALUResultM, dut.MemWriteM);
      $display("      WriteDataM=0x%08h  DataAdrM=0x%08h", dut.WriteDataM, dut.DataAdrM);
    end

    // Writeback stage
    if (dut.riscv.dp.RdW != 0 && dut.riscv.RegWriteW) begin
      $display("  [W] RdW=%0d  isFPW=%b  useFP_RF_W=%b  ResultW=0x%08h  RegWrite=%b", 
               dut.riscv.dp.RdW, dut.riscv.isFPW, dut.riscv.useFP_RF_W,
               dut.riscv.dp.ResultW, dut.riscv.RegWriteW);
      if (dut.riscv.useFP_RF_W)
        $display("      -> Writing to FP register f%0d", dut.riscv.dp.RdW);
      else
        $display("      -> Writing to Int register x%0d", dut.riscv.dp.RdW);
    end

    // Hazard signals
    if (dut.riscv.hu.StallF || dut.riscv.hu.FlushE)
      $display("  [HAZARD] StallF=%b  StallD=%b  FlushD=%b  FlushE=%b", 
               dut.riscv.hu.StallF, dut.riscv.hu.StallD, dut.riscv.hu.FlushD, dut.riscv.hu.FlushE);

    // Dump de registros cada 5 ciclos
    if (cycle % 5 == 0) begin
      $display("  [REG DUMP]");
      $display("      Int: x1=0x%08h  x2=0x%08h  x10=0x%08h", 
               dut.riscv.dp.rf_int.rf[1], dut.riscv.dp.rf_int.rf[2], dut.riscv.dp.rf_int.rf[10]);
      $display("      FP:  f1=0x%08h  f2=0x%08h  f10=0x%08h", 
               dut.riscv.dp.rf_fp.rf[1], dut.riscv.dp.rf_fp.rf[2], dut.riscv.dp.rf_fp.rf[10]);
    end
  end
end


endmodule