
module tb_pipeline();
  logic clk;
  logic reset;
  logic [31:0] WriteData, DataAdr;
  logic MemWrite;

  // Nombre de archivo
  logic [127:0] mem_file = "pipeline.mem";

  // pipeline
  top dut(clk, reset, WriteData, DataAdr, MemWrite, mem_file);

  // initialize test
  initial begin
    reset <= 1; #22; reset <= 0;
  end

  // generate clock to sequence tests
  always begin
    clk <= 1; #5; clk <= 0; #5;
  end

  // check results
  always @(negedge clk) begin
    if (MemWrite) begin
      if (DataAdr === 100 & WriteData === 25) begin
        $display("Simulation succeeded");
        $stop;
      end else if (DataAdr !== 96) begin
        $display("Simulation failed");
        $stop;
      end
    end
  end
endmodule