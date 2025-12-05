// modulo para dividir en partes el floating point number
// caracteristicas: exponente sin bias, mantissa con 1 implícito añadido o corrimiento si subnormal
// tipo (NORMAL, SUBNORMAL, ZERO, INFINITY, QNAN, SNAN)
module unpacker(f,fExp,fSig,fFlags);
  parameter NEXP=5;
  parameter NSIG=10;

  input [NEXP+NSIG:0] f; // numero a desempaquetar
  output reg signed [NEXP+1:0] fExp; // exponente sin bias
  output reg [NSIG:0] fSig; // 1+NSIG (MSB=1 si NORMAL, 0 si SUBNORMAL)
  output [5:0] fFlags; // clasificación del numero

  localparam integer NORMAL=0, SUBNORMAL=1, ZERO=2, INFINITY=3, QNAN=4, SNAN=5;
  localparam integer BIAS=(1<<(NEXP-1))-1;
  localparam integer EMAX=BIAS;
  localparam integer EMIN=1-EMAX;

  // Partes del número
  wire [NEXP-1:0] e=f[NEXP+NSIG-1:NSIG];
  wire [NSIG-1:0] m=f[NSIG-1:0];

  // Clasificación IEEE
  assign fFlags[SNAN]=(&e)&(|m)&~m[NSIG-1];
  assign fFlags[QNAN]=(&e)&(|m)&m[NSIG-1];
  assign fFlags[INFINITY]=(&e)&~(|m);
  assign fFlags[ZERO]=~(|e)&~(|m);
  assign fFlags[SUBNORMAL]=~(|e)&(|m);
  assign fFlags[NORMAL]=(~&e)&(|e); // ni todo 1s ni todo 0s

  // Construcción de exp/sig
  always @* begin
    if(fFlags[NORMAL]) begin
      fExp=$signed({1'b0,e})-$signed(BIAS);
      fSig={1'b1,m};
    end else if(fFlags[SUBNORMAL]) begin
      fExp=EMIN;
      fSig={1'b0,m};
    end else begin
      fExp='b0;
      fSig={1'b0,m};
    end
  end
endmodule