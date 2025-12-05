// fp_pack: empaquetador IEEE-754 con redondeo interno (truncate / nearest-even)
//  - Entradas ya normalizadas a 1.x (o cerca), con ventana G/R/S
//  - Maneja carry del redondeo, overflow, subnormalización y underflow
//  - Genera solo {ovf, udf, inx}
module packer(yS,E_unb,mant_trunc,G,Rb,S,round_mode,y,flags_oiu);
  parameter NEXP=8;
  parameter NSIG=23;

  // Puertos
  input wire yS; // signo final
  input wire signed [NEXP+2:0] E_unb; // exponente SIN bias (tras normalización)
  input wire [NSIG:0] mant_trunc; // M bits: [MSB(=1 implícito) : ... : LSB]
  input wire G; // Guard
  input wire Rb; // Round bit
  input wire S; // Sticky (pre)
  input wire round_mode; // 0=truncate, 1=nearest-even
  output reg [NEXP+NSIG:0] y; // número IEEE-754 empaquetado
  output reg [2:0] flags_oiu; // {ovf, udf, inx}

  // Constantes locales
  localparam integer W = 1 + NEXP + NSIG; // tamaño total
  localparam integer M = NSIG + 1; // mantisa con bit explicito
  localparam integer BIAS = (1 << (NEXP - 1)) - 1;

  // Redondeo sobre mant_trunc (ruta "normal")
  wire lsb = mant_trunc[0];
  wire inc_nearest_even = G & (Rb | S | lsb); // G=1 y (R|S|lsb)=1 → +1
  wire inc = round_mode ? inc_nearest_even : 1'b0;
  wire [M:0] mant_round_ext = {1'b0, mant_trunc} + inc; // mantisa con redondeo
  wire mant_carry = mant_round_ext[M];
  wire [M-1:0] mant_round = mant_carry ? mant_round_ext[M:1] : mant_round_ext[M-1:0]; // normalización de mantisa

  // Exponente con bias (ajustando carry del redondeo)
  reg signed [NEXP+2:0] Ebias_ext;
  reg [NEXP-1:0] Ebias;

  // Señales para subnormalización
  integer sh, i;
  reg [M:0] sr_in, sr_out; // ancho M+1
  reg guard_sub, round_sub, sticky_sub; // G/R/S tras shift subnormal
  reg lsb_sub, inc_sub;
  reg [NSIG:0] sub_frac_ext; // NSIG+1 (para detectar carry)

  // Empaquetado / banderas
  always @* begin
    // Por defecto
    y = {W{1'b0}};
    flags_oiu = 3'b000;

    // Aplica carry y bias al exponente
    Ebias_ext = E_unb + (mant_carry ? 1 : 0);
    Ebias_ext = Ebias_ext + $signed(BIAS);

    // --------- OVERFLOW: ±Inf y inexact=1 ----------
    if (Ebias_ext >= $signed({1'b0, {NEXP{1'b1}}})) begin
      y = {yS, {NEXP{1'b1}}, {NSIG{1'b0}}};
      flags_oiu = 3'b101; // ovf=1, udf=0, inx=1
    end

    // --------- SUBNORMAL / ZERO ----------
    else if (Ebias_ext <= 0) begin
      // desplazar para llevar el 1 implícito a exponent=0
      sh = 1 - Ebias_ext;
      if (sh < 0) sh = 0;
      if (sh > M + 1) sh = M + 1; // exponente muy pequeño que aborda toda la mantisa

      // preparar datos para shift/round subnormal
      sr_in = {1'b0, mant_round}; // M+1 bits
      sr_out = sr_in >> sh;
      guard_sub = (sh > 0) ? sr_in[sh - 1] : 1'b0;
      round_sub = (sh > 1) ? sr_in[sh - 2] : 1'b0;
      sticky_sub = 1'b0;
      for (i = 0; i <= M; i = i + 1) begin
        if (i < (sh > 2 ? (sh - 2) : 0)) sticky_sub = sticky_sub | sr_in[i];
      end

      lsb_sub = sr_out[NSIG - 1];
      inc_sub = round_mode ? (guard_sub & (round_sub | sticky_sub | lsb_sub)) : 1'b0;

      // suma de redondeo en subnormal (NSIG bits útiles)
      sub_frac_ext = {1'b0, sr_out[NSIG - 1:0]} + inc_sub;

      // inexact en subnormal: bits perdidos (pre) o por el shift subnormal
      // (nota: lost_bits_pre = G|R|S)
      flags_oiu[0] = (G | Rb | S) | inc | guard_sub | round_sub | sticky_sub;

      if (sub_frac_ext[NSIG]) begin
        // carry -> mínimo normal (exp sesgado = 1)
        y = {yS, {{(NEXP - 1){1'b0}}, 1'b1}, {NSIG{1'b0}}};
        flags_oiu[1] = 1'b0; // ya no es tiny
      end else begin
        // subnormal (o 0 exacto)
        y = {yS, {NEXP{1'b0}}, sub_frac_ext[NSIG - 1:0]};
        // underflow = tiny (subnormal != 0) AND inexact
        if (|sub_frac_ext[NSIG - 1:0])
          flags_oiu[1] = flags_oiu[0];
        else
          flags_oiu[1] = 1'b0;
      end
    end

    // --------- NORMAL ----------
    else begin
      Ebias = Ebias_ext[NEXP - 1:0];
      y = {yS, Ebias, mant_round[M - 2:0]};
      // inexact si hubo bits perdidos o incremento por redondeo
      flags_oiu[0] = (G | Rb | S) | inc;
      flags_oiu[1] = 1'b0; // udf
      // ovf ya cubierto arriba
    end
  end
endmodule