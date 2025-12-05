module fdiv_core (a, b, round_mode, y, flags);
  
  // ----- parametros exponente y mantissa -----
  parameter NEXP = 8;
  parameter NSIG = 23;

  input wire round_mode; // 0=truncate, 1=nearest-even
  input wire [NEXP+NSIG:0] a, b;
  output reg [NEXP+NSIG:0] y; // resultado
  output reg [4:0] flags; // {invalid(4), div0(3), ovf(2), udf(1), inx(0)}

  // type float number
  localparam integer NORMAL = 0, SUBNORMAL = 1, ZERO = 2, INFINITY = 3, QNAN = 4, SNAN = 5;

  localparam integer W = 1 + NEXP + NSIG; // ancho total del numero
  localparam integer M = NSIG + 1; // mantisa con 1 (implícito) o 0 (subnormal)
  localparam integer EXTRA = 3; // bits extra para G/R/S en el cociente
  localparam integer NUMW = 2*M + EXTRA + 1; // ancho seguro del numerador escalado

  // ------ Desempaquetado/clasificación ------
  wire signed [NEXP+1:0] aExp, bExp; // exponentes sin bias
  wire [NSIG:0] aSig, bSig; // mantissas extendidas (NSIG+1)
  wire [5:0] aTFlags, bTFlags; // flags de tipo de numero

  unpacker #(NEXP, NSIG) uA (a, aExp, aSig, aTFlags);
  unpacker #(NEXP, NSIG) uB (b, bExp, bSig, bTFlags);

  // =========== División de significandos (ruta normal) ===========
  // Numerador escalado para obtener M+EXTRA bits fraccionarios
  wire [NUMW-1:0] num_base = {{(NUMW-M){1'b0}}, aSig} << (M+EXTRA); // aSig * 2^(M+EXTRA)

  // Cociente con escala fija y resto (si bSig==0, el resultado es 0)
  wire [NUMW-1:0] q_full = (bSig != {M{1'b0}}) ? (num_base / bSig) : {NUMW{1'b0}};
  wire [M-1:0] rem = (bSig != {M{1'b0}}) ? (num_base % bSig) : {M{1'b0}};

  // Normalización a 1.x: si el bit alto no está, desplazar 1 a la izq.
  wire top1 = q_full[M+EXTRA]; // cociente normalizado (1.x) ?
  wire [NUMW-1:0] mant_pre = top1 ? q_full : (q_full << 1);

  // Ventana para extraer mant_trunc y G/R/S
  localparam integer HI = M + EXTRA; // MSB tras normalizar
  localparam integer LO = HI - (M-1); // inicio de los M bits

  wire [M-1:0] mant_trunc = mant_pre[HI:LO]; // M bits (1 implícito + NSIG)
  wire G = (LO-1 >= 0) ? mant_pre[LO-1] : 1'b0;
  wire Rb = (LO-2 >= 0) ? mant_pre[LO-2] : 1'b0;
  wire S_bits = (LO-3 >= 0) ? (|mant_pre[LO-3:0]) : 1'b0;
  wire S_rem = (rem != {M{1'b0}}); // sticky por resto ≠ 0
  wire S = S_bits | S_rem; // sticky total

  // Exponente SIN bias que verá el packer (aExp - bExp - (top1?0:1))
  wire signed [NEXP+2:0] E_unb_pre = $signed(aExp) - $signed(bExp) - (top1 ? 0 : 1);
  wire yS = a[W-1] ^ b[W-1];

  // ---------- Empaquetador común (con redondeo dentro) ----------
  wire [W-1:0] y_pack;
  wire [2:0] oiu_pack; // {ovf, udf, inx}

  packer #(.NEXP(NEXP), .NSIG(NSIG)) u_pack (
    .yS(yS),
    .E_unb(E_unb_pre),
    .mant_trunc(mant_trunc),
    .G(G),
    .Rb(Rb),
    .S(S),
    .round_mode(round_mode),
    .y(y_pack),
    .flags_oiu(oiu_pack)
  );

  // qNaN canónico y signo del resultado
  wire [W-1:0] qnan_canon = {1'b0, {NEXP{1'b1}}, {1'b1, {(NSIG-1){1'b0}}}};

  // ---------- Casos especiales + fusión de flags ----------
  always @(*) begin
    y = {W{1'b0}};
    flags = 5'b0;

    // sNaN → qNaN, invalid=1
    if (aTFlags[SNAN] | bTFlags[SNAN]) begin
      y = qnan_canon;
      flags[4] = 1'b1; // invalid
    end
    // qNaN → qNaN
    else if (aTFlags[QNAN] | bTFlags[QNAN]) begin
      y = qnan_canon;
    end
    // 0/0 o ∞/∞ → invalid
    else if ((aTFlags[ZERO] & bTFlags[ZERO]) | (aTFlags[INFINITY] & bTFlags[INFINITY])) begin
      y = qnan_canon;
      flags[4] = 1'b1; // invalid
    end
    // División por cero: finito no-cero / 0 → ±∞, div0=1
    else if (bTFlags[ZERO]) begin
      y = {yS, {NEXP{1'b1}}, {NSIG{1'b0}}}; // ±Inf
      flags[3] = 1'b1; // div0
    end
    // ∞ / finito → ±∞
    else if (aTFlags[INFINITY] & ~bTFlags[INFINITY]) begin
      y = {yS, {NEXP{1'b1}}, {NSIG{1'b0}}};
    end
    // finito / ∞ → ±0
    else if (~aTFlags[INFINITY] & bTFlags[INFINITY]) begin
      y = {yS, {NEXP{1'b0}}, {NSIG{1'b0}}};
    end
    // 0 / finito → ±0
    else if (aTFlags[ZERO] & ~bTFlags[ZERO]) begin
      y = {yS, {NEXP{1'b0}}, {NSIG{1'b0}}};
    end
    // Ruta normal: delega a packer
    else begin
      y = y_pack;
      flags[2] = oiu_pack[2]; // ovf
      flags[1] = oiu_pack[1]; // udf
      flags[0] = oiu_pack[0]; // inx
    end
  end
endmodule