module fadd_core(a, b, round_mode, y, flags);
  // ----- parámetros exponente y mantissa -----
  parameter NEXP = 8;
  parameter NSIG = 23;

  input wire round_mode; // 0=truncate, 1=nearest-even
  input wire [NEXP+NSIG:0] a, b;
  output reg [NEXP+NSIG:0] y; // resultado
  output reg [4:0] flags; // {invalid, div0, ovf, udf, inx}

  // ------ Constantes y clasificación ------
  localparam integer NORMAL = 0, SUBNORMAL = 1, ZERO = 2, INFINITY = 3, QNAN = 4, SNAN = 5;
  localparam integer W = 1 + NEXP + NSIG; // ancho total
  localparam integer M = NSIG + 1; // 1 + NSIG (bit implícito)
  localparam integer TAIL = 4; // 3 bits (G/R/S) + 1 holgura
  localparam integer ALIGNW = M + TAIL + 1; // +1 por posible carry en suma

  // ------ Desempaquetado/clasificación ------
  wire signed [NEXP+1:0] aExp, bExp; // exponentes sin bias
  wire [NSIG:0] aSig, bSig; // significandos extendidos (NSIG+1)
  wire [5:0] aTFlags, bTFlags;

  unpacker #(NEXP, NSIG) uA(a, aExp, aSig, aTFlags);
  unpacker #(NEXP, NSIG) uB(b, bExp, bSig, bTFlags);

  // --------- Comparación por magnitud ---------
  wire a_ge_b_mag = (aExp > bExp) || ((aExp == bExp) && (aSig >= bSig)); // a>b ?

  wire [NSIG:0] sig_big = a_ge_b_mag ? aSig : bSig; 
  wire [NSIG:0] sig_small = a_ge_b_mag ? bSig : aSig;
  wire signed [NEXP+1:0] exp_big = a_ge_b_mag ? aExp : bExp;
  wire signed [NEXP+1:0] exp_small = a_ge_b_mag ? bExp : aExp;
  wire s_big = a_ge_b_mag ? a[W-1] : b[W-1];
  wire same_sign = (a[W-1] == b[W-1]); // 1: suma de magnitudes + signo | 0: big - small + s_big

  // --------- Señales internas ---------
  reg [ALIGNW-1:0] big_ext, small_ext, small_shifted, res_ext, mant_pre;
  reg signed [NEXP+2:0] yExp; // exponente sin bias tras normalización

  // Ventana para mant_trunc y G/R/S
  localparam integer HI = M + 3;
  localparam integer LO = HI - (M - 1);
  wire [M-1:0] mant_trunc = mant_pre[HI:LO];
  wire G = mant_pre[LO-1];
  wire Rb = mant_pre[LO-2];
  wire S = |mant_pre[LO-3:0];

  // --------- Empaquetador común ---------
  reg do_pack; //habilita empaquetado
  reg s_for_pack;
  reg signed [NEXP+2:0] E_unb_for_pack;

  wire [W-1:0] y_pack;
  wire [2:0] oiu_pack; // {ovf, udf, inx}

  packer #(.NEXP(NEXP), .NSIG(NSIG)) u_pack(
    .yS(s_for_pack),
    .E_unb(E_unb_for_pack),
    .mant_trunc(mant_trunc),
    .G(G),
    .Rb(Rb),
    .S(S),
    .round_mode(round_mode),
    .y(y_pack),
    .flags_oiu(oiu_pack)
  );

  // --------- Función: posición del MSB ---------
  function integer msb_pos;
    input [ALIGNW-1:0] v;
    integer k2;
    begin
      msb_pos = -1;
      for (k2 = ALIGNW-1; k2 >= 0; k2 = k2 - 1)
        if (msb_pos == -1 && v[k2]) msb_pos = k2;
    end
  endfunction

  // ---- Variables auxiliares ----
  integer dE;
  integer i;
  integer msb;
  integer shl;
  reg sticky_shift;

  // qNaN canónico
  wire [W-1:0] qnan_canon = {1'b0, {NEXP{1'b1}}, {1'b1, {(NSIG-1){1'b0}}}};

  // ========================= Camino principal =========================
  always @* begin
    y = {W{1'b0}};
    flags = 5'b0;
    do_pack = 1'b0;
    s_for_pack = s_big;
    E_unb_for_pack = 0;

    // ===== Casos especiales IEEE-754 =====
    if (aTFlags[SNAN] | bTFlags[SNAN]) begin
      y = qnan_canon; flags[4] = 1'b1; // invalid
    end else if (aTFlags[QNAN] | bTFlags[QNAN]) begin
      y = qnan_canon;
    end else if ((aTFlags[INFINITY] & bTFlags[INFINITY]) && (a[W-1] ^ b[W-1])) begin
      y = qnan_canon; flags[4] = 1'b1; // +Inf + -Inf -> invalid
    end else if (aTFlags[INFINITY]) begin
      y = {a[W-1], {NEXP{1'b1}}, {NSIG{1'b0}}};
    end else if (bTFlags[INFINITY]) begin
      y = {b[W-1], {NEXP{1'b1}}, {NSIG{1'b0}}};
    end else if (aTFlags[ZERO] & bTFlags[ZERO]) begin
      y = {(a[W-1] & b[W-1]), {NEXP{1'b0}}, {NSIG{1'b0}}};
    end else begin
      // ===== Alinear =====
      big_ext = {1'b0, sig_big, {TAIL{1'b0}}};
      small_ext = {1'b0, sig_small, {TAIL{1'b0}}};

      // Delta de exponentes saturado
      dE = $signed(exp_big - exp_small);
      if (dE < 0) dE = 0;
      if (dE > (ALIGNW - 1)) dE = (ALIGNW - 1);

      // Shift del menor + sticky
      if (dE == 0) begin
        small_shifted = small_ext;
      end else begin
        small_shifted = small_ext >> dE;
        sticky_shift = 1'b0;
        for (i = 0; i < ALIGNW; i = i + 1)
          if (i < dE) sticky_shift = sticky_shift | small_ext[i];
        small_shifted[0] = small_shifted[0] | sticky_shift;
      end

      if (same_sign) begin
        // ===== SUMA =====
        res_ext = big_ext + small_shifted;

        if (res_ext[ALIGNW-1]) begin
          mant_pre = res_ext >> 1;
          yExp = $signed(exp_big) + 1;
        end else begin
          mant_pre = res_ext;
          yExp = $signed(exp_big);
        end

        s_for_pack = s_big;
        E_unb_for_pack = yExp;
        do_pack = 1'b1;

      end else begin
        // ===== RESTA =====
        res_ext = big_ext - small_shifted;

        if (res_ext == {ALIGNW{1'b0}}) begin
          y = {1'b0, {NEXP{1'b0}}, {NSIG{1'b0}}}; // es 0
          flags = 5'b0; //inx
        end else begin
          msb = msb_pos(res_ext);
          shl = (M + 3) - msb;
          if (shl < 0) shl = 0;
          if (shl > (M + 3)) shl = (M + 3);

          mant_pre = res_ext << shl;
          yExp = $signed(exp_big) - shl;

          s_for_pack = s_big;
          E_unb_for_pack = yExp;
          do_pack = 1'b1;
        end
      end
    end

    // ===== Empaquetado común =====
    if (do_pack) begin
      y = y_pack;
      flags[2] = oiu_pack[2]; // ovf
      flags[1] = oiu_pack[1]; // udf
      flags[0] = oiu_pack[0]; // inx
    end

    flags[3] = 1'b0; // div0 = 0 en ADD/SUB
  end
endmodule