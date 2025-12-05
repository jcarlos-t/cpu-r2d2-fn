// modulo para multiplicar numeros en floating point de nbits
// flags de EXCEPCIÓN de salida: {invalid, div0, ovf, udf, inx}
module fmul_core(a,b,round_mode,y,flags);
  parameter NEXP=8;
  parameter NSIG=23;

  input wire round_mode; // 0=truncate, 1=nearest-even
  input wire [NEXP+NSIG:0] a,b;
  output reg [NEXP+NSIG:0] y; // resultado
  output reg [4:0] flags; // {invalid (4), div0, ovf, udf, inx (0)}

  // Clasificación de tipos numericos
  localparam integer NORMAL=0, SUBNORMAL=1, ZERO=2, INFINITY=3, QNAN=4, SNAN=5;
  localparam integer W=1+NEXP+NSIG; // ancho total
  localparam integer M=NSIG+1; // mantisa con bit explicito

  // Desempaquetado/clasificación
  wire signed [NEXP+1:0] aExp,bExp;
  wire [NSIG:0] aSig,bSig;
  wire [5:0] aTFlags,bTFlags;

  // desempaquetado (exponente sin bias)
  unpacker #(NEXP,NSIG) uA(a,aExp,aSig,aTFlags);
  unpacker #(NEXP,NSIG) uB(b,bExp,bSig,bTFlags);

  // Producto bruto y normalización
  wire [2*M-1:0] rawSignificand=aSig*bSig;
  wire top1=rawSignificand[2*M-1];
  wire [2*M-1:0] mant_pre=top1?(rawSignificand>>1):rawSignificand;

  localparam integer HI=2*M-2;
  localparam integer LO=(2*M-2)-(M-1);
  wire [M-1:0] mant_trunc=mant_pre[HI:LO];

  // Redondeo (0=trunc, 1=nearest-even)
  wire G=(LO-1>=0)?mant_pre[LO-1]:1'b0;
  wire R=(LO-2>=0)?mant_pre[LO-2]:1'b0;
  wire S=(LO-3>=0)?(|mant_pre[LO-3:0]):1'b0;

  // Exponente sin bias y signo
  wire signed [NEXP+2:0] E_unb_pre=$signed(aExp)+$signed(bExp)+(top1?1:0);
  wire yS=a[W-1]^b[W-1];

  // Empaquetador (con redondeo)
  wire [W-1:0] y_pack;
  wire [2:0] oiu_pack; // {ovf, udf, inx}
  packer #(.NEXP(NEXP),.NSIG(NSIG)) u_pack(.yS(yS),.E_unb(E_unb_pre),.mant_trunc(mant_trunc),.G(G),.Rb(R),.S(S),.round_mode(round_mode),.y(y_pack),.flags_oiu(oiu_pack));

  // NaN canónico
  wire [W-1:0] qnan_canon={1'b0,{NEXP{1'b1}},{1'b1,{(NSIG-1){1'b0}}}};

  // Control de casos especiales + fusión de flags
  always @* begin
    y={W{1'b0}};
    flags=5'b0;
    if(aTFlags[SNAN]|bTFlags[SNAN]) begin
      y=qnan_canon; 
      flags[4]=1'b1; // invalid
    end else if(aTFlags[QNAN]|bTFlags[QNAN]) begin
      y=qnan_canon;
    end else if(aTFlags[INFINITY]|bTFlags[INFINITY]) begin
      if(aTFlags[ZERO]|bTFlags[ZERO]) begin
        y=qnan_canon; 
        flags[4]=1'b1; // 0×∞ → NaN, invalid
      end else y={yS,{NEXP{1'b1}},{NSIG{1'b0}}};
    end else if(aTFlags[ZERO]|bTFlags[ZERO]) begin
      y={yS,{NEXP{1'b0}},{NSIG{1'b0}}};
    end else begin
      y=y_pack;
      flags[2]=oiu_pack[2]; // ovf
      flags[1]=oiu_pack[1]; // udf
      flags[0]=oiu_pack[0]; // inx
    end
    flags[3]=1'b0; // div0=0 en MUL
  end
endmodule