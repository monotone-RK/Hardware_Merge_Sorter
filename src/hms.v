/******************************************************************************/
/* HMS(E), an E-record hardware merge sorter                 Ryohei Kobayashi */
/*                                                         Version 2017-11-30 */
/******************************************************************************/
`default_nettype none

/***** A comparator                                                       *****/
/******************************************************************************/
module COMPARATOR #(parameter              KEYW = 32)
                   (input  wire [KEYW-1:0] DIN0,
                    input  wire [KEYW-1:0] DIN1,
                    output wire            RSLT);
  
  assign RSLT = (DIN0 <= DIN1);
  
endmodule


/***** A 2-input multiplexer                                              *****/
/******************************************************************************/
module MUX2 #(parameter              DATW = 64)
             (input  wire [DATW-1:0] DIN0,
              input  wire [DATW-1:0] DIN1,
              input  wire            SEL,
              output wire [DATW-1:0] DOUT);

  function [DATW-1:0] mux;
    input [DATW-1:0] a;
    input [DATW-1:0] b;
    input            sel;
    begin
      case (sel)
        1'b0: mux = a;
        1'b1: mux = b;
      endcase
    end
  endfunction

  assign DOUT = mux(DIN1, DIN0, SEL);
    
endmodule
  

/***** A 3-input multiplexer                                              *****/
/******************************************************************************/
module MUX3 #(parameter              DATW = 64)
             (input  wire [DATW-1:0] DIN0,
              input  wire [DATW-1:0] DIN1,
              input  wire [DATW-1:0] DIN2,
              input  wire [1:0]      SEL,
              output wire [DATW-1:0] DOUT);

  function [DATW-1:0] mux;
    input [DATW-1:0] a;
    input [DATW-1:0] b;
    input [DATW-1:0] c;
    input [1:0]      sel;
    begin
      casex (sel)
        2'bx0: mux = a;
        2'b01: mux = b;
        2'b11: mux = c;
      endcase
    end
  endfunction

  assign DOUT = mux(DIN0, DIN2, DIN1, SEL);
    
endmodule
  

/***** A sort logic                                                       *****/
/******************************************************************************/
module SORT_LOGIC #(parameter                       E_LOG = 2,
                    parameter                       DATW  = 64,
                    parameter                       KEYW  = 32)
                   (input  wire                     CLK,
                    input  wire                     RST,
                    input  wire                     STALL,
                    input  wire [(DATW<<E_LOG)-1:0] DIN,
                    input  wire                     DINEN,
                    output wire [(DATW<<E_LOG)-1:0] DOT,
                    output wire                     DOTEN);
   
  genvar i;
  
  // Stage A
  //////////////////////////////////////////////////////////
  reg  [(DATW<<E_LOG)-1:0] din_a;    // pipeline register for E records
  reg                      dinen_a;  // pipeline register for control
  reg  [DATW-1:0]          fb_buf;
  wire [DATW-1:0]          fb_record;
  wire [(1<<E_LOG)-1:0]    comp_rslts;

  always @(posedge CLK) if (!STALL) din_a <= DIN;
  always @(posedge CLK) begin
    if      (RST)    dinen_a <= 0;
    else if (!STALL) dinen_a <= DINEN;
  end

  generate
    for (i=0; i<(1<<E_LOG); i=i+1) begin: comparators
      COMPARATOR #(KEYW)
      comparator(din_a[(KEYW+DATW*i)-1:DATW*i], fb_buf[KEYW-1:0], comp_rslts[i]);
    end
  endgenerate
  
  MUX2 #(DATW)
  mux2(din_a[(DATW<<E_LOG)-1:(DATW<<E_LOG)-DATW], fb_buf, comp_rslts[(1<<E_LOG)-1], fb_record);

  always @(posedge CLK) begin
    if (RST) begin
      fb_buf <= 0;  // This register is initialized to 0 for an ascending order
    end else if (!STALL) begin
      if (dinen_a) fb_buf <= fb_record;
    end
  end
  
  // Stage B
  //////////////////////////////////////////////////////////
  reg  [(DATW+(DATW<<E_LOG))-1:0] din_b;    // pipeline register for E records
  reg                             dinen_b;  // pipeline register for control
  reg  [(1<<E_LOG)-1:0]           comp_rslts_buf;
  wire [(DATW<<E_LOG)-1:0]        selected_records;

  always @(posedge CLK) if (!STALL) din_b <= {fb_buf, din_a};
  always @(posedge CLK) begin
    if      (RST)    dinen_b <= 0;
    else if (!STALL) dinen_b <= dinen_a;
  end
  always @(posedge CLK) if (!STALL) comp_rslts_buf <= comp_rslts;

  generate
    for (i=0; i<(1<<E_LOG); i=i+1) begin: multiplexers
      if (i == 0) begin
        MUX2 #(DATW)
        mux2(din_b[DATW-1:0], din_b[(DATW+(DATW<<E_LOG))-1:(DATW<<E_LOG)], comp_rslts_buf[0], selected_records[DATW-1:0]);
      end else begin
        MUX3 #(DATW)
        mux3(din_b[DATW*i-1:DATW*(i-1)], din_b[DATW*(i+1)-1:DATW*i], din_b[(DATW+(DATW<<E_LOG))-1:(DATW<<E_LOG)], comp_rslts_buf[i:i-1], selected_records[DATW*(i+1)-1:DATW*i]);
      end
    end
  endgenerate

  // output
  assign DOT   = selected_records;
  assign DOTEN = dinen_b;
  
endmodule


  
`default_nettype wire
