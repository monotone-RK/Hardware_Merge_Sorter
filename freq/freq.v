//-----------------------------------------------------------------------------
// File          : freq.v
// Author        : Ryohei Kobayashi
// Created       : 09.12.2017
// Last modified : 09.12.2017
//-----------------------------------------------------------------------------
// Description :
// A project file to evaluate the maximum frequency of the hardware merge sorter 
//-----------------------------------------------------------------------------
`default_nettype none

`define E_LOG  5
`define DATW  64
`define KEYW  32
  
module freq(input  wire CLK,
            input  wire RST_IN,
            output wire OUT);

  reg RST; always @(posedge CLK) RST <= RST_IN;
     
  wire [(`DATW<<`E_LOG)-1:0] merge_tree_din;
  wire [(1<<`E_LOG)-1:0]     merge_tree_dinen;
  wire [(1<<`E_LOG)-1:0]     merge_tree_ful;
  wire [(`DATW<<`E_LOG)-1:0] merge_tree_dot;
  wire                       merge_tree_doten;

  assign merge_tree_dinen = ~merge_tree_ful;
  
  genvar i;
  generate
    for (i=0; i<(1<<`E_LOG); i=i+1) begin: loop
      reg [`KEYW-1:0]         init_key;
      reg [(`DATW-`KEYW)-1:0] init_payload;
      always @(posedge CLK) begin
        if      (RST)                 init_key <= (1<<`E_LOG) - i;
        else if (merge_tree_dinen[i]) init_key <= init_key + (1<<`E_LOG);
      end
      always @(posedge CLK) begin
        if      (RST)                 init_payload <= i + 1;
        else if (merge_tree_dinen[i]) init_payload <= (init_payload << 1) + 1;
      end
      assign merge_tree_din[`DATW*(i+1)-1:`DATW*i] = {init_payload, init_key};
    end
  endgenerate

  MERGE_TREE #(`E_LOG, `DATW, `KEYW)
  merge_tree(CLK, RST, 1'b0, merge_tree_din, merge_tree_dinen, 
             merge_tree_ful, merge_tree_dot, merge_tree_doten);
  
  reg [(`DATW<<`E_LOG)-1:0] dot_buf; 
  always @(posedge CLK) begin
    if (merge_tree_doten) dot_buf <= merge_tree_dot;
  end

  assign OUT = ^dot_buf;
  
endmodule

`default_nettype wire
