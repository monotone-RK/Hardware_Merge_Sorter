/******************************************************************************/
/* A test bench                                              Ryohei Kobayashi */
/*                                                         Version 2017-11-30 */
/******************************************************************************/
`default_nettype none
  
`include "hms.v"

`define E_LOG  2
`define DATW  64
`define KEYW  32
  
module tb_MERGE_NETWORK();
  reg CLK; initial begin CLK=0; forever #50 CLK=~CLK; end
  reg RST; initial begin RST=1; #400 RST=0; end

  wire [(`DATW<<`E_LOG)-1:0] init_e_records_keyodd;
  wire [(`DATW<<`E_LOG)-1:0] init_e_records_keyeven;
  
  wire                       fifo4odd_enq;
  wire                       fifo4odd_deq;
  wire [(`DATW<<`E_LOG)-1:0] fifo4odd_din;
  wire [(`DATW<<`E_LOG)-1:0] fifo4odd_dot;
  wire                       fifo4odd_emp;
  wire                       fifo4odd_ful;
  wire [4:0]                 fifo4odd_cnt;
  
  wire                       fifo4even_enq;
  wire                       fifo4even_deq;
  wire [(`DATW<<`E_LOG)-1:0] fifo4even_din;
  wire [(`DATW<<`E_LOG)-1:0] fifo4even_dot;
  wire                       fifo4even_emp;
  wire                       fifo4even_ful;
  wire [4:0]                 fifo4even_cnt;

  wire                       comp_rslt;
  wire [(`DATW<<`E_LOG)-1:0] selected_records;

  wire [(`DATW<<`E_LOG)-1:0] merge_network_dot;
  wire                       merge_network_doten;
  
  assign fifo4odd_enq  = !fifo4odd_ful;
  assign fifo4even_enq = !fifo4even_ful;

  assign fifo4odd_deq  = !fifo4odd_emp && !fifo4even_emp &&  comp_rslt;
  assign fifo4even_deq = !fifo4odd_emp && !fifo4even_emp && ~comp_rslt;
  
  SRL_FIFO #(4, (`DATW<<`E_LOG))
  fifo4odd(CLK, RST, fifo4odd_enq, fifo4odd_deq, fifo4odd_din, 
           fifo4odd_dot, fifo4odd_emp, fifo4odd_ful, fifo4odd_cnt);
  SRL_FIFO #(4, (`DATW<<`E_LOG))
  fifo4even(CLK, RST, fifo4even_enq, fifo4even_deq, fifo4even_din, 
            fifo4even_dot, fifo4even_emp, fifo4even_ful, fifo4even_cnt);
  
  COMPARATOR #(`KEYW)
  comparator(fifo4odd_dot[`KEYW-1:0], fifo4even_dot[`KEYW-1:0], comp_rslt);

  MUX2 #((`DATW<<`E_LOG))
  mux2(fifo4odd_dot, fifo4even_dot, comp_rslt, selected_records);
  
  MERGE_NETWORK #(`E_LOG, `DATW, `KEYW)
  merge_network(CLK, RST, 1'b0, selected_records, (!fifo4odd_emp && !fifo4even_emp), 
                merge_network_dot, merge_network_doten);
  
  genvar i;
  generate
    for (i=0; i<(1<<`E_LOG); i=i+1) begin: loop
      wire [`KEYW-1:0] init_keyodd  = 2 * i + 1;
      wire [`KEYW-1:0] init_keyeven = 2 * i + 2;
      reg  [`DATW-1:0] fifo4odd_initrecord;
      reg  [`DATW-1:0] fifo4even_initrecord;
      always @(posedge CLK) begin
        if      (RST)          fifo4odd_initrecord <= {{(`DATW-`KEYW){1'b1}}, init_keyodd};
        else if (fifo4odd_enq) fifo4odd_initrecord <= fifo4odd_initrecord + (1<<(`E_LOG+1));
      end
      always @(posedge CLK) begin
        if      (RST)           fifo4even_initrecord <= {{(`DATW-`KEYW){1'b1}}, init_keyeven};
        else if (fifo4even_enq) fifo4even_initrecord <= fifo4even_initrecord + (1<<(`E_LOG+1));
      end
      assign fifo4odd_din[`DATW*(i+1)-1:`DATW*i]  = fifo4odd_initrecord;
      assign fifo4even_din[`DATW*(i+1)-1:`DATW*i] = fifo4even_initrecord;
    end
  endgenerate

  always @(posedge CLK) begin
    if (!fifo4odd_emp && !fifo4even_emp) begin
      $write("%d %d %d %d %d ", comp_rslt,
             selected_records[(`KEYW+`DATW*0)-1:`DATW*0], selected_records[(`KEYW+`DATW*1)-1:`DATW*1], selected_records[(`KEYW+`DATW*2)-1:`DATW*2], selected_records[(`KEYW+`DATW*3)-1:`DATW*3]);
      $write("| %d %d ", fifo4odd_cnt, fifo4even_cnt);
      $write("\n");
      $fflush();
    end
  end
  
  // always @(posedge CLK) begin
  //   if (merge_network_doten) begin
  //     $write("%d %d %d %d ", merge_network_dot[(`KEYW+`DATW*0)-1:`DATW*0], merge_network_dot[(`KEYW+`DATW*1)-1:`DATW*1], merge_network_dot[(`KEYW+`DATW*2)-1:`DATW*2], merge_network_dot[(`KEYW+`DATW*3)-1:`DATW*3]);
  //     $write("\n");
  //     $fflush();
  //   end
  // end
  
  reg [31:0] cycle;
  always @(posedge CLK) begin
    if (RST) begin
      cycle <= 0;
    end else begin
      cycle <= cycle + 1;
      if (cycle >= 30) $finish();
    end
  end
  
    
endmodule


`default_nettype wire
