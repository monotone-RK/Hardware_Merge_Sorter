/******************************************************************************/
/* A test bench                                              Ryohei Kobayashi */
/*                                                         Version 2017-11-30 */
/******************************************************************************/
`default_nettype none
  
`include "hms.v"

`define E_LOG  5  // 1 ~ 5
`define DATW  64
`define KEYW  32
  
// module tb_SELECTOR_LOGIC_and_MERGE_NETWORK();
//   reg CLK; initial begin CLK=0; forever #50 CLK=~CLK; end
//   reg RST; initial begin RST=1; #400 RST=0; end

//   wire [(`DATW<<`E_LOG)-1:0] init_e_records_keyodd;
//   wire [(`DATW<<`E_LOG)-1:0] init_e_records_keyeven;
  
//   wire                       fifo4odd_enq;
//   wire                       fifo4odd_deq;
//   wire [(`DATW<<`E_LOG)-1:0] fifo4odd_din;
//   wire [(`DATW<<`E_LOG)-1:0] fifo4odd_dot;
//   wire                       fifo4odd_emp;
//   wire                       fifo4odd_ful;
//   wire [4:0]                 fifo4odd_cnt;
  
//   wire                       fifo4even_enq;
//   wire                       fifo4even_deq;
//   wire [(`DATW<<`E_LOG)-1:0] fifo4even_din;
//   wire [(`DATW<<`E_LOG)-1:0] fifo4even_dot;
//   wire                       fifo4even_emp;
//   wire                       fifo4even_ful;
//   wire [4:0]                 fifo4even_cnt;

//   wire                       comp_rslt;
//   wire [(`DATW<<`E_LOG)-1:0] selected_records;
//   wire                       selected_records_valid;

//   wire [(`DATW<<`E_LOG)-1:0] merge_network_dot;
//   wire                       merge_network_doten;
  
//   wire [(`DATW<<`E_LOG)-1:0] check_records;
  
//   assign fifo4odd_enq  = !fifo4odd_ful;
//   assign fifo4even_enq = !fifo4even_ful;

//   SRL_FIFO #(4, (`DATW<<`E_LOG))
//   fifo4odd(CLK, RST, fifo4odd_enq, fifo4odd_deq, fifo4odd_din, 
//            fifo4odd_dot, fifo4odd_emp, fifo4odd_ful, fifo4odd_cnt);
//   SRL_FIFO #(4, (`DATW<<`E_LOG))
//   fifo4even(CLK, RST, fifo4even_enq, fifo4even_deq, fifo4even_din, 
//             fifo4even_dot, fifo4even_emp, fifo4even_ful, fifo4even_cnt);
  
//   // assign selected_records_valid = !fifo4odd_emp && !fifo4even_emp;
  
//   // assign fifo4odd_deq  = selected_records_valid &&  comp_rslt;
//   // assign fifo4even_deq = selected_records_valid && ~comp_rslt;
  
//   // COMPARATOR #(`KEYW)
//   // comparator(fifo4odd_dot[`KEYW-1:0], fifo4even_dot[`KEYW-1:0], comp_rslt);

//   // MUX2 #((`DATW<<`E_LOG))
//   // mux2(fifo4odd_dot, fifo4even_dot, comp_rslt, selected_records);

//   SELECTOR_LOGIC #(`E_LOG, `DATW, `KEYW)
//   selector_logic(CLK, RST, 1'b0, fifo4odd_dot, fifo4odd_emp, fifo4even_dot, fifo4even_emp, 
//                  fifo4odd_deq, fifo4even_deq, selected_records, selected_records_valid);

//   MERGE_NETWORK #(`E_LOG, `DATW, `KEYW)
//   merge_network(CLK, RST, 1'b0, selected_records, selected_records_valid, 
//                 merge_network_dot, merge_network_doten);
  
//   genvar i;
//   generate
//     for (i=0; i<(1<<`E_LOG); i=i+1) begin: loop
//       wire [`KEYW-1:0] init_keyodd  = 2 * i + 1;
//       wire [`KEYW-1:0] init_keyeven = 2 * i + 2;
//       wire [`KEYW-1:0] chk_rslt_key = i + 1;
//       reg  [`DATW-1:0] fifo4odd_initrecord;
//       reg  [`DATW-1:0] fifo4even_initrecord;
//       reg  [`DATW-1:0] chk_rslt;
//       always @(posedge CLK) begin
//         if      (RST)          fifo4odd_initrecord <= {{(`DATW-`KEYW){1'b1}}, init_keyodd};
//         else if (fifo4odd_enq) fifo4odd_initrecord <= fifo4odd_initrecord + (1<<(`E_LOG+1));
//       end
//       always @(posedge CLK) begin
//         if      (RST)           fifo4even_initrecord <= {{(`DATW-`KEYW){1'b1}}, init_keyeven};
//         else if (fifo4even_enq) fifo4even_initrecord <= fifo4even_initrecord + (1<<(`E_LOG+1));
//       end
//       always @(posedge CLK) begin
//         if      (RST)                 chk_rslt <= {{(`DATW-`KEYW){1'b1}}, chk_rslt_key};
//         else if (merge_network_doten) chk_rslt <= chk_rslt + (1<<`E_LOG);
//       end
//       assign fifo4odd_din[`DATW*(i+1)-1:`DATW*i]  = fifo4odd_initrecord;
//       assign fifo4even_din[`DATW*(i+1)-1:`DATW*i] = fifo4even_initrecord;
//       assign check_records[`DATW*(i+1)-1:`DATW*i] = chk_rslt;
//     end
//   endgenerate

//   // show result
//   // always @(posedge CLK) begin
//   //   if (RST) begin end
//   //   else if (selected_records_valid) begin
//   //     case (`E_LOG)
//   //       1: $write("%d %d ", selected_records[(`KEYW+`DATW*0)-1:`DATW*0], selected_records[(`KEYW+`DATW*1)-1:`DATW*1]);
//   //       2: $write("%d %d %d %d ", selected_records[(`KEYW+`DATW*0)-1:`DATW*0], selected_records[(`KEYW+`DATW*1)-1:`DATW*1], selected_records[(`KEYW+`DATW*2)-1:`DATW*2], selected_records[(`KEYW+`DATW*3)-1:`DATW*3]);
//   //       3: $write("%d %d %d %d %d %d %d %d ", selected_records[(`KEYW+`DATW*0)-1:`DATW*0], selected_records[(`KEYW+`DATW*1)-1:`DATW*1], selected_records[(`KEYW+`DATW*2)-1:`DATW*2], selected_records[(`KEYW+`DATW*3)-1:`DATW*3], selected_records[(`KEYW+`DATW*4)-1:`DATW*4], selected_records[(`KEYW+`DATW*5)-1:`DATW*5], selected_records[(`KEYW+`DATW*6)-1:`DATW*6], selected_records[(`KEYW+`DATW*7)-1:`DATW*7]);
//   //       4: $write("%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d ", selected_records[(`KEYW+`DATW*0)-1:`DATW*0], selected_records[(`KEYW+`DATW*1)-1:`DATW*1], selected_records[(`KEYW+`DATW*2)-1:`DATW*2], selected_records[(`KEYW+`DATW*3)-1:`DATW*3], selected_records[(`KEYW+`DATW*4)-1:`DATW*4], selected_records[(`KEYW+`DATW*5)-1:`DATW*5], selected_records[(`KEYW+`DATW*6)-1:`DATW*6], selected_records[(`KEYW+`DATW*7)-1:`DATW*7], selected_records[(`KEYW+`DATW*8)-1:`DATW*8], selected_records[(`KEYW+`DATW*9)-1:`DATW*9], selected_records[(`KEYW+`DATW*10)-1:`DATW*10], selected_records[(`KEYW+`DATW*11)-1:`DATW*11], selected_records[(`KEYW+`DATW*12)-1:`DATW*12], selected_records[(`KEYW+`DATW*13)-1:`DATW*13], selected_records[(`KEYW+`DATW*14)-1:`DATW*14], selected_records[(`KEYW+`DATW*15)-1:`DATW*15]);
//   //       5: $write("%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d ", selected_records[(`KEYW+`DATW*0)-1:`DATW*0], selected_records[(`KEYW+`DATW*1)-1:`DATW*1], selected_records[(`KEYW+`DATW*2)-1:`DATW*2], selected_records[(`KEYW+`DATW*3)-1:`DATW*3], selected_records[(`KEYW+`DATW*4)-1:`DATW*4], selected_records[(`KEYW+`DATW*5)-1:`DATW*5], selected_records[(`KEYW+`DATW*6)-1:`DATW*6], selected_records[(`KEYW+`DATW*7)-1:`DATW*7], selected_records[(`KEYW+`DATW*8)-1:`DATW*8], selected_records[(`KEYW+`DATW*9)-1:`DATW*9], selected_records[(`KEYW+`DATW*10)-1:`DATW*10], selected_records[(`KEYW+`DATW*11)-1:`DATW*11], selected_records[(`KEYW+`DATW*12)-1:`DATW*12], selected_records[(`KEYW+`DATW*13)-1:`DATW*13], selected_records[(`KEYW+`DATW*14)-1:`DATW*14], selected_records[(`KEYW+`DATW*15)-1:`DATW*15], selected_records[(`KEYW+`DATW*16)-1:`DATW*16], selected_records[(`KEYW+`DATW*17)-1:`DATW*17], selected_records[(`KEYW+`DATW*18)-1:`DATW*18], selected_records[(`KEYW+`DATW*19)-1:`DATW*19], selected_records[(`KEYW+`DATW*20)-1:`DATW*20], selected_records[(`KEYW+`DATW*21)-1:`DATW*21], selected_records[(`KEYW+`DATW*22)-1:`DATW*22], selected_records[(`KEYW+`DATW*23)-1:`DATW*23], selected_records[(`KEYW+`DATW*24)-1:`DATW*24], selected_records[(`KEYW+`DATW*25)-1:`DATW*25], selected_records[(`KEYW+`DATW*26)-1:`DATW*26], selected_records[(`KEYW+`DATW*27)-1:`DATW*27], selected_records[(`KEYW+`DATW*28)-1:`DATW*28], selected_records[(`KEYW+`DATW*29)-1:`DATW*29], selected_records[(`KEYW+`DATW*30)-1:`DATW*30], selected_records[(`KEYW+`DATW*31)-1:`DATW*31]);
//   //     endcase
//   //     $write("\n");
//   //     $fflush();
//   //   end
//   // end
//   always @(posedge CLK) begin
//     if (merge_network_doten) begin
//       case (`E_LOG)
//         1: $write("%d %d ", merge_network_dot[(`KEYW+`DATW*0)-1:`DATW*0], merge_network_dot[(`KEYW+`DATW*1)-1:`DATW*1]);
//         2: $write("%d %d %d %d ", merge_network_dot[(`KEYW+`DATW*0)-1:`DATW*0], merge_network_dot[(`KEYW+`DATW*1)-1:`DATW*1], merge_network_dot[(`KEYW+`DATW*2)-1:`DATW*2], merge_network_dot[(`KEYW+`DATW*3)-1:`DATW*3]);
//         3: $write("%d %d %d %d %d %d %d %d ", merge_network_dot[(`KEYW+`DATW*0)-1:`DATW*0], merge_network_dot[(`KEYW+`DATW*1)-1:`DATW*1], merge_network_dot[(`KEYW+`DATW*2)-1:`DATW*2], merge_network_dot[(`KEYW+`DATW*3)-1:`DATW*3], merge_network_dot[(`KEYW+`DATW*4)-1:`DATW*4], merge_network_dot[(`KEYW+`DATW*5)-1:`DATW*5], merge_network_dot[(`KEYW+`DATW*6)-1:`DATW*6], merge_network_dot[(`KEYW+`DATW*7)-1:`DATW*7]);
//         4: $write("%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d ", merge_network_dot[(`KEYW+`DATW*0)-1:`DATW*0], merge_network_dot[(`KEYW+`DATW*1)-1:`DATW*1], merge_network_dot[(`KEYW+`DATW*2)-1:`DATW*2], merge_network_dot[(`KEYW+`DATW*3)-1:`DATW*3], merge_network_dot[(`KEYW+`DATW*4)-1:`DATW*4], merge_network_dot[(`KEYW+`DATW*5)-1:`DATW*5], merge_network_dot[(`KEYW+`DATW*6)-1:`DATW*6], merge_network_dot[(`KEYW+`DATW*7)-1:`DATW*7], merge_network_dot[(`KEYW+`DATW*8)-1:`DATW*8], merge_network_dot[(`KEYW+`DATW*9)-1:`DATW*9], merge_network_dot[(`KEYW+`DATW*10)-1:`DATW*10], merge_network_dot[(`KEYW+`DATW*11)-1:`DATW*11], merge_network_dot[(`KEYW+`DATW*12)-1:`DATW*12], merge_network_dot[(`KEYW+`DATW*13)-1:`DATW*13], merge_network_dot[(`KEYW+`DATW*14)-1:`DATW*14], merge_network_dot[(`KEYW+`DATW*15)-1:`DATW*15]);
//         5: $write("%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d ", merge_network_dot[(`KEYW+`DATW*0)-1:`DATW*0], merge_network_dot[(`KEYW+`DATW*1)-1:`DATW*1], merge_network_dot[(`KEYW+`DATW*2)-1:`DATW*2], merge_network_dot[(`KEYW+`DATW*3)-1:`DATW*3], merge_network_dot[(`KEYW+`DATW*4)-1:`DATW*4], merge_network_dot[(`KEYW+`DATW*5)-1:`DATW*5], merge_network_dot[(`KEYW+`DATW*6)-1:`DATW*6], merge_network_dot[(`KEYW+`DATW*7)-1:`DATW*7], merge_network_dot[(`KEYW+`DATW*8)-1:`DATW*8], merge_network_dot[(`KEYW+`DATW*9)-1:`DATW*9], merge_network_dot[(`KEYW+`DATW*10)-1:`DATW*10], merge_network_dot[(`KEYW+`DATW*11)-1:`DATW*11], merge_network_dot[(`KEYW+`DATW*12)-1:`DATW*12], merge_network_dot[(`KEYW+`DATW*13)-1:`DATW*13], merge_network_dot[(`KEYW+`DATW*14)-1:`DATW*14], merge_network_dot[(`KEYW+`DATW*15)-1:`DATW*15], merge_network_dot[(`KEYW+`DATW*16)-1:`DATW*16], merge_network_dot[(`KEYW+`DATW*17)-1:`DATW*17], merge_network_dot[(`KEYW+`DATW*18)-1:`DATW*18], merge_network_dot[(`KEYW+`DATW*19)-1:`DATW*19], merge_network_dot[(`KEYW+`DATW*20)-1:`DATW*20], merge_network_dot[(`KEYW+`DATW*21)-1:`DATW*21], merge_network_dot[(`KEYW+`DATW*22)-1:`DATW*22], merge_network_dot[(`KEYW+`DATW*23)-1:`DATW*23], merge_network_dot[(`KEYW+`DATW*24)-1:`DATW*24], merge_network_dot[(`KEYW+`DATW*25)-1:`DATW*25], merge_network_dot[(`KEYW+`DATW*26)-1:`DATW*26], merge_network_dot[(`KEYW+`DATW*27)-1:`DATW*27], merge_network_dot[(`KEYW+`DATW*28)-1:`DATW*28], merge_network_dot[(`KEYW+`DATW*29)-1:`DATW*29], merge_network_dot[(`KEYW+`DATW*30)-1:`DATW*30], merge_network_dot[(`KEYW+`DATW*31)-1:`DATW*31]);
//       endcase
//       $write("\n");
//       $fflush();
//     end
//   end

//   // error checker
//   always @(posedge CLK) begin
//     if (merge_network_doten) begin
//       if (merge_network_dot != check_records) begin
//         $write("\nError!\n"); 
//         $finish();
//       end
//     end
//   end
  
//   // simulation finish condition
//   reg [31:0] cycle;
//   always @(posedge CLK) begin
//     if (RST) begin
//       cycle <= 0;
//     end else begin
//       cycle <= cycle + 1;
//       if (cycle >= 100) $finish();
//     end
//   end
    
// endmodule


// module tb_MERGE_LOGIC();
//   reg CLK; initial begin CLK=0; forever #50 CLK=~CLK; end
//   reg RST; initial begin RST=1; #400 RST=0; end

//   wire                       fifo4odd_enq;
//   wire [(`DATW<<`E_LOG)-1:0] fifo4odd_din;
//   wire                       fifo4odd_ful;
  
//   wire                       fifo4even_enq;
//   wire [(`DATW<<`E_LOG)-1:0] fifo4even_din;
//   wire                       fifo4even_ful;
  
//   wire [(`DATW<<`E_LOG)-1:0] merge_logic_dot;
//   wire                       merge_logic_doten;
  
//   wire [(`DATW<<`E_LOG)-1:0] check_records;
  
//   assign fifo4odd_enq  = !fifo4odd_ful;
//   assign fifo4even_enq = !fifo4even_ful;

//   genvar i;
//   generate
//     for (i=0; i<(1<<`E_LOG); i=i+1) begin: loop
//       wire [`KEYW-1:0] init_keyodd  = 2 * i + 1;
//       wire [`KEYW-1:0] init_keyeven = 2 * i + 2;
//       wire [`KEYW-1:0] chk_rslt_key = i + 1;
//       reg  [`DATW-1:0] fifo4odd_initrecord;
//       reg  [`DATW-1:0] fifo4even_initrecord;
//       reg  [`DATW-1:0] chk_rslt;
//       always @(posedge CLK) begin
//         if      (RST)          fifo4odd_initrecord <= {{(`DATW-`KEYW){1'b1}}, init_keyodd};
//         else if (fifo4odd_enq) fifo4odd_initrecord <= fifo4odd_initrecord + (1<<(`E_LOG+1));
//       end
//       always @(posedge CLK) begin
//         if      (RST)           fifo4even_initrecord <= {{(`DATW-`KEYW){1'b1}}, init_keyeven};
//         else if (fifo4even_enq) fifo4even_initrecord <= fifo4even_initrecord + (1<<(`E_LOG+1));
//       end
//       always @(posedge CLK) begin
//         if      (RST)               chk_rslt <= {{(`DATW-`KEYW){1'b1}}, chk_rslt_key};
//         else if (merge_logic_doten) chk_rslt <= chk_rslt + (1<<`E_LOG);
//       end
//       assign fifo4odd_din[`DATW*(i+1)-1:`DATW*i]  = fifo4odd_initrecord;
//       assign fifo4even_din[`DATW*(i+1)-1:`DATW*i] = fifo4even_initrecord;
//       assign check_records[`DATW*(i+1)-1:`DATW*i] = chk_rslt;
//     end
//   endgenerate

//   MERGE_LOGIC #(`E_LOG, `DATW, `KEYW)
//   merge_logic(CLK, RST, 1'b0, fifo4odd_enq, fifo4odd_din, fifo4even_enq, fifo4even_din, 
//               fifo4odd_ful, fifo4even_ful, merge_logic_dot, merge_logic_doten);
  
//   always @(posedge CLK) begin
//     if (merge_logic_doten) begin
//       case (`E_LOG)
//         1: $write("%d %d ", merge_logic_dot[(`KEYW+`DATW*0)-1:`DATW*0], merge_logic_dot[(`KEYW+`DATW*1)-1:`DATW*1]);
//         2: $write("%d %d %d %d ", merge_logic_dot[(`KEYW+`DATW*0)-1:`DATW*0], merge_logic_dot[(`KEYW+`DATW*1)-1:`DATW*1], merge_logic_dot[(`KEYW+`DATW*2)-1:`DATW*2], merge_logic_dot[(`KEYW+`DATW*3)-1:`DATW*3]);
//         3: $write("%d %d %d %d %d %d %d %d ", merge_logic_dot[(`KEYW+`DATW*0)-1:`DATW*0], merge_logic_dot[(`KEYW+`DATW*1)-1:`DATW*1], merge_logic_dot[(`KEYW+`DATW*2)-1:`DATW*2], merge_logic_dot[(`KEYW+`DATW*3)-1:`DATW*3], merge_logic_dot[(`KEYW+`DATW*4)-1:`DATW*4], merge_logic_dot[(`KEYW+`DATW*5)-1:`DATW*5], merge_logic_dot[(`KEYW+`DATW*6)-1:`DATW*6], merge_logic_dot[(`KEYW+`DATW*7)-1:`DATW*7]);
//         4: $write("%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d ", merge_logic_dot[(`KEYW+`DATW*0)-1:`DATW*0], merge_logic_dot[(`KEYW+`DATW*1)-1:`DATW*1], merge_logic_dot[(`KEYW+`DATW*2)-1:`DATW*2], merge_logic_dot[(`KEYW+`DATW*3)-1:`DATW*3], merge_logic_dot[(`KEYW+`DATW*4)-1:`DATW*4], merge_logic_dot[(`KEYW+`DATW*5)-1:`DATW*5], merge_logic_dot[(`KEYW+`DATW*6)-1:`DATW*6], merge_logic_dot[(`KEYW+`DATW*7)-1:`DATW*7], merge_logic_dot[(`KEYW+`DATW*8)-1:`DATW*8], merge_logic_dot[(`KEYW+`DATW*9)-1:`DATW*9], merge_logic_dot[(`KEYW+`DATW*10)-1:`DATW*10], merge_logic_dot[(`KEYW+`DATW*11)-1:`DATW*11], merge_logic_dot[(`KEYW+`DATW*12)-1:`DATW*12], merge_logic_dot[(`KEYW+`DATW*13)-1:`DATW*13], merge_logic_dot[(`KEYW+`DATW*14)-1:`DATW*14], merge_logic_dot[(`KEYW+`DATW*15)-1:`DATW*15]);
//         5: $write("%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d ", merge_logic_dot[(`KEYW+`DATW*0)-1:`DATW*0], merge_logic_dot[(`KEYW+`DATW*1)-1:`DATW*1], merge_logic_dot[(`KEYW+`DATW*2)-1:`DATW*2], merge_logic_dot[(`KEYW+`DATW*3)-1:`DATW*3], merge_logic_dot[(`KEYW+`DATW*4)-1:`DATW*4], merge_logic_dot[(`KEYW+`DATW*5)-1:`DATW*5], merge_logic_dot[(`KEYW+`DATW*6)-1:`DATW*6], merge_logic_dot[(`KEYW+`DATW*7)-1:`DATW*7], merge_logic_dot[(`KEYW+`DATW*8)-1:`DATW*8], merge_logic_dot[(`KEYW+`DATW*9)-1:`DATW*9], merge_logic_dot[(`KEYW+`DATW*10)-1:`DATW*10], merge_logic_dot[(`KEYW+`DATW*11)-1:`DATW*11], merge_logic_dot[(`KEYW+`DATW*12)-1:`DATW*12], merge_logic_dot[(`KEYW+`DATW*13)-1:`DATW*13], merge_logic_dot[(`KEYW+`DATW*14)-1:`DATW*14], merge_logic_dot[(`KEYW+`DATW*15)-1:`DATW*15], merge_logic_dot[(`KEYW+`DATW*16)-1:`DATW*16], merge_logic_dot[(`KEYW+`DATW*17)-1:`DATW*17], merge_logic_dot[(`KEYW+`DATW*18)-1:`DATW*18], merge_logic_dot[(`KEYW+`DATW*19)-1:`DATW*19], merge_logic_dot[(`KEYW+`DATW*20)-1:`DATW*20], merge_logic_dot[(`KEYW+`DATW*21)-1:`DATW*21], merge_logic_dot[(`KEYW+`DATW*22)-1:`DATW*22], merge_logic_dot[(`KEYW+`DATW*23)-1:`DATW*23], merge_logic_dot[(`KEYW+`DATW*24)-1:`DATW*24], merge_logic_dot[(`KEYW+`DATW*25)-1:`DATW*25], merge_logic_dot[(`KEYW+`DATW*26)-1:`DATW*26], merge_logic_dot[(`KEYW+`DATW*27)-1:`DATW*27], merge_logic_dot[(`KEYW+`DATW*28)-1:`DATW*28], merge_logic_dot[(`KEYW+`DATW*29)-1:`DATW*29], merge_logic_dot[(`KEYW+`DATW*30)-1:`DATW*30], merge_logic_dot[(`KEYW+`DATW*31)-1:`DATW*31]);
//       endcase
//       $write("\n");
//       $fflush();
//     end
//   end

//   // error checker
//   always @(posedge CLK) begin
//     if (merge_logic_doten) begin
//       if (merge_logic_dot != check_records) begin
//         $write("\nError!\n"); 
//         $finish();
//       end
//     end
//   end
  
//   // simulation finish condition
//   reg [31:0] cycle;
//   always @(posedge CLK) begin
//     if (RST) begin
//       cycle <= 0;
//     end else begin
//       cycle <= cycle + 1;
//       if (cycle >= 100) $finish();
//     end
//   end
  
// endmodule


// module tb_MERGE_NODE();
//   reg CLK; initial begin CLK=0; forever #50 CLK=~CLK; end
//   reg RST; initial begin RST=1; #400 RST=0; end
  
//   wire [(`DATW<<(`E_LOG-1))-1:0] din_a;
//   wire                           dinen_a;
//   wire                           full_a;
  
//   wire [(`DATW<<(`E_LOG-1))-1:0] din_b;
//   wire                           dinen_b;
//   wire                           full_b;
  
//   wire [(`DATW<<`E_LOG)-1:0]     merge_node_dot;
//   wire                           merge_node_doten;
  
//   wire [(`DATW<<`E_LOG)-1:0]     fifo4odd_din;
//   wire [(`DATW<<`E_LOG)-1:0]     fifo4even_din;
  
//   wire [(`DATW<<`E_LOG)-1:0]     check_records;

//   reg                            stall_a;
//   reg                            stall_b;
     
//   reg                            flip_selector_a;
//   reg                            flip_selector_b;

//   always @(posedge CLK) stall_a <= full_a;
//   always @(posedge CLK) stall_b <= full_b;

//   always @(posedge CLK) begin
//     if      (RST)     flip_selector_a <= 0;
//     else if (dinen_a) flip_selector_a <= ~flip_selector_a;
//   end
//   always @(posedge CLK) begin
//     if      (RST)     flip_selector_b <= 0;
//     else if (dinen_b) flip_selector_b <= ~flip_selector_b;
//   end

//   genvar i;
//   generate
//     for (i=0; i<(1<<`E_LOG); i=i+1) begin: loop
//       wire [`KEYW-1:0] init_keyodd  = 2 * i + 1;
//       wire [`KEYW-1:0] init_keyeven = 2 * i + 2;
//       wire [`KEYW-1:0] chk_rslt_key = i + 1;
//       reg  [`DATW-1:0] fifo4odd_initrecord;
//       reg  [`DATW-1:0] fifo4even_initrecord;
//       reg  [`DATW-1:0] chk_rslt;
//       always @(posedge CLK) begin
//         if      (RST)                        fifo4odd_initrecord <= {{(`DATW-`KEYW){1'b1}}, init_keyodd};
//         else if (dinen_a && flip_selector_a) fifo4odd_initrecord <= fifo4odd_initrecord + (1<<(`E_LOG+1));
//       end
//       always @(posedge CLK) begin
//         if      (RST)                        fifo4even_initrecord <= {{(`DATW-`KEYW){1'b1}}, init_keyeven};
//         else if (dinen_b && flip_selector_b) fifo4even_initrecord <= fifo4even_initrecord + (1<<(`E_LOG+1));
//       end
//       always @(posedge CLK) begin
//         if      (RST)              chk_rslt <= {{(`DATW-`KEYW){1'b1}}, chk_rslt_key};
//         else if (merge_node_doten) chk_rslt <= chk_rslt + (1<<`E_LOG);
//       end
//       assign fifo4odd_din[`DATW*(i+1)-1:`DATW*i]  = fifo4odd_initrecord;
//       assign fifo4even_din[`DATW*(i+1)-1:`DATW*i] = fifo4even_initrecord;
//       assign check_records[`DATW*(i+1)-1:`DATW*i] = chk_rslt;
//     end
//   endgenerate

//   assign din_a = (flip_selector_a) ? fifo4odd_din[(`DATW<<`E_LOG)-1:(`DATW<<(`E_LOG-1))] : fifo4odd_din[(`DATW<<(`E_LOG-1))-1:0];
//   assign din_b = (flip_selector_b) ? fifo4even_din[(`DATW<<`E_LOG)-1:(`DATW<<(`E_LOG-1))] : fifo4even_din[(`DATW<<(`E_LOG-1))-1:0];

//   assign dinen_a = ~stall_a;
//   assign dinen_b = ~stall_b;

//   MERGE_NODE #(`E_LOG, `DATW, `KEYW)
//   merge_node(CLK, RST, 1'b0, din_a, dinen_a, din_b, dinen_b, 
//              full_a, full_b, merge_node_dot, merge_node_doten);

//   always @(posedge CLK) begin
//     if (merge_node_doten) begin
//       case (`E_LOG)
//         1: $write("%d %d ", merge_node_dot[(`KEYW+`DATW*0)-1:`DATW*0], merge_node_dot[(`KEYW+`DATW*1)-1:`DATW*1]);
//         2: $write("%d %d %d %d ", merge_node_dot[(`KEYW+`DATW*0)-1:`DATW*0], merge_node_dot[(`KEYW+`DATW*1)-1:`DATW*1], merge_node_dot[(`KEYW+`DATW*2)-1:`DATW*2], merge_node_dot[(`KEYW+`DATW*3)-1:`DATW*3]);
//         3: $write("%d %d %d %d %d %d %d %d ", merge_node_dot[(`KEYW+`DATW*0)-1:`DATW*0], merge_node_dot[(`KEYW+`DATW*1)-1:`DATW*1], merge_node_dot[(`KEYW+`DATW*2)-1:`DATW*2], merge_node_dot[(`KEYW+`DATW*3)-1:`DATW*3], merge_node_dot[(`KEYW+`DATW*4)-1:`DATW*4], merge_node_dot[(`KEYW+`DATW*5)-1:`DATW*5], merge_node_dot[(`KEYW+`DATW*6)-1:`DATW*6], merge_node_dot[(`KEYW+`DATW*7)-1:`DATW*7]);
//         4: $write("%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d ", merge_node_dot[(`KEYW+`DATW*0)-1:`DATW*0], merge_node_dot[(`KEYW+`DATW*1)-1:`DATW*1], merge_node_dot[(`KEYW+`DATW*2)-1:`DATW*2], merge_node_dot[(`KEYW+`DATW*3)-1:`DATW*3], merge_node_dot[(`KEYW+`DATW*4)-1:`DATW*4], merge_node_dot[(`KEYW+`DATW*5)-1:`DATW*5], merge_node_dot[(`KEYW+`DATW*6)-1:`DATW*6], merge_node_dot[(`KEYW+`DATW*7)-1:`DATW*7], merge_node_dot[(`KEYW+`DATW*8)-1:`DATW*8], merge_node_dot[(`KEYW+`DATW*9)-1:`DATW*9], merge_node_dot[(`KEYW+`DATW*10)-1:`DATW*10], merge_node_dot[(`KEYW+`DATW*11)-1:`DATW*11], merge_node_dot[(`KEYW+`DATW*12)-1:`DATW*12], merge_node_dot[(`KEYW+`DATW*13)-1:`DATW*13], merge_node_dot[(`KEYW+`DATW*14)-1:`DATW*14], merge_node_dot[(`KEYW+`DATW*15)-1:`DATW*15]);
//         5: $write("%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d ", merge_node_dot[(`KEYW+`DATW*0)-1:`DATW*0], merge_node_dot[(`KEYW+`DATW*1)-1:`DATW*1], merge_node_dot[(`KEYW+`DATW*2)-1:`DATW*2], merge_node_dot[(`KEYW+`DATW*3)-1:`DATW*3], merge_node_dot[(`KEYW+`DATW*4)-1:`DATW*4], merge_node_dot[(`KEYW+`DATW*5)-1:`DATW*5], merge_node_dot[(`KEYW+`DATW*6)-1:`DATW*6], merge_node_dot[(`KEYW+`DATW*7)-1:`DATW*7], merge_node_dot[(`KEYW+`DATW*8)-1:`DATW*8], merge_node_dot[(`KEYW+`DATW*9)-1:`DATW*9], merge_node_dot[(`KEYW+`DATW*10)-1:`DATW*10], merge_node_dot[(`KEYW+`DATW*11)-1:`DATW*11], merge_node_dot[(`KEYW+`DATW*12)-1:`DATW*12], merge_node_dot[(`KEYW+`DATW*13)-1:`DATW*13], merge_node_dot[(`KEYW+`DATW*14)-1:`DATW*14], merge_node_dot[(`KEYW+`DATW*15)-1:`DATW*15], merge_node_dot[(`KEYW+`DATW*16)-1:`DATW*16], merge_node_dot[(`KEYW+`DATW*17)-1:`DATW*17], merge_node_dot[(`KEYW+`DATW*18)-1:`DATW*18], merge_node_dot[(`KEYW+`DATW*19)-1:`DATW*19], merge_node_dot[(`KEYW+`DATW*20)-1:`DATW*20], merge_node_dot[(`KEYW+`DATW*21)-1:`DATW*21], merge_node_dot[(`KEYW+`DATW*22)-1:`DATW*22], merge_node_dot[(`KEYW+`DATW*23)-1:`DATW*23], merge_node_dot[(`KEYW+`DATW*24)-1:`DATW*24], merge_node_dot[(`KEYW+`DATW*25)-1:`DATW*25], merge_node_dot[(`KEYW+`DATW*26)-1:`DATW*26], merge_node_dot[(`KEYW+`DATW*27)-1:`DATW*27], merge_node_dot[(`KEYW+`DATW*28)-1:`DATW*28], merge_node_dot[(`KEYW+`DATW*29)-1:`DATW*29], merge_node_dot[(`KEYW+`DATW*30)-1:`DATW*30], merge_node_dot[(`KEYW+`DATW*31)-1:`DATW*31]);
//       endcase
//       $write("\n");
//       $fflush();
//     end
//   end

//   // error checker
//   always @(posedge CLK) begin
//     if (merge_node_doten) begin
//       if (merge_node_dot != check_records) begin
//         $write("\nError!\n"); 
//         $finish();
//       end
//     end
//   end
  
//   // simulation finish condition
//   reg [31:0] cycle;
//   always @(posedge CLK) begin
//     if (RST) begin
//       cycle <= 0;
//     end else begin
//       cycle <= cycle + 1;
//       if (cycle >= 100) $finish();
//     end
//   end
  
// endmodule


module tb_MERGE_TREE();
  reg CLK; initial begin CLK=0; forever #50 CLK=~CLK; end
  reg RST; initial begin RST=1; #400 RST=0; end
  
  wire [(`DATW<<`E_LOG)-1:0] merge_tree_din;
  wire [(1<<`E_LOG)-1:0]     merge_tree_dinen;
  wire [(1<<`E_LOG)-1:0]     merge_tree_ful;
  wire [(`DATW<<`E_LOG)-1:0] merge_tree_dot;
  wire                       merge_tree_doten;

  wire [(`DATW<<`E_LOG)-1:0] check_records;

  assign merge_tree_dinen = ~merge_tree_ful;
  
  genvar i;
  generate
    for (i=0; i<(1<<`E_LOG); i=i+1) begin: loop
      wire [`KEYW-1:0] init_key     = (1<<`E_LOG) - i;
      wire [`KEYW-1:0] chk_rslt_key = i + 1;
      reg  [`DATW-1:0] init_record;
      reg  [`DATW-1:0] chk_rslt;
      always @(posedge CLK) begin
        if      (RST)                 init_record <= {{(`DATW-`KEYW){1'b1}}, init_key};
        else if (merge_tree_dinen[i]) init_record <= init_record + (1<<`E_LOG);
      end
      always @(posedge CLK) begin
        if      (RST)              chk_rslt <= {{(`DATW-`KEYW){1'b1}}, chk_rslt_key};
        else if (merge_tree_doten) chk_rslt <= chk_rslt + (1<<`E_LOG);
      end
      assign merge_tree_din[`DATW*(i+1)-1:`DATW*i] = init_record;
      assign check_records[`DATW*(i+1)-1:`DATW*i]  = chk_rslt;
    end
  endgenerate

  MERGE_TREE #(`E_LOG, `DATW, `KEYW)
  merge_tree(CLK, RST, 1'b0, merge_tree_din, merge_tree_dinen, 
             merge_tree_ful, merge_tree_dot, merge_tree_doten);

  // show result
  always @(posedge CLK) begin
    if (merge_tree_doten) begin
      case (`E_LOG)
        1: $write("%d %d ", merge_tree_dot[(`KEYW+`DATW*0)-1:`DATW*0], merge_tree_dot[(`KEYW+`DATW*1)-1:`DATW*1]);
        2: $write("%d %d %d %d ", merge_tree_dot[(`KEYW+`DATW*0)-1:`DATW*0], merge_tree_dot[(`KEYW+`DATW*1)-1:`DATW*1], merge_tree_dot[(`KEYW+`DATW*2)-1:`DATW*2], merge_tree_dot[(`KEYW+`DATW*3)-1:`DATW*3]);
        3: $write("%d %d %d %d %d %d %d %d ", merge_tree_dot[(`KEYW+`DATW*0)-1:`DATW*0], merge_tree_dot[(`KEYW+`DATW*1)-1:`DATW*1], merge_tree_dot[(`KEYW+`DATW*2)-1:`DATW*2], merge_tree_dot[(`KEYW+`DATW*3)-1:`DATW*3], merge_tree_dot[(`KEYW+`DATW*4)-1:`DATW*4], merge_tree_dot[(`KEYW+`DATW*5)-1:`DATW*5], merge_tree_dot[(`KEYW+`DATW*6)-1:`DATW*6], merge_tree_dot[(`KEYW+`DATW*7)-1:`DATW*7]);
        4: $write("%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d ", merge_tree_dot[(`KEYW+`DATW*0)-1:`DATW*0], merge_tree_dot[(`KEYW+`DATW*1)-1:`DATW*1], merge_tree_dot[(`KEYW+`DATW*2)-1:`DATW*2], merge_tree_dot[(`KEYW+`DATW*3)-1:`DATW*3], merge_tree_dot[(`KEYW+`DATW*4)-1:`DATW*4], merge_tree_dot[(`KEYW+`DATW*5)-1:`DATW*5], merge_tree_dot[(`KEYW+`DATW*6)-1:`DATW*6], merge_tree_dot[(`KEYW+`DATW*7)-1:`DATW*7], merge_tree_dot[(`KEYW+`DATW*8)-1:`DATW*8], merge_tree_dot[(`KEYW+`DATW*9)-1:`DATW*9], merge_tree_dot[(`KEYW+`DATW*10)-1:`DATW*10], merge_tree_dot[(`KEYW+`DATW*11)-1:`DATW*11], merge_tree_dot[(`KEYW+`DATW*12)-1:`DATW*12], merge_tree_dot[(`KEYW+`DATW*13)-1:`DATW*13], merge_tree_dot[(`KEYW+`DATW*14)-1:`DATW*14], merge_tree_dot[(`KEYW+`DATW*15)-1:`DATW*15]);
        5: $write("%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d ", merge_tree_dot[(`KEYW+`DATW*0)-1:`DATW*0], merge_tree_dot[(`KEYW+`DATW*1)-1:`DATW*1], merge_tree_dot[(`KEYW+`DATW*2)-1:`DATW*2], merge_tree_dot[(`KEYW+`DATW*3)-1:`DATW*3], merge_tree_dot[(`KEYW+`DATW*4)-1:`DATW*4], merge_tree_dot[(`KEYW+`DATW*5)-1:`DATW*5], merge_tree_dot[(`KEYW+`DATW*6)-1:`DATW*6], merge_tree_dot[(`KEYW+`DATW*7)-1:`DATW*7], merge_tree_dot[(`KEYW+`DATW*8)-1:`DATW*8], merge_tree_dot[(`KEYW+`DATW*9)-1:`DATW*9], merge_tree_dot[(`KEYW+`DATW*10)-1:`DATW*10], merge_tree_dot[(`KEYW+`DATW*11)-1:`DATW*11], merge_tree_dot[(`KEYW+`DATW*12)-1:`DATW*12], merge_tree_dot[(`KEYW+`DATW*13)-1:`DATW*13], merge_tree_dot[(`KEYW+`DATW*14)-1:`DATW*14], merge_tree_dot[(`KEYW+`DATW*15)-1:`DATW*15], merge_tree_dot[(`KEYW+`DATW*16)-1:`DATW*16], merge_tree_dot[(`KEYW+`DATW*17)-1:`DATW*17], merge_tree_dot[(`KEYW+`DATW*18)-1:`DATW*18], merge_tree_dot[(`KEYW+`DATW*19)-1:`DATW*19], merge_tree_dot[(`KEYW+`DATW*20)-1:`DATW*20], merge_tree_dot[(`KEYW+`DATW*21)-1:`DATW*21], merge_tree_dot[(`KEYW+`DATW*22)-1:`DATW*22], merge_tree_dot[(`KEYW+`DATW*23)-1:`DATW*23], merge_tree_dot[(`KEYW+`DATW*24)-1:`DATW*24], merge_tree_dot[(`KEYW+`DATW*25)-1:`DATW*25], merge_tree_dot[(`KEYW+`DATW*26)-1:`DATW*26], merge_tree_dot[(`KEYW+`DATW*27)-1:`DATW*27], merge_tree_dot[(`KEYW+`DATW*28)-1:`DATW*28], merge_tree_dot[(`KEYW+`DATW*29)-1:`DATW*29], merge_tree_dot[(`KEYW+`DATW*30)-1:`DATW*30], merge_tree_dot[(`KEYW+`DATW*31)-1:`DATW*31]);
      endcase
      $write("\n");
      $fflush();
    end
  end

  // error checker
  always @(posedge CLK) begin
    if (merge_tree_doten) begin
      if (merge_tree_dot != check_records) begin
        $write("\nError!\n"); 
        $finish();
      end
    end
  end
  
  // simulation finish condition
  reg [31:0] cycle;
  always @(posedge CLK) begin
    if (RST) begin
      cycle <= 0;
    end else begin
      cycle <= cycle + 1;
      if (cycle >= 200) $finish();
    end
  end

endmodule

`default_nettype wire
