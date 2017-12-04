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
  
  assign RSLT = (DIN0 < DIN1);
  
endmodule


/***** A control logic to detect both keys are equal                      *****/
/******************************************************************************/
module EQUAL_DETECTOR #(parameter              KEYW = 32)
                       (input  wire [KEYW-1:0] DIN0,
                        input  wire [KEYW-1:0] DIN1,
                        output wire            RSLT);
  
  assign RSLT = (DIN0 == DIN1);
  
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
  mux2(din_a[(DATW<<E_LOG)-1:(DATW<<E_LOG)-DATW], fb_buf, ~comp_rslts[(1<<E_LOG)-1], fb_record);

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
  wire [(DATW<<E_LOG)-1:0]        remaining_records;

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
        mux2(din_b[DATW-1:0], din_b[(DATW+(DATW<<E_LOG))-1:(DATW<<E_LOG)], comp_rslts_buf[0], remaining_records[DATW-1:0]);
      end else begin
        MUX3 #(DATW)
        mux3(din_b[DATW*i-1:DATW*(i-1)], din_b[DATW*(i+1)-1:DATW*i], din_b[(DATW+(DATW<<E_LOG))-1:(DATW<<E_LOG)], comp_rslts_buf[i:i-1], remaining_records[DATW*(i+1)-1:DATW*i]);
      end
    end
  endgenerate

  // Output
  assign DOT   = remaining_records;
  assign DOTEN = dinen_b;
  
endmodule


/***** An E-record merge network                                          *****/
/******************************************************************************/
module MERGE_NETWORK #(parameter                       E_LOG = 2,
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
  generate
    for (i=0; i<(1<<E_LOG); i=i+1) begin: sort_logics
      wire [(DATW<<E_LOG)-1:0] din;
      wire                     dinen;
      wire [(DATW<<E_LOG)-1:0] dot;
      wire                     doten;
      SORT_LOGIC #(E_LOG, DATW, KEYW)
      sort_logic(CLK, RST, STALL, din, dinen, dot, doten);
    end
  endgenerate

  generate
    for (i=0; i<(1<<E_LOG); i=i+1) begin: connection
      if (i == 0) begin
        assign sort_logics[0].din   = DIN;
        assign sort_logics[0].dinen = DINEN;
      end else begin
        assign sort_logics[i].din   = sort_logics[i-1].dot;
        assign sort_logics[i].dinen = sort_logics[i-1].doten;
      end
    end
  endgenerate

  reg init_record_ejected;
  always @(posedge CLK) begin
    if      (RST)                             init_record_ejected <= 0;
    else if (sort_logics[(1<<E_LOG)-1].doten) init_record_ejected <= 1;
  end

  // Output
  assign DOT   = sort_logics[(1<<E_LOG)-1].dot;
  assign DOTEN = sort_logics[(1<<E_LOG)-1].doten & init_record_ejected;
  
endmodule
  

/***** An SRL-based FIFO                                                  *****/
/******************************************************************************/
module SRL_FIFO #(parameter                    FIFO_SIZE  = 4,   // size in log scale, 4 for 16 entry
                  parameter                    FIFO_WIDTH = 64)  // fifo width in bit
                 (input  wire                  CLK,
                  input  wire                  RST,
                  input  wire                  enq,
                  input  wire                  deq,
                  input  wire [FIFO_WIDTH-1:0] din,
                  output wire [FIFO_WIDTH-1:0] dot,
                  output wire                  emp,
                  output wire                  full,
                  output reg  [FIFO_SIZE:0]    cnt);

  reg  [FIFO_SIZE-1:0]  head;
  reg  [FIFO_WIDTH-1:0] mem [(1<<FIFO_SIZE)-1:0];
  
  assign emp  = (cnt == 0);
  assign full = (cnt >= (1<<FIFO_SIZE)-1);  // to store this stall signal in a register (note!!!)
  assign dot  = mem[head];
    
  always @(posedge CLK) begin
    if (RST) begin
      cnt  <= 0;
      head <= {(FIFO_SIZE){1'b1}};
    end else begin
      case ({enq, deq})
        2'b01: begin cnt <= cnt - 1; head <= head - 1; end
        2'b10: begin cnt <= cnt + 1; head <= head + 1; end
      endcase
    end
  end

  integer i;
  always @(posedge CLK) begin
    if (enq) begin
      mem[0] <= din;
      for (i=1; i<(1<<FIFO_SIZE); i=i+1) mem[i] <= mem[i-1];
    end
  end
  
endmodule


/***** A selector logic                                                   *****/
/******************************************************************************/
module SELECTOR_LOGIC #(parameter                       E_LOG = 2,
                        parameter                       DATW  = 64,
                        parameter                       KEYW  = 32)
                       (input  wire                     CLK,
                        input  wire                     RST,
                        input  wire                     STALL,
                        input  wire [(DATW<<E_LOG)-1:0] DIN_A,
                        input  wire                     EMP_A,
                        input  wire [(DATW<<E_LOG)-1:0] DIN_B,
                        input  wire                     EMP_B,
                        output wire                     DEQ_A,
                        output wire                     DEQ_B,
                        output wire [(DATW<<E_LOG)-1:0] DOT,
                        output wire                     DOTEN);

  reg  [(DATW<<E_LOG)-1:0] A0, A1, B0, B1;
  reg                      A_in, B_in;
  reg                      init_A_done, init_B_done;
  reg                      comp_mux_buf;
  reg                      comp_state;
  reg                      flip_selector;
  wire                     deq_A_initbefore, deq_B_initbefore;
  wire                     deq_A_initafter, deq_B_initafter;
  wire [(DATW<<E_LOG)-1:0] selected_records;
  wire                     comp_rslt_A0B1, comp_rslt_A1B0;
  wire                     is_equal_A0B1, is_equal_A1B0;
  wire                     comp_mux_out;
  wire                     is_equal_mux_out;

  assign deq_A_initbefore = ~|{EMP_A,init_A_done};
  assign deq_B_initbefore = ~|{EMP_B,init_B_done} && init_A_done;

  assign deq_A_initafter = &{(~EMP_A),   comp_mux_buf,  init_B_done, (~STALL)};
  assign deq_B_initafter = &{(~EMP_B), (~comp_mux_buf), init_B_done, (~STALL)};
  
  always @(posedge CLK) begin
    if (RST) begin
      A_in        <= 0;
      B_in        <= 0;
      init_A_done <= 0;
      init_B_done <= 0;
    end else begin
      if (!A_in)            A_in        <= deq_A_initbefore;
      if (!B_in)            B_in        <= deq_B_initbefore;
      if (deq_A_initbefore) init_A_done <= A_in;
      if (deq_B_initbefore) init_B_done <= B_in;
    end
  end

  // Stage 1
  //////////////////////////////////////////////////////////
  always @(posedge CLK) if (DEQ_A) A0 <= DIN_A;
  always @(posedge CLK) if (DEQ_B) B0 <= DIN_B;

  COMPARATOR #(KEYW)
  comparatorA0B1(A0[KEYW-1:0], B1[KEYW-1:0], comp_rslt_A0B1);
  COMPARATOR #(KEYW)
  comparatorA1B0(A1[KEYW-1:0], B0[KEYW-1:0], comp_rslt_A1B0);

  EQUAL_DETECTOR #(KEYW)
  equal_detectorA0B1(A0[KEYW-1:0], B1[KEYW-1:0], is_equal_A0B1);
  EQUAL_DETECTOR #(KEYW)
  equal_detectorA1B0(A1[KEYW-1:0], B0[KEYW-1:0], is_equal_A1B0);
  
  MUX2 #(1) mux2_4_comprslt(comp_rslt_A0B1, comp_rslt_A1B0, comp_mux_buf, comp_mux_out);
  MUX2 #(1) mux2_4_is_equal(is_equal_A0B1, is_equal_A1B0, comp_mux_buf, is_equal_mux_out);
    
  // Stage 2
  //////////////////////////////////////////////////////////
  always @(posedge CLK) if (DEQ_A) A1 <= A0;
  always @(posedge CLK) if (DEQ_B) B1 <= B0;

  always @(posedge CLK) begin
    if (RST) begin
      comp_mux_buf  <= 0;
      comp_state    <= 0;
      flip_selector <= 0;
    end else begin
      case (comp_state)
        0: begin
          if (B_in && deq_B_initbefore) begin
            comp_mux_buf <= comp_mux_out;
            comp_state   <= 1;
          end
        end
        1: begin
          if (deq_A_initafter || deq_B_initafter) begin
            if (is_equal_mux_out) begin comp_mux_buf <= flip_selector; flip_selector <= ~flip_selector; end
            else                  begin comp_mux_buf <= comp_mux_out;                                   end
          end
        end
      endcase
    end
  end
  
  MUX2 #((DATW<<E_LOG)) mux2_4_record(A1, B1, comp_mux_buf, selected_records);

  // Output
  assign DEQ_A = deq_A_initbefore || deq_A_initafter;
  assign DEQ_B = deq_B_initbefore || deq_B_initafter;
  assign DOT   = selected_records;
  assign DOTEN = deq_A_initafter || deq_B_initafter;
  
endmodule
  

/***** A merge logic                                                      *****/
/******************************************************************************/
module MERGE_LOGIC #(parameter                       E_LOG = 2,
                     parameter                       DATW  = 64,
                     parameter                       KEYW  = 32)
                    (input  wire                     CLK,
                     input  wire                     RST,
                     input  wire                     IN_FULL,
                     input  wire                     ENQ_A,
                     input  wire [(DATW<<E_LOG)-1:0] DIN_A,
                     input  wire                     ENQ_B,
                     input  wire [(DATW<<E_LOG)-1:0] DIN_B,
                     output wire                     FUL_A,
                     output wire                     FUL_B,
                     output wire [(DATW<<E_LOG)-1:0] DOT,
                     output wire                     DOTEN);

  // stall signal
  reg stall; 
  always @(posedge CLK) stall <= IN_FULL;

  wire                     fifo_4_regA_deq, fifo_4_regB_deq;
  wire [(DATW<<E_LOG)-1:0] fifo_4_regA_dot, fifo_4_regB_dot;
  wire                     fifo_4_regA_emp, fifo_4_regB_emp;
  wire                     fifo_4_regA_ful, fifo_4_regB_ful;
  wire [4:0]               fifo_4_regA_cnt, fifo_4_regB_cnt;

  wire [(DATW<<E_LOG)-1:0] selected_records;
  wire                     selected_records_valid;

  wire [(DATW<<E_LOG)-1:0] merge_network_dot;
  wire                     merge_network_doten;

  SRL_FIFO #(4, (DATW<<E_LOG))
  fifo_4_regA(CLK, RST, ENQ_A, fifo_4_regA_deq, DIN_A, 
              fifo_4_regA_dot, fifo_4_regA_emp, fifo_4_regA_ful, fifo_4_regA_cnt);
  SRL_FIFO #(4, (DATW<<E_LOG))
  fifo_4_regB(CLK, RST, ENQ_B, fifo_4_regB_deq, DIN_B, 
              fifo_4_regB_dot, fifo_4_regB_emp, fifo_4_regB_ful, fifo_4_regB_cnt);

  SELECTOR_LOGIC #(E_LOG, DATW, KEYW)
  selector_logic(CLK, RST, stall, fifo_4_regA_dot, fifo_4_regA_emp, fifo_4_regB_dot, fifo_4_regB_emp, 
                 fifo_4_regA_deq, fifo_4_regB_deq, selected_records, selected_records_valid);
  
  MERGE_NETWORK #(E_LOG, DATW, KEYW)
  merge_network(CLK, RST, stall, selected_records, selected_records_valid, 
                merge_network_dot, merge_network_doten);

  // Output
  assign FUL_A = fifo_4_regA_ful;
  assign FUL_B = fifo_4_regB_ful;
  assign DOT   = merge_network_dot;
  assign DOTEN = merge_network_doten;
  
endmodule
  
`default_nettype wire
