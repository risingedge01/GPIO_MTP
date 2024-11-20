// Code your design here
//@copyright24
//TOP Module

`timescale 1ns / 1ps

module gpio_fifo_top (
    input wire wclk,         // Write clock for tx_fifo
    input wire wrst_n,       // Reset for tx_fifo
    input wire winc,         // Write enable for tx_fifo
    input wire [7:0] wdata,  // Data from testbench to tx_fifo

    input wire rclk,         // Read clock for rx_fifo
    input wire rrst_n,       // Reset for rx_fifo

    input wire [7:0] gpio_in,     // Input from GPIO pins
    output wire [7:0] gpio_out,   // Output to GPIO pins
    output wire [7:0] pin_status  // GPIO pin status
);

    // Internal signals
    wire [7:0] tx_fifo_rdata;
    wire tx_fifo_rempty;
    wire tx_fifo_rinc;

    wire [7:0] rx_fifo_wdata;
    wire rx_fifo_wfull;
    wire rx_fifo_winc;

    wire [7:0] gpio_data_in;
    wire [7:0] gpio_direction;

    // Instantiate tx_fifo
    async_fifo1 #(8, 4) tx_fifo (
        .wdata(wdata),
        .winc(winc),
        .wclk(wclk),
        .wrst_n(wrst_n),
        .rdata(tx_fifo_rdata),
        .rinc(tx_fifo_rinc),
        .rclk(rclk),
        .rrst_n(rrst_n),
        .wfull(),
        .rempty(tx_fifo_rempty)
    );

    // Instantiate GPIO block
    gpio_block gpio_inst (
        .clk(rclk),
        .rst_n(rrst_n),
        .data_in(tx_fifo_rdata),
        .direction(gpio_direction),  // External control of direction register
        .gpio_out(gpio_out),
        .gpio_in(gpio_in),
        .pin_status(pin_status)
    );

    // Instantiate rx_fifo
    async_fifo1 #(8, 4) rx_fifo (
        .wdata(rx_fifo_wdata),
        .winc(rx_fifo_winc),
        .wclk(rclk),
        .wrst_n(rrst_n),
        .rdata(),  // rx_fifo data not read in this design
        .rinc(1'b0),  // Read disabled for rx_fifo
        .rclk(rclk),
        .rrst_n(rrst_n),
        .wfull(rx_fifo_wfull),
        .rempty()
    );

    // Control logic
    assign tx_fifo_rinc = !tx_fifo_rempty;  // Read from tx_fifo if not empty
    assign gpio_data_in = tx_fifo_rdata;   // Data from tx_fifo to GPIO
    assign gpio_direction = 8'b11111111;   // Example: All GPIO pins set as output
    assign rx_fifo_winc = !rx_fifo_wfull;  // Write to rx_fifo if not full
    assign rx_fifo_wdata = gpio_out;       // Write GPIO output to rx_fifo

endmodule











//FIFO Module

// Name: Ganga Sagar Tripathi
// IIT Madras
//
module async_fifo1
#(
  parameter DSIZE = 8,
  parameter ASIZE = 4
 )
(
  input   winc, wclk, wrst_n,//winc write enable signal
  input   rinc, rclk, rrst_n,//rinc read enable signal
  input   [DSIZE-1:0] wdata,

  output  [DSIZE-1:0] rdata,
  output  wfull,
  output  rempty
);

  wire [ASIZE-1:0] waddr, raddr;
  wire [ASIZE:0] wptr, rptr, wq2_rptr, rq2_wptr;

sync_r2w sync_r2w (.wq2_rptr(wq2_rptr), .rptr(rptr), .wclk(wclk), .wrst_n(wrst_n));
sync_w2r sync_w2r (.rq2_wptr(rq2_wptr), .wptr(wptr), .rclk(rclk), .rrst_n(rrst_n));
  fifomem #(DSIZE, ASIZE) fifomem (.rdata(rdata), .wdata(wdata), .waddr(waddr), .raddr(raddr), .winc(winc), .wfull(wfull), .wclk(wclk));
rptr_empty #(ASIZE) rptr_empty (.rempty(rempty), .raddr(raddr), .rptr(rptr), .rq2_wptr(rq2_wptr), .rinc(rinc), .rclk(rclk), .rrst_n(rrst_n));
wptr_full #(ASIZE) wptr_full (.wfull(wfull), .waddr(waddr), .wptr(wptr), .wq2_rptr(wq2_rptr), .winc(winc), .wclk(wclk), .wrst_n(wrst_n));

endmodule

//
// FIFO memory
//
module fifomem
#(
  parameter DATASIZE = 8, // Memory data word width
  parameter ADDRSIZE = 4  // Number of mem address bits
)
(
  input   winc, wfull, wclk,
  input   [ADDRSIZE-1:0] waddr, raddr,
  input   [DATASIZE-1:0] wdata,
  output  [DATASIZE-1:0] rdata
);

  // RTL Verilog memory model
  localparam DEPTH = 1<<ADDRSIZE;//2*addsize

  reg [DATASIZE-1:0] mem [0:DEPTH-1];

  assign rdata = mem[raddr];

  always @(posedge wclk)
    if (winc && !wfull)
      mem[waddr] <= wdata;
endmodule


//r_pointer_epty.v

module rptr_empty
#(
  parameter ADDRSIZE = 4
)
(
  input   rinc, rclk, rrst_n,
  input   [ADDRSIZE :0] rq2_wptr,
  output reg  rempty,
  output  [ADDRSIZE-1:0] raddr,
  output reg [ADDRSIZE :0] rptr
);

  reg [ADDRSIZE:0] rbin;
  wire [ADDRSIZE:0] rgraynext, rbinnext;

  //-------------------
  // GRAYSTYLE2 pointer
  //-------------------
  always @(posedge rclk or negedge rrst_n)
    if (!rrst_n)
      {rbin, rptr} <= '0;
    else
      {rbin, rptr} <= {rbinnext, rgraynext};

  // Memory read-address pointer (okay to use binary to address memory)
  assign raddr = rbin[ADDRSIZE-1:0];
  assign rbinnext = rbin + (rinc & ~rempty);
  assign rgraynext = (rbinnext>>1) ^ rbinnext;

  //---------------------------------------------------------------
  // FIFO empty when the next rptr == synchronized wptr or on reset
  //---------------------------------------------------------------
  assign rempty_val = (rgraynext == rq2_wptr);

  always @(posedge rclk or negedge rrst_n)
    if (!rrst_n)
      rempty <= 1'b1;
    else
      rempty <= rempty_val;

endmodule

// sync_r2w.v
//
// Read pointer to write clock synchronizer
//
module sync_r2w
#(
  parameter ADDRSIZE = 4
)
(
  input   wclk, wrst_n,
  input   [ADDRSIZE:0] rptr,
  output reg  [ADDRSIZE:0] wq2_rptr//readpointer with write side
);

  reg [ADDRSIZE:0] wq1_rptr;

  always @(posedge wclk or negedge wrst_n)
    if (!wrst_n) {wq2_rptr,wq1_rptr} <= 0;
    else {wq2_rptr,wq1_rptr} <= {wq1_rptr,rptr};

endmodule

//syncw2r.v


module sync_w2r
#(
  parameter ADDRSIZE = 4
)
(
  input   rclk, rrst_n,
  input   [ADDRSIZE:0] wptr,
  output reg [ADDRSIZE:0] rq2_wptr
);

  reg [ADDRSIZE:0] rq1_wptr;

  always @(posedge rclk or negedge rrst_n)
    if (!rrst_n)
      {rq2_wptr,rq1_wptr} <= 0;
    else
      {rq2_wptr,rq1_wptr} <= {rq1_wptr,wptr};

endmodule

//w_ptr_wfull.v

module wptr_full
#(
  parameter ADDRSIZE = 4
)
(
  input   winc, wclk, wrst_n,
  input   [ADDRSIZE :0] wq2_rptr,
  output reg  wfull,
  output  [ADDRSIZE-1:0] waddr,
  output reg [ADDRSIZE :0] wptr
);

   reg [ADDRSIZE:0] wbin;
  wire [ADDRSIZE:0] wgraynext, wbinnext;

  // GRAYSTYLE2 pointer
  always @(posedge wclk or negedge wrst_n)
    if (!wrst_n)
      {wbin, wptr} <= '0;
    else
      {wbin, wptr} <= {wbinnext, wgraynext};

  // Memory write-address pointer (okay to use binary to address memory)
  assign waddr = wbin[ADDRSIZE-1:0];
  assign wbinnext = wbin + (winc & ~wfull);
  assign wgraynext = (wbinnext>>1) ^ wbinnext;

  //------------------------------------------------------------------
  // Simplified version of the three necessary full-tests:
  // assign wfull_val=((wgnext[ADDRSIZE] !=wq2_rptr[ADDRSIZE] ) &&
  // (wgnext[ADDRSIZE-1] !=wq2_rptr[ADDRSIZE-1]) &&
  // (wgnext[ADDRSIZE-2:0]==wq2_rptr[ADDRSIZE-2:0]));
  //------------------------------------------------------------------
  assign wfull_val = (wgraynext=={~wq2_rptr[ADDRSIZE:ADDRSIZE-1], wq2_rptr[ADDRSIZE-2:0]});

  always @(posedge wclk or negedge wrst_n)
    if (!wrst_n)
      wfull <= 1'b0;
    else
      wfull <= wfull_val;

endmodule

//GPIO Module

module gpio_block (
    input wire clk,                   // Clock signal
    input wire rst_n,                 // Active low reset
    input wire [7:0] data_in,         // Input data for GPIO pins
    input wire [7:0] direction,       // Direction register: 1 = Output, 0 = Input
    output wire [7:0] gpio_out,       // Output to GPIO pins
    input wire [7:0] gpio_in,         // Input from GPIO pins
    output wire [7:0] pin_status      // Status register for GPIO pins
);

    reg [7:0] pin_data_reg;           // Data register
    reg [7:0] pin_status_reg;         // Status register

    // Pin Data Register Logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            pin_data_reg <= 8'b0;     // Reset data register to 0
        else
            pin_data_reg <= data_in;  // Update data register
    end

    // Pin Status Register Logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            pin_status_reg <= 8'b0;   // Reset status register to 0
        else
            pin_status_reg <= gpio_in; // Update status register with input values
    end

    // GPIO Output Logic
    assign gpio_out = direction ? pin_data_reg : 8'bZ; // Tri-state output based on direction

    // GPIO Pin Status Logic
    assign pin_status = pin_status_reg;

endmodule
