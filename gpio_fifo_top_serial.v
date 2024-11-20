`timescale 1ns / 1ps

module gpio_fifo_top_serial (
    input wire wclk,         // Write clock for tx_fifo
    input wire wrst_n,       // Reset for tx_fifo
    input wire winc,         // Write enable for tx_fifo
    input wire [7:0] wdata,  // Data from testbench to tx_fifo

    input wire rclk,         // Read clock for GPIO operations
    input wire rrst_n,       // Reset for GPIO operations

    input wire gpio_direction, // Direction control from testbench (1 = Transmit, 0 = Receive)
    input wire gpio_in,        // Serial input to GPIO pin
    output wire serial_out,    // Serial data output from GPIO pin
    output wire [7:0] pin_status // GPIO pin status (8-bit received data)
);

    // Internal signals
    wire [7:0] tx_fifo_rdata;
    wire tx_fifo_rempty;

    wire gpio_interrupt;     // Interrupt from GPIO block for new data load
    reg [7:0] gpio_data_in;  // Internal buffer for data to GPIO block

    // Assign FIFO Read Control to GPIO Interrupt
    assign tx_fifo_rinc = gpio_interrupt && !tx_fifo_rempty;

    // Instantiate tx_fifo
    async_fifo1 #(8, 4) tx_fifo (
        .wdata(wdata),
        .winc(winc),
        .wclk(wclk),
        .wrst_n(wrst_n),
        .rdata(tx_fifo_rdata),
        .rinc(tx_fifo_rinc),    // Controlled by GPIO interrupt
        .rclk(rclk),
        .rrst_n(rrst_n),
        .wfull(),
        .rempty(tx_fifo_rempty)
    );

    // Instantiate GPIO block
    gpio_block gpio_inst (
        .clk(rclk),
        .rst_n(rrst_n),
        .data_in(gpio_data_in),     // Data to be transmitted serially
        .direction(gpio_direction), // Direction control from testbench
        .gpio_out(serial_out),      // Serial output
        .gpio_in(gpio_in),          // Serial input
        .pin_status(pin_status),    // Received 8-bit data
        .interrupt(gpio_interrupt)  // Interrupt for data reload
    );

    // Control Logic for GPIO Data Load
    always @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n) begin
            gpio_data_in <= 8'b0;
        end else if (gpio_interrupt && !tx_fifo_rempty) begin
            // Load new data from tx_fifo into GPIO block
            gpio_data_in <= tx_fifo_rdata;
        end
    end

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
    input wire clk,                  // Clock signal
    input wire rst_n,                // Active low reset
    input wire [7:0] data_in,        // 8-bit input data for GPIO pins
    input wire direction,            // Direction register: 1 = Transmit, 0 = Receive
    output reg gpio_out,             // Serial output to the GPIO pin
    input wire gpio_in,              // Serial input from the GPIO pin
    output reg [7:0] pin_status,     // 8-bit status register for GPIO pins
    output reg interrupt             // Interrupt signal for data reload
);

    reg [7:0] shift_reg;             // Shift register for serial data transmission
    reg [2:0] bit_counter;           // Counter to track the bit position during transmission/reception
    reg transmit_active;             // Flag to indicate active transmission

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shift_reg <= 8'b0;       // Reset the shift register
            bit_counter <= 3'b0;    // Reset the bit counter
            gpio_out <= 1'b0;       // Reset the GPIO output
            pin_status <= 8'b0;     // Reset the pin status register
            interrupt <= 1'b0;      // Reset the interrupt signal
            transmit_active <= 1'b0; // Reset transmit active flag
        end else begin
            if (direction) begin
                // Transmit Mode
                if (!transmit_active) begin
                    // Load new data into the shift register at the beginning of transmission
                    shift_reg <= data_in;
                    bit_counter <= 3'b0;     // Reset the bit counter
                    transmit_active <= 1'b1; // Set transmit active flag
                    interrupt <= 1'b0;       // Clear interrupt
                end else begin
                    // Transmitting data bit by bit
                    gpio_out <= shift_reg[7]; // Send the MSB
                    shift_reg <= {shift_reg[6:0], 1'b0}; // Shift left to prepare next bit
                    bit_counter <= bit_counter + 1'b1; // Increment bit counter

                    if (bit_counter == 3'b111) begin
                        // Transmission complete
                        transmit_active <= 1'b0; // Clear transmit active flag
                        interrupt <= 1'b1;       // Raise interrupt for data reload
                    end
                end
            end else begin
                // Receive Mode
                shift_reg <= {shift_reg[6:0], gpio_in}; // Shift in data serially
                bit_counter <= bit_counter + 1'b1;      // Increment bit counter

                if (bit_counter == 3'b111) begin
                    // Reception complete, update pin_status
                    pin_status <= shift_reg;
                    bit_counter <= 3'b0; // Reset bit counter after full reception
                end
            end
        end
    end

endmodule

