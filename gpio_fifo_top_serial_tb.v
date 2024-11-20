// Code your testbench here
// or browse Examples
//Name: Mishra ji
`timescale 1ns / 1ps

module gpio_fifo_top_serial_tb;

    // Parameters
    parameter DSIZE = 8;  // Data width
    parameter ASIZE = 4;  // Address width

    // DUT Signals
    reg wclk, wrst_n, winc;
    reg [DSIZE-1:0] wdata;

    reg rclk, rrst_n;
    reg gpio_direction;
    reg gpio_in;
    wire serial_out;
    wire [DSIZE-1:0] pin_status;

    // Instantiate DUT
    gpio_fifo_top_serial dut (
        .wclk(wclk),
        .wrst_n(wrst_n),
        .winc(winc),
        .wdata(wdata),
        .rclk(rclk),
        .rrst_n(rrst_n),
        .gpio_direction(gpio_direction),
        .gpio_in(gpio_in),
        .serial_out(serial_out),
        .pin_status(pin_status)
    );

    // Clock Generation
    initial begin
        wclk = 1'b0;
        forever #10 wclk = ~wclk;  // Write clock: 20ns period
    end

    initial begin
        rclk = 1'b0;
        forever #35 rclk = ~rclk;  // Read/serial clock: 70ns period
    end

    // Test Sequence
    initial begin
        // Initialize signals
        wrst_n = 1'b0;
        rrst_n = 1'b0;
        winc = 1'b0;
        wdata = 8'b0;
        gpio_direction = 1'b0;  // Start in receive mode
        gpio_in = 1'b0;

        // Reset
        #50;
        wrst_n = 1'b1;
        rrst_n = 1'b1;

        // Test Case 1: Transmit Data via FIFO and GPIO
        #100;
        gpio_direction = 1'b1; // Transmit mode
      repeat (15) begin
            @(posedge wclk);
            wdata = $random % 256;  // Random 8-bit data
            winc = 1'b1;
            @(posedge wclk);
            winc = 1'b0;
        end

        // Wait for data transmission
        #1000;

//         // Test Case 2: Receive Data Serially
//         gpio_direction = 1'b0; // Switch to receive mode
//         repeat (8) begin
//             @(posedge rclk);
//             gpio_in = $random % 2;  // Random serial bit input
//             $display("Received Bit: %b", gpio_in);
//         end

//         // Wait for full data reception
//         #100;
//         $display("Received Data: %b", pin_status);

        // End Simulation
        #2000;
        $finish;
    end

    // Waveform Dump
    initial begin
        $dumpfile("gpio_fifo_top_serial_tb.vcd");
        $dumpvars(0, gpio_fifo_top_serial_tb);
    end

endmodule
