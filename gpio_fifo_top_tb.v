// Code your testbench here
//Name: Mishra ji
// `timescale 1ns / 1ps


`timescale 1ns / 1ps

module gpio_fifo_top_tb;

    // Parameters
    parameter DSIZE = 8;
    parameter ASIZE = 4;

    // DUT Signals
    reg wclk, wrst_n, winc;
    reg [DSIZE-1:0] wdata;

    reg rclk, rrst_n;
    wire [DSIZE-1:0] gpio_out, pin_status;
    wire [DSIZE-1:0] gpio_in;

    // Instantiate DUT
    gpio_fifo_top dut (
        .wclk(wclk),
        .wrst_n(wrst_n),
        .winc(winc),
        .wdata(wdata),
        .rclk(rclk),
        .rrst_n(rrst_n),
        .gpio_in(8'b10101010),  // Test GPIO input
        .gpio_out(gpio_out),
        .pin_status(pin_status)
    );

    // Clock Generation
    initial begin
        wclk = 1'b0;
        forever #10 wclk = ~wclk;  // Write clock: 20ns period
    end

    initial begin
        rclk = 1'b0;
        forever #35 rclk = ~rclk;  // Read clock: 70ns period
    end

    // Test Sequence
    initial begin
        // Initialize signals
        winc = 1'b0;
        wdata = 8'b0;
        wrst_n = 1'b0;
        rrst_n = 1'b0;

        // Release resets
        #50 wrst_n = 1'b1;
        #80 rrst_n = 1'b1;

        // Write data to tx_fifo
        #100;
        repeat (16) begin
            @(posedge wclk);
            wdata = $random % 256;  // Random 8-bit data
            winc = 1'b1;
            @(posedge wclk);
            winc = 1'b0;
        end

        // Observe GPIO outputs
        #1000;
        $display("GPIO Output: %b", gpio_out);
        $display("GPIO Pin Status: %b", pin_status);

        // Finish simulation
        #2000;
        $finish;
    end

    // Waveform Dump
    initial begin
        $dumpfile("gpio_fifo_top_tb.vcd");
        $dumpvars(0, gpio_fifo_top_tb);
    end

endmodule










// module gpio_block_tb;

//     // Testbench Signals
//     reg clk;
//     reg rst_n;
//     reg [7:0] data_in;
//     reg [7:0] direction;
//     reg [7:0] gpio_in;
//     wire [7:0] gpio_out;
//     wire [7:0] pin_status;

//     // Instantiate the GPIO Block
//     gpio_block uut (
//         .clk(clk),
//         .rst_n(rst_n),
//         .data_in(data_in),
//         .direction(direction),
//         .gpio_out(gpio_out),
//         .gpio_in(gpio_in),
//         .pin_status(pin_status)
//     );

//     // Clock generation
//     initial clk = 0;
//     always #5 clk = ~clk; // 10ns clock period

//     // Testbench Logic
//     initial begin
//         // Initialize inputs
//         rst_n = 0;         // Assert reset
//         data_in = 8'b0;
//         direction = 8'b0;
//         gpio_in = 8'b0;

//         // Apply reset
//         #10;
//         rst_n = 1;         // Deassert reset

//         // Test Case 1: All pins as outputs, write data to pins
//         #10;
//         direction = 8'b11111111; // All pins as outputs
//         data_in = 8'b10101010;  // Write alternating 1s and 0s
//         #10;

//         // Test Case 2: All pins as inputs, observe pin status
//         #10;
//         direction = 8'b00000000; // All pins as inputs
//         gpio_in = 8'b11001100;  // Simulate input values
//         #10;

//         // Test Case 3: Mixed direction, some inputs, some outputs
//         #10;
//         direction = 8'b01010101; // Mixed direction: alternating input/output
//         data_in = 8'b11110000;  // Data for output pins
//         gpio_in = 8'b00110011;  // Simulate input values
//         #20;

//         // Test Case 4: Change data while pins are outputs
//         #10;
//         data_in = 8'b00001111;  // Update output data
//         #20;

//         // End simulation
//         $stop;
//     end

//     // Dump waveform data
//     initial begin
//         $dumpfile("gpio_block_tb.vcd"); // Specify dumpfile name
//         $dumpvars(0, gpio_block_tb);    // Dump all variables in the testbench
//     end

//     // Monitor signals
//     initial begin
//         $monitor("Time=%0t | rst_n=%b | direction=%b | data_in=%b | gpio_in=%b | gpio_out=%b | pin_status=%b",
//                  $time, rst_n, direction, data_in, gpio_in, gpio_out, pin_status);
//     end

// endmodule
