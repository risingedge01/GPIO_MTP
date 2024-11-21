//GPIO with Async FIFO tb
//Mishra ji

`timescale 1ns / 1ps

module gpio_fifo_tx_rx_top_serial_tb;

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

    // RX FIFO Signals
    reg rx_fifo_rinc;              // Read enable for RX FIFO
    wire rx_fifo_rempty;           // RX FIFO empty flag
    wire [DSIZE-1:0] rx_fifo_rdata; // Data read from RX FIFO

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
        .pin_status(pin_status),
        .rx_fifo_rdata(rx_fifo_rdata),
        .rx_fifo_rinc(rx_fifo_rinc),
        .rx_fifo_rempty(rx_fifo_rempty)
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
        rx_fifo_rinc = 1'b0;

        // Reset
        #50;
        wrst_n = 1'b1;
        rrst_n = 1'b1;

        // --- Test Case 1: Transmit Data ---
        $display("\n=== Transmit Test Case ===");
        gpio_direction = 1'b1; // Set to transmit mode

        // Write data to TX FIFO and observe serial_out
        repeat (15) begin
            @(posedge wclk);
            wdata = $random % 256; // Random 8-bit data
            winc = 1'b1;           // Enable TX FIFO write
            @(posedge wclk);
            winc = 1'b0;           // Disable TX FIFO write
            $display("Data written to TX FIFO: %h", wdata);
        end

        // Wait for transmission to complete
        #1000;
        $display("Transmit test completed. Verify 'serial_out' in waveform.");

        // --- Test Case 2: Receive Data Serially ---
        $display("\n=== Receive Test Case ===");
        gpio_direction = 1'b0; // Switch to receive mode

        // Provide serial input to gpio_in
      repeat (18) begin
            @(posedge rclk);
            gpio_in = $random % 2; // Random serial bit input
            $display("Serial Bit Input: %b", gpio_in);
        end

        // Wait for winc_interrupt (RX FIFO write trigger)
        @(posedge dut.gpio_inst.winc_interrupt);

        // Read data from RX FIFO
        while (!rx_fifo_rempty) begin
            @(posedge rclk);
            rx_fifo_rinc = 1'b1; // Enable RX FIFO read
            @(posedge rclk);
            rx_fifo_rinc = 1'b0; // Disable RX FIFO read
            $display("Data read from RX FIFO: %h", rx_fifo_rdata);
        end

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
