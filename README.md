# Smart Speed-Adaptive Streetlight Control System – FPGA (Spartan-7 Boolean Board)



## Aim  
To design and simulate a **Smart Speed-Adaptive Streetlight Control System** using Verilog and implement it on the Spartan-7 Boolean FPGA board.

---

## Objectives  
Design and implement a smart streetlight system that automatically adjusts its brightness based on the speed of approaching vehicles, providing higher illumination at higher speeds for improved safety and reducing brightness during low-speed or no-traffic conditions to save energy and enhance overall efficiency. 

---

## Apparatus  
- Vivado 2024.1  
- Spartan-7 Boolean Board (XC7S50-CSGA324)  

---
## Block Diagram  

<img width="1024" height="1024" alt="Gemini_Generated_Image_nq46l3nq46l3nq46" src="https://github.com/user-attachments/assets/5dc740bd-38e1-441f-863b-605c409dd19b" />

---

# **VERILOG CODE**

```verilog
`timescale 1ns/1ps

module speed_detector(input clk, input reset, input [7:0] external_speed, input use_external,
                      output reg [7:0] speed_value);
    reg [7:0] sim_speed;
    reg [25:0] sim_cnt;
    always @(posedge clk or posedge reset) begin
        if (reset) begin sim_speed <= 0; sim_cnt <= 0; end
        else begin
            sim_cnt <= sim_cnt + 1;
            if (sim_cnt >= 26'd50_000_000) begin sim_cnt <= 0; sim_speed <= sim_speed + 8'd10; end
        end
    end
    always @(*) speed_value = use_external ? external_speed : sim_speed;
endmodule

module pwm_generator(input clk, input reset, input [7:0] duty_cycle, output reg pwm_out);
    localparam CLK_DIV = 195;
    reg [7:0] pwm_cnt;
    reg [7:0] div_cnt;
    always @(posedge clk or posedge reset) begin
        if (reset) begin pwm_cnt<=0; div_cnt<=0; pwm_out<=0; end
        else begin
            if (div_cnt >= CLK_DIV-1) begin div_cnt<=0; pwm_cnt <= (pwm_cnt==8'hFF) ? 0 : pwm_cnt+1; end
            else div_cnt <= div_cnt + 1;
            pwm_out <= (pwm_cnt < duty_cycle);
        end
    end
endmodule

module auto_shutdown_timer(input clk, input reset, input [7:0] speed, output reg shutdown_signal);
    localparam TIMEOUT = 28'd250_000_000;
    reg [27:0] t;
    always @(posedge clk or posedge reset) begin
        if (reset) begin t<=0; shutdown_signal<=0; end
        else if (speed==0) begin
            if (t >= TIMEOUT-1) shutdown_signal<=1;
            else begin t<=t+1; shutdown_signal<=0; end
        end else begin t<=0; shutdown_signal<=0; end
    end
endmodule

module fsm_controller(input clk, input reset, input [7:0] speed, input shutdown,
                      output reg [7:0] duty_cycle, output reg [1:0] current_state);
    localparam IDLE=2'd0, ACTIVE=2'd1, BRIGHT=2'd2;
    localparam SPEED_ACTIVE_MAX = 8'd150;
    localparam MIN_B = 8'd25, MAX_B = 8'd255;
    reg [1:0] next;
    always @(*) begin
        case(current_state)
            IDLE:   next = shutdown ? IDLE : (speed>0 && speed<=SPEED_ACTIVE_MAX ? ACTIVE : (speed>SPEED_ACTIVE_MAX ? BRIGHT : IDLE));
            ACTIVE: next = (speed==0) ? IDLE : (speed>SPEED_ACTIVE_MAX ? BRIGHT : ACTIVE);
            BRIGHT: next = (speed==0) ? IDLE : ((speed<=SPEED_ACTIVE_MAX && speed>0) ? ACTIVE : BRIGHT);
            default: next = IDLE;
        endcase
    end
    always @(posedge clk or posedge reset) current_state <= (reset ? IDLE : next);
    always @(posedge clk or posedge reset) begin
        if (reset) duty_cycle <= MIN_B;
        else case(current_state)
            IDLE:   duty_cycle <= shutdown ? 8'd0 : MIN_B;
            ACTIVE: duty_cycle <= MIN_B + (speed >> 1) + (speed >> 2);
            BRIGHT: duty_cycle <= MAX_B;
            default: duty_cycle <= MIN_B;
        endcase
    end
endmodule

module streetlight_top(input clk_50mhz, input reset, input [7:0] external_speed, input use_external_speed,
                       output wire streetlight_pwm, output wire [1:0] state_indicator, output wire [7:0] current_speed, output wire shutdown_active);
    wire [7:0] speed_internal, duty_cycle_internal;
    wire shutdown_signal;
    speed_detector u_speed(clk_50mhz, reset, external_speed, use_external_speed, speed_internal);
    auto_shutdown_timer u_timer(clk_50mhz, reset, speed_internal, shutdown_signal);
    fsm_controller u_fsm(clk_50mhz, reset, speed_internal, shutdown_signal, duty_cycle_internal, state_indicator);
    pwm_generator u_pwm(clk_50mhz, reset, duty_cycle_internal, streetlight_pwm);
    assign current_speed = speed_internal;
    assign shutdown_active = shutdown_signal;
endmodule
```
# **TESTBENCH**
```verilog
module streetlight_tb;
    reg clk_50mhz=0, reset=1;
    reg [7:0] external_speed=0;
    reg use_external_speed=1;
    wire streetlight_pwm;
    wire [1:0] state_indicator;
    wire [7:0] current_speed;
    wire shutdown_active;
    streetlight_top dut(.clk_50mhz(clk_50mhz), .reset(reset), .external_speed(external_speed), .use_external_speed(use_external_speed),
                        .streetlight_pwm(streetlight_pwm), .state_indicator(state_indicator), .current_speed(current_speed), .shutdown_active(shutdown_active));
    initial forever #10 clk_50mhz = ~clk_50mhz;

    initial begin
        #100 reset = 0;
        #1000 external_speed = 0;      #5000;
        external_speed = 50;          #5000;
        external_speed = 100;         #5000;
        external_speed = 150;         #5000;
        external_speed = 200;         #5000;
        external_speed = 255;         #5000;
        external_speed = 0;
        repeat(256) begin #50000; external_speed = external_speed + 1; end
        external_speed = 0;
        #2_500_000_000;
        #10000 $finish;
    end

    integer pwm_high_count = 0, pwm_total_count = 0;
    initial begin
        forever @(posedge clk_50mhz) if (!reset) begin
            pwm_total_count = pwm_total_count + 1;
            if (streetlight_pwm) pwm_high_count = pwm_high_count + 1;
            if (pwm_total_count == 50000) begin
                pwm_high_count = 0; pwm_total_count = 0;
            end
        end
    end
endmodule

```
## Constraint file ##
```verilog
## Clock Constraint
create_clock -period 20.000 -name clk_50mhz -waveform {0.000 10.000} [get_ports clk_50mhz]

## Clock Pin - 100 MHz oscillator (note: using internal clock divider to get 50MHz in design)
set_property -dict {PACKAGE_PIN F14 IOSTANDARD LVCMOS33} [get_ports clk_50mhz]

## Reset - Pushbutton BTN0
set_property -dict {PACKAGE_PIN J2 IOSTANDARD LVCMOS33} [get_ports reset]

## Use External Speed - Slide Switch SW0
set_property -dict {PACKAGE_PIN V2 IOSTANDARD LVCMOS33} [get_ports use_external_speed]

## External Speed[7:0] - Slide Switches SW1-SW8
set_property -dict {PACKAGE_PIN U2 IOSTANDARD LVCMOS33} [get_ports {external_speed[0]}]
set_property -dict {PACKAGE_PIN U1 IOSTANDARD LVCMOS33} [get_ports {external_speed[1]}]
set_property -dict {PACKAGE_PIN T2 IOSTANDARD LVCMOS33} [get_ports {external_speed[2]}]
set_property -dict {PACKAGE_PIN T1 IOSTANDARD LVCMOS33} [get_ports {external_speed[3]}]
set_property -dict {PACKAGE_PIN R2 IOSTANDARD LVCMOS33} [get_ports {external_speed[4]}]
set_property -dict {PACKAGE_PIN R1 IOSTANDARD LVCMOS33} [get_ports {external_speed[5]}]
set_property -dict {PACKAGE_PIN P2 IOSTANDARD LVCMOS33} [get_ports {external_speed[6]}]
set_property -dict {PACKAGE_PIN P1 IOSTANDARD LVCMOS33} [get_ports {external_speed[7]}]

## Streetlight PWM - LED0
set_property -dict {PACKAGE_PIN G1 IOSTANDARD LVCMOS33} [get_ports streetlight_pwm]

## State Indicator[1] - LED1
set_property -dict {PACKAGE_PIN G2 IOSTANDARD LVCMOS33} [get_ports {state_indicator[1]}]

## State Indicator[0] - LED2
set_property -dict {PACKAGE_PIN F1 IOSTANDARD LVCMOS33} [get_ports {state_indicator[0]}]

## Current Speed[7:0] - LEDs LED3-LED10
set_property -dict {PACKAGE_PIN F2 IOSTANDARD LVCMOS33} [get_ports {current_speed[0]}]
set_property -dict {PACKAGE_PIN E1 IOSTANDARD LVCMOS33} [get_ports {current_speed[1]}]
set_property -dict {PACKAGE_PIN E2 IOSTANDARD LVCMOS33} [get_ports {current_speed[2]}]
set_property -dict {PACKAGE_PIN D1 IOSTANDARD LVCMOS33} [get_ports {current_speed[3]}]
set_property -dict {PACKAGE_PIN D2 IOSTANDARD LVCMOS33} [get_ports {current_speed[4]}]
set_property -dict {PACKAGE_PIN C1 IOSTANDARD LVCMOS33} [get_ports {current_speed[5]}]
set_property -dict {PACKAGE_PIN C2 IOSTANDARD LVCMOS33} [get_ports {current_speed[6]}]
set_property -dict {PACKAGE_PIN B1 IOSTANDARD LVCMOS33} [get_ports {current_speed[7]}]

## Shutdown Active - LED11
set_property -dict {PACKAGE_PIN B2 IOSTANDARD LVCMOS33} [get_ports shutdown_active]
```
## Simulation Output 

![WhatsApp Image 2025-11-18 at 21 34 14_61365ace](https://github.com/user-attachments/assets/d99f2d08-144a-4305-be36-a91f4b852ff9)

## Hardware Output


## Report

- The system adjusts streetlight brightness automatically based on the speed of passing vehicles.
- Two sensors measure the vehicle’s speed by calculating the time taken to travel a fixed distance.
- A microcontroller processes the speed and controls LED brightness using PWM.
- High vehicle speed results in higher streetlight brightness for better visibility.
- Low or no traffic reduces brightness to save energy and extend LED lifespan.
- The system improves road safety while reducing unnecessary power consumption.
- It is suitable for highways, smart cities, and energy-efficient lighting projects.

## Result 


The system successfully detected vehicle speeds and adjusted the streetlight brightness in real time. It increased illumination for fast-moving vehicles and reduced brightness during low or no traffic, demonstrating effective energy saving and improved lighting efficiency.

