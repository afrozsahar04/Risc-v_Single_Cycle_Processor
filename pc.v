`timescale 1ns / 1ps


module program_counter(
            input  clk,
            input  reset,
            input  [31:0] pc_in,
            output reg [31:0] pc_out
            );
            
                  always@(posedge clk)
                  if(reset)
                    pc_out<=32'b0;         //Reset PC to 0 on reset
                  else                      
                    pc_out<=pc_in;         // Otherwise, update PC  
 endmodule
            
