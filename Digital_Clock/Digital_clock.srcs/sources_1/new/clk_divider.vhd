----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 12/03/2021 12:06:34 AM
-- Design Name: 
-- Module Name: clk_divider - Behavioral
-- Project Name: 
-- Target Devices: 
-- Tool Versions: 
-- Description: 
-- 
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
USE IEEE.STD_LOGIC_UNSIGNED.ALL;
use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity clk_divider is
 generic(
    clk_freq    : integer := 125_000_000);  --system clock frequency in Hz       
port (
   clk: in std_logic;
   clk_1s: out std_logic
  );
end clk_divider;
architecture Behavioral of clk_divider is

signal counter: integer range 0 to clk_freq := 0;
signal clk_1s_temp : STD_LOGIC := '0';

begin

 process(clk)
 begin
  if(rising_edge(clk)) then
   counter <= counter + 1;
   if(counter>=clk_freq - 1) then -- for running on FPGA -- comment when running simulation
   clk_1s_temp <= '1';
    counter <= 0;
    else
   clk_1s_temp <= '0';
   end if;
  end if;
 end process;
 clk_1s <= clk_1s_temp; -- when counter
end Behavioral;

