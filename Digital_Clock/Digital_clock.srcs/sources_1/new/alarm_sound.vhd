----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 12/08/2021 11:49:35 PM
-- Design Name: 
-- Module Name: alarm_sound - Behavioral
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
use IEEE.NUMERIC_STD.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity alarm_sound is     -- generate sound for 3 seconds
    Port ( clk : in STD_LOGIC;
           sound_ena : in STD_LOGIC;
         counter_out : in STD_LOGIC;
           sound_out : out STD_LOGIC);
end alarm_sound;

architecture Behavioral of alarm_sound is

signal counter: integer range 0 to 25000 := 0;
signal length_sound: integer range 0 to 375000000 :=0;  --(3s)

begin
Alarm_sound: process(clk)
  begin
    if (rising_edge(clk)) then
      if (sound_ena = '1' or counter_out = '1' or length_sound /= 0) then  
        length_sound <= length_sound + 1;
        counter <= counter + 1;
        if (counter <= 6250) then  
        sound_out <= '1';
        elsif (counter <25000 and counter>6250) then
        sound_out <= '0';
        end if;
       end if;
      end if;  
end process;
end Behavioral;

