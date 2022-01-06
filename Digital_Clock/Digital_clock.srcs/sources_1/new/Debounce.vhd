----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 12/02/2021 11:35:59 PM
-- Design Name: 
-- Module Name: Debounce - Behavioral
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

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity Debounce is
   generic(
    clk_freq    : integer := 125_000_000;  --system clock frequency in Hz
    stable_time : integer := 10);         --time button must remain stable in ms
  port(
    clk     : in  STD_LOGIC;  --input clock
    btn  : in  STD_LOGIC;  --input signal to be debounced
    result  : out STD_LOGIC := '0'); --debounced signal
end debounce;

architecture logic of debounce is

  signal glitch_free : STD_LOGIC := '0';
  signal ff_glitch_free   :  STD_LOGIC_VECTOR(1 downto 0) := "00"; --flipflop

  SIGNAL flipflops   : STD_LOGIC_VECTOR(1 downto 0) := "00"; --input flip flops for checking if input was change
  SIGNAL counter_set : STD_LOGIC;                    --sync reset to zero
begin

  counter_set <= flipflops(0) xor flipflops(1);  --determine when to start/reset counter by checking if input was change
                                                 --filter the glitches after btn pressed 
debouncing_rising_edge_clk:  process(clk)
    variable count :  integer range 0 to clk_freq*stable_time/1000;  --counter for timing 100MHz*10ms/1000
    begin                                    
      if(rising_edge(clk)) then                      --rising clock edge
      flipflops(0) <= btn;                           --store button value in 1st flipflop
      flipflops(1) <= flipflops(0);                  --store 1st flipflop value in 2nd flipflop
      if (counter_set = '1') then                     --reset counter because input is changing
        count := 0;                                    --clear the counter
      elsif (count < clk_freq*stable_time/1000) then  --stable input time is not yet met
        count := count + 1;                            --increment counter
      else                                           --stable input time is met
        glitch_free <= flipflops(1);                        --output the stable value
      end if;    
    end if;
  end process;
  
stable_pulse: process (clk) 
     begin                                    
      if(rising_edge(clk)) then   
        if ( (ff_glitch_free(0) and (not ff_glitch_free(1))) = '1' )  then 
        result <= '1';
        else 
        result <= '0';
        end if;
        ff_glitch_free(0) <= glitch_free;
        ff_glitch_free(1) <= ff_glitch_free(0);
      end if;
     end process;   
  
end logic;