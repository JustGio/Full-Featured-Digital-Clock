----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 09.12.2021 04:08:38
-- Design Name: 
-- Module Name: UART_rx - Behavioral
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
use ieee.numeric_std.all;
-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity UART_rx is
  Port (
        CLK 	: in  STD_LOGIC;
        Rx_in   : in  STD_LOGIC;
        
      Data_out  : out STD_LOGIC_VECTOR(7 downto 0);
       Data_rdy : out STD_LOGIC
   );
   
   
   
end UART_rx;

architecture Behavioral of UART_rx is

type state_machine_uart is (IDLE,START_BIT,DATA,END_BIT);

-- CLK_PER_BIT = (Frequency of CLK)/(Frequency of UART)
-- CLK_PER_BIT = 125M / 9600 = 13020

constant CLK_PER_BIT : integer := 13020;

signal state_machine : state_machine_uart := IDLE;

signal CLK_counter : integer range 0 to CLK_PER_BIT-1 := 0;
signal DATA_cnt    : integer range 0 to 7 := 0;

signal Data_out_reg : STD_LOGIC_VECTOR (7 downto 0);

begin

UART_rx_process : process(CLK)  --where to go from state to state if condition met 
begin
  if(rising_edge(CLK)) then
    case state_machine is
    
      when IDLE => if(Rx_in = '0') then-- Start detected
                     state_machine <= START_BIT;
                   end if;  
  
 when START_BIT => if(CLK_counter = CLK_PER_BIT/2) then -- Half of UART baud rate to find the middle
                     state_machine <= DATA;
                   end if; 
                    
      when DATA => if(DATA_cnt = 7 and CLK_counter = CLK_PER_BIT - 1) then -- Data over
                     state_machine <= END_BIT;
                   end if;  

   when END_BIT => if(CLK_counter = CLK_PER_BIT - 1) then -- Go back to idle
                     state_machine <= IDLE;
                   end if;  
    end case; 
  end if;
end process;

Counters_process : process(CLK)  --what to do in each state
begin
if(rising_edge(CLK)) then
  if(state_machine = START_BIT) then     -- reset at mid bit
    if(CLK_counter = CLK_PER_BIT/2) then
      CLK_counter <= 0;
      DATA_cnt    <= 0;
    else 
      CLK_counter <= CLK_counter + 1;
    end if;
  elsif(state_machine /= IDLE) then -- All other states other than IDLE - in idle nothing happened
    if(CLK_counter = CLK_PER_BIT - 1) then      
      DATA_cnt    <= DATA_cnt + 1;
      CLK_counter <= 0;
    else
      CLK_counter <= CLK_counter + 1;
    end if;
  end if; 
end if; 
end process;

Data_out_process : process(CLK)    --every signal has it's own process 
begin
  if(rising_edge(CLK)) then
    if(state_machine = DATA and CLK_counter = CLK_PER_BIT - 1) then
      Data_out_reg(7) <= Rx_in;     --Sample the Rx_in      
      Data_out_reg(6 downto 0) <= Data_out_reg(7 downto 1); -- Right shift -(Rx_in,z,z,z,z,z,z,z,z) untill it's full
    end if;
  end if;
end process;

Data_out <= Data_out_reg;   --(230900) = (23:09:00)

Data_rdy_process : process(CLK)      -- flag process 
begin
  if(rising_edge(CLK)) then
    if(state_machine = DATA and DATA_cnt = 7 and CLK_counter = CLK_PER_BIT - 1) then
      Data_rdy <= '1';
    else 
      Data_rdy <= '0';
    end if;
  end if;
end process;

end Behavioral;
