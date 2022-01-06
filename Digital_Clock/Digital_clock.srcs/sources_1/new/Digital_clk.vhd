----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 12/01/2021 10:16:15 PM
-- Design Name: 
-- Module Name: Digital_clk - Behavioral
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
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.package_state_machine.all;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity Digital_clk is
    Port ( Rx : in STD_LOGIC_VECTOR (7 downto 0);   
            clk : in STD_LOGIC;  --clock operation
       pulse_1s : in STD_LOGIC;  --clock 1 sec (pulse)
           btn0 : in STD_LOGIC; --set time
           btn1 : in STD_LOGIC; --set alarm
           btn2 : in STD_LOGIC; --set timer
           btn3 : in STD_LOGIC; --counter time
           btn4 : in STD_LOGIC; --rst
  Rx_data_ready : in STD_LOGIC; --indicate received input Hour form Rx 
       alarm_out : out STD_LOGIC; --sound alarm
     counter_out : out STD_LOGIC; --sound counter done
       state_out : out clock_state_type;
           led0  : out STD_LOGIC; --set time state on
           led1  : out STD_LOGIC; --set alarm state on
           led2  : out STD_LOGIC; --set timer state on
           led3  : out STD_LOGIC; -- counter time state on       
           H_out : out STD_LOGIC_VECTOR (7 downto 0); --hour(1)(4bit) digit and hour(0) digit (4bit) 
           M_out : out STD_LOGIC_VECTOR (7 downto 0); --minute(1) digit and minute(0) digit 
           S_out : out STD_LOGIC_VECTOR (7 downto 0)); --second(1) digit and second(0) digit 
end Digital_clk;

architecture Behavioral of Digital_clk is

signal H, H_alarm , H_timer , H_T_count: STD_LOGIC_VECTOR (7 downto 0);
signal M, M_alarm , M_timer , M_T_count: STD_LOGIC_VECTOR (7 downto 0);
signal S, S_alarm , S_timer , S_T_count: STD_LOGIC_VECTOR (7 downto 0);
signal alarm_is_set , counter_is_set:  STD_LOGIC := '0'; 
signal Rx_cnt_data :integer range 0 to 5; 

signal H_tens,H_units,M_tens,M_units,S_tens,S_units :STD_LOGIC_VECTOR (3 downto 0); 


--type state_type is (idle,set_time,set_alarm,set_timer,timer,timer_pause,counter_pause,set_time_counter,time_counter,rst);

signal fsm_state: clock_state_type := idle;

begin

state_out <= fsm_state;

fsm_process: process(clk,btn4)   -- where to go from state to state
     begin 
       if (btn4 = '1') then fsm_state <= rst;
   
       elsif (rising_edge(clk)) then 
             case (fsm_state) is 
                  when idle =>  if (btn0 = '1') then fsm_state <= set_time;
                                elsif (btn1 = '1') then fsm_state <= set_alarm;
                                elsif (btn2 = '1') then fsm_state <= set_timer;
                                elsif (btn3 = '1') then fsm_state <= set_time_counter;
                                end if;
                  
              when set_time =>  if (btn0 = '1') then fsm_state <= idle;
                                end if;
              
             when set_alarm =>  if (btn1 = '1') then fsm_state <= idle;
                                end if;                  
              
             when set_timer =>  if (btn2 = '1') then fsm_state <= timer;
                                end if;  
                              
                 when timer =>  if (btn2 = '1') then fsm_state <= timer_pause;
                                end if;     
                                
                 when timer_pause =>  if (btn2 = '1') then fsm_state <= timer;
                                end if;   
                                
      when set_time_counter =>  if (btn3 = '1') then fsm_state <= time_counter;
                                end if;
                                            
            when time_counter =>  if (btn3 = '1') then fsm_state <= counter_pause;
                                end if;
                           
            when counter_pause =>  if (btn3 = '1') then fsm_state <= time_counter;
                                end if;                                
                   
                   when rst =>  if (pulse_1s = '1') then fsm_state <= idle;
                                end if; -- if reset then stay in reset for 1 second
                                        -- and move to idle 
                   
             end case;
        end if;
end process;       
                                
fsm_task: process(clk)   -- what to do in each state
       begin 
          if (rising_edge(clk)) then
           if (fsm_state = rst) then
               led0 <= '1'; led1 <= '1'; led2 <= '1'; led3 <= '1';
               alarm_is_set <= '0';
         elsif (fsm_state = idle) then 
               led0 <= '0'; led1 <= '0'; led2 <= '0'; led3 <= '0';
               if ((pulse_1s = '1')) then
                 if (alarm_is_set = '1' and M_alarm = M and H_alarm = H and S_alarm = S) then --alarm on/off
                 alarm_out <= '1'; 
                 else
                 alarm_out <= '0';
                 end if;
                end if;
         elsif (fsm_state = set_time) then 
                led0 <= '1'; led1 <= '0'; led2 <= '0'; led3 <= '0';
         elsif (fsm_state = set_alarm) then 
                alarm_is_set <= '1';
                led0 <= '0'; led1 <= '1'; led2 <= '0'; led3 <= '0';
                  H_alarm <= (H_tens * "1010") + H_units;  --(convert back to decimal)
                  M_alarm <= (M_tens * "1010") + M_units;  --(convert back to decimal)
                  S_alarm <= (S_tens * "1010") + S_units;  --(convert back to decimal)              
         elsif (fsm_state = set_timer) then  -- set count down time
                  counter_is_set <= '1';
                  led0 <= '0'; led1 <= '0'; led2 <= '1'; led3 <= '0';
                  H_timer <= (H_tens * "1010") + H_units;  --(convert back to decimal)
                  M_timer <= (M_tens * "1010") + M_units;  --(convert back to decimal)
                  S_timer <= (S_tens * "1010") + S_units;  --(convert back to decimal)
         elsif (fsm_state = set_time_counter) then  -- set count up state
                led0 <= '0'; led1 <= '0'; led2 <= '0'; led3 <= '1';
                H_T_count <= (others => '0');
                M_T_count <= (others => '0');
                S_T_count <= (others => '0');
         elsif (fsm_state = timer) then      -- count down
               if ((pulse_1s = '1')) then
                  if (counter_is_set = '1'and M_timer=("00000000")and H_timer=("00000000")and S_timer=("00000000"))then
                  counter_out <= '1';
                  counter_is_set <= '0';
                  else
                  counter_out <= '0';
                  end if;
              if (M_timer=("00000000")and H_timer=("00000000")and S_timer=("00000000"))then
                 S_timer <= (others=>'0'); M_timer <= (others=>'0'); H_timer <= (others=>'0');
               elsif(S_timer = 0) then --// second > 59 then minute increases
                 S_timer <= std_logic_vector(to_unsigned(59,S_timer'length));   --??59                     
                 if(M_timer = 0) then -- //minute > 59 then hour increases
                   M_timer <= std_logic_vector(to_unsigned(59,M_timer'length));   --??59  
                   if (H_timer /= 0) then -- hour > 24 then set hour to 0
                     H_timer <= H_timer - 1;
                     end if;
                    else
                    M_timer<= M_timer - 1;
                    end if;
                    else
                    S_timer <= S_timer - 1;
                  end if;
                 else 
                 
               end if;
           elsif (fsm_state = time_counter) then  --count up (stop watch)
               if ((pulse_1s = '1')) then
                 if(S_T_count = 59) then --// second > 59 then minute increases
                 S_T_count  <= std_logic_vector(to_unsigned(0,S_T_count'length));                 
                 if(M_T_count = 59) then -- //minute > 59 then hour increases
                   M_T_count  <= std_logic_vector(to_unsigned(0,M_T_count'length));   
                   if (H_T_count = 23) then -- hour > 23 then set hour to 0
                     H_T_count  <= (others => '0');     
                  else
                   H_T_count <= H_T_count + 1;
                   end if;
                 else 
                    M_T_count <= M_T_count + 1;     
                 end if;
               else 
                S_T_count <= S_T_count + 1;
               end if; 
              end if;
           -- both pause state dosen't need task process.                                       
            end if;
          end if;
       end process;   
       
time_output: process(clk)
          begin
          if (rising_edge(clk)) then
           if ((fsm_state = idle) or (fsm_state = set_time)) then  
             S_out <= S;
             M_out <= M;
             H_out <= H;
           elsif  (fsm_state = set_alarm) then  
             S_out <= S_alarm;
             M_out <= M_alarm;
             H_out <= H_alarm;
           elsif ((fsm_state = set_timer) or (fsm_state = timer) or (fsm_state = timer_pause)) then
             S_out <= S_timer;
             M_out <= M_timer;
             H_out <= H_timer;
           elsif ((fsm_state = set_time_counter) or (fsm_state = time_counter) or (fsm_state = counter_pause)) then
             S_out <= S_T_count;
             M_out <= M_T_count;
             H_out <= H_T_count;
           end if;     
          end if; 
       end process; 
       
real_time: process (clk)   --not depending on state, if in state then must include it in every state
           begin
          if (rising_edge(clk)) then
            if ((pulse_1s = '1') and (fsm_state /= set_time)) then --need to make sure we are not seting time 
               if(S = 59) then --// second > 59 then minute increases
                 S <= std_logic_vector(to_unsigned(0,S_T_count'length));                 
                 if(M = 59) then -- //minute > 59 then hour increases
                   M <= std_logic_vector(to_unsigned(0,M_T_count'length));   
                   if (H = 23) then -- hour > 23 then set hour to 0
                     H <= (others => '0');     
                  else
                   H <= H + 1;
                   end if;
                 else 
                    M <= M + 1;     
                 end if;
               else 
                S <= S + 1;
               end if; 
            elsif (fsm_state = set_time) then  --read new setup time data 
                  H <= (H_tens * "1010") + H_units;  --(convert back to decimal)
                  M <= (M_tens * "1010") + M_units;  --(convert back to decimal)
                  S <= (S_tens * "1010") + S_units;  --(convert back to decimal)
            end if;
         end if; 
    end process; 
            
            
 data_counter_for_inputs: process (clk) --increment data count
  begin 
     if (rising_edge(clk)) then
       if (fsm_state = set_time or fsm_state = set_timer or fsm_state = set_alarm) then
         if ((Rx_data_ready = '1') and (Rx_cnt_data /= 5)) then
          Rx_cnt_data <= Rx_cnt_data + 1;   
         elsif ((Rx_data_ready = '1') and (Rx_cnt_data = 5)) then
          Rx_cnt_data <= 0;
         end if;
       end if;
     end if;
  end process;   
  
 time_input: process(clk)     -- time input from Rx - UART
    begin
     if (rising_edge(clk)) then
       if (fsm_state = set_time or fsm_state = set_timer or fsm_state = set_alarm) then
         if (Rx_cnt_data = 0 and Rx_data_ready = '1') then 
           H_tens <= Rx(3 downto 0);
         elsif (Rx_cnt_data = 1 and Rx_data_ready = '1') then 
           H_units <= Rx(3 downto 0);
         elsif (Rx_cnt_data = 2 and Rx_data_ready = '1') then
           M_tens <= Rx(3 downto 0);
         elsif (Rx_cnt_data = 3 and Rx_data_ready = '1') then
           M_units <= Rx(3 downto 0);
         elsif (Rx_cnt_data = 4 and Rx_data_ready = '1') then
           S_tens <= Rx(3 downto 0);
         elsif (Rx_cnt_data = 5 and Rx_data_ready = '1') then
           S_units <= Rx(3 downto 0); 
         end if;
       end if;
     end if;
  end process;
            
end Behavioral;
