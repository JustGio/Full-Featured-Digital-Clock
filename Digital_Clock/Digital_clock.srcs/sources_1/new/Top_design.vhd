----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 12/02/2021 11:04:37 PM
-- Design Name: 
-- Module Name: Top_design - Behavioral
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
use work.package_state_machine.all;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity Top_design is
  port(    Rx : in STD_LOGIC;  --bit input 
           clk : in STD_LOGIC;  --clock operation
           btn0_T : in STD_LOGIC; --set time
           btn1_T : in STD_LOGIC; --set alarm
           btn2_T : in STD_LOGIC; --set timer
           btn3_T : in STD_LOGIC; --counter time
           btn4_T : in STD_LOGIC; --rst   
           sound_out : out STD_LOGIC; --sound out
           led0  : out STD_LOGIC; --set time state on
           led1  : out STD_LOGIC; --set alarm state on
           led2  : out STD_LOGIC; --set timer state on
           led3  : out STD_LOGIC;-- counter time state on 

           CS      : out STD_LOGIC; -- chipselect
          SDIN    : out STD_LOGIC;  -- serial data in
          SCLK    : out STD_LOGIC; -- serial clk
          DC      : out STD_LOGIC; -- voltage
          RES     : out STD_LOGIC; -- reset
          VBAT    : out STD_LOGIC; -- voltage
          VDD     : out STD_LOGIC  -- voltage     
           );
           
end Top_design;

architecture Behavioral of Top_design is

component clk_divider
port (
 clk: in std_logic;
 clk_1s: out std_logic
     );
end component;

component alarm_sound is
    Port ( clk : in STD_LOGIC;
           sound_ena : in STD_LOGIC;
         counter_out : in STD_LOGIC;
           sound_out : out STD_LOGIC);
end component;

component Debounce is
   generic(
    clk_freq    : integer := 125_000_000;  --system clock frequency in Hz
    stable_time : integer := 10);         --time button must remain stable in ms
  PORT(
    clk     : in  STD_LOGIC;  --input clock
    btn  : in  STD_LOGIC;  --input signal to be debounced
    result  : out STD_LOGIC := '0'); --debounced signal
end component debounce;

component Digital_clk is
    Port ( Rx : in STD_LOGIC_VECTOR (7 downto 0);
            clk : in STD_LOGIC;  --clock operation
       Pulse_1s : in STD_LOGIC;  --clock 1 sec (pulse)
           btn0 : in STD_LOGIC; --set time
           btn1 : in STD_LOGIC; --set alarm
           btn2 : in STD_LOGIC; --set timer
           btn3 : in STD_LOGIC; --counter time
           btn4 : in STD_LOGIC; --rst
      Rx_data_ready: in STD_LOGIC; --indicate received input form Rx 
       alarm_out : out STD_LOGIC; --sound alarm
       state_out : out clock_state_type;
     counter_out : out STD_LOGIC; --sound counter done
           led0  : out STD_LOGIC; --set time state on
           led1  : out STD_LOGIC; --set alarm state on
           led2  : out STD_LOGIC; --set timer state on
           led3  : out STD_LOGIC; -- counter time state on       
           H_out : out STD_LOGIC_VECTOR (7 downto 0); --hour(1)(4bit) digit and hour(0) digit (4bit) 
           M_out : out STD_LOGIC_VECTOR (7 downto 0); --minute(1) digit and minute(0) digit 
           S_out : out STD_LOGIC_VECTOR (7 downto 0)); --second(1) digit and second(0) digit 
end component;

component binary_bcd is
    generic(N: positive := 8);
    port(
        clk, reset  : in std_logic;
        binary_in   : in std_logic_vector(N-1 downto 0);
        bcd0, bcd1  : out std_logic_vector(3 downto 0)
    );
end component ;

component PmodOLEDCtrl is
    Port (  CLK 	: in  STD_LOGIC;
		
            CS  	: out STD_LOGIC;
            SDIN	: out STD_LOGIC;
            SCLK	: out STD_LOGIC;
            DC		: out STD_LOGIC;
            RES	: out STD_LOGIC;
            VBAT	: out STD_LOGIC;
            VDD	: out STD_LOGIC;
            state_in : in clock_state_type;
            pulse_1sec : in STD_LOGIC;
            H_tens,H_units : in STD_LOGIC_VECTOR(3 downto 0);
            M_tens,M_units : in STD_LOGIC_VECTOR(3 downto 0);
            S_tens,S_units : in STD_LOGIC_VECTOR(3 downto 0)
			  );--Finish flag for example block
end component;

component UART_rx is
  Port (
        CLK 	: in  STD_LOGIC;
        Rx_in   : in  STD_LOGIC;
        
      Data_out  : out STD_LOGIC_VECTOR(7 downto 0);
       Data_rdy : out STD_LOGIC
   );
end component;

  signal glitch_free : STD_LOGIC := '0';
  signal ff_glitch_free   :  STD_LOGIC_VECTOR(1 downto 0) := "00"; --flipflop

  SIGNAL flipflops   : STD_LOGIC_VECTOR(1 downto 0) := "00"; --input flip flops for checking if input was change
  SIGNAL counter_set : STD_LOGIC;                    --sync reset to zero
  
  signal de_Btn0, de_Btn1, de_Btn2, de_Btn3, de_Btn4 : STD_LOGIC;
  
 signal pulse_1s : STD_LOGIC;
 
 signal Data_out : STD_LOGIC_VECTOR (7 downto 0);  
 signal Data_rdy : STD_LOGIC; --flag for data ready  
 signal state_out : clock_state_type;
 
 signal alarm_out : STD_LOGIC; 
 signal counter_out : STD_LOGIC;
 
 signal Rx_data_ready :  STD_LOGIC; --indicate received input Hour form Rx 
 
 signal Tx : STD_LOGIC_VECTOR (7 downto 0);

--Binary outputs of digital clock module
 signal H_out : STD_LOGIC_VECTOR (7 downto 0); --hour(1)(4bit) digit and hour(0) digit (4bit) 
 signal M_out : STD_LOGIC_VECTOR (7 downto 0); --minute(1) digit and minute(0) digit 
 signal S_out : STD_LOGIC_VECTOR (7 downto 0); --seccond(1) digit and second(1) digit

--Tens and units digit values of Hours, minutes and seconds 
 signal H_out_units,H_out_tens : STD_LOGIC_VECTOR (3 downto 0);
 signal M_out_units,M_out_tens : STD_LOGIC_VECTOR (3 downto 0);
 signal S_out_units,S_out_tens : STD_LOGIC_VECTOR (3 downto 0);
 
begin

UUT_dig_clk: Digital_clk port map (Rx => Data_out,
                                   clk=>clk,
                                   pulse_1s=>pulse_1s,
                                   btn0 =>de_btn0,
                                   btn1=>de_btn1,
                                   btn2=>de_btn2, 
                                   btn3=>de_btn3,
                                   btn4=>btn4_T,
                                   Rx_data_ready=>Rx_data_ready,
                                   
                                   alarm_out=>alarm_out,
                                   counter_out=>counter_out,
                                   state_out => state_out,
                                   
                                   led0=>led0, 
                                   led1=>led1,
                                   led2=>led2,
                                   led3=>led3,
                                   H_out=>H_out,
                                   M_out=>M_out,
                                   S_out=>S_out
                                   );
                                
UUT_Btn0: Debounce port map (clk => clk, btn => Btn0_T, result => de_Btn0);
UUT_Btn1: Debounce port map (clk => clk, btn => Btn1_T, result => de_Btn1);
UUT_Btn2: Debounce port map (clk => clk, btn => Btn2_T, result => de_Btn2);
UUT_Btn3: Debounce port map (clk => clk, btn => Btn3_T, result => de_Btn3);
--UUT_Btn4: Debounce port map (clk => clk, btn => Btn4_T, result => de_Btn4);

UTT_clk_div: clk_divider port map (clk => clk, clk_1s => pulse_1s); 

UUT_alarm_sound: alarm_sound port map (clk => clk,sound_ena => alarm_out, counter_out => counter_out, sound_out => sound_out);

UUT_Hours : binary_bcd port map (
            clk => clk,
            reset => '0',
            binary_in => H_out,
            bcd0 => H_out_units,
            bcd1 => H_out_tens
            );
            
UUT_Mins : binary_bcd port map (
            clk => clk,
            reset => '0',
            binary_in => M_out,
            bcd0 => M_out_units,
            bcd1 => M_out_tens
            );
            
UUT_Secs : binary_bcd port map (
            clk => clk,
            reset => '0',
            binary_in => S_out,
            bcd0 => S_out_units,
            bcd1 => S_out_tens
            );
            
PMOD_display: PmodOLEDCtrl Port map(
	       CLK => CLK, 
	       pulse_1sec => pulse_1s,
	       
	       CS  => CS, 
	       SDIN => SDIN, 
	       SCLK => SCLK, 
	       DC   => DC, 
	       RES  => RES,
	       VBAT => VBAT,
	       VDD  => VDD,
	       state_in => state_out,
	       H_tens => H_out_tens,
	       H_units => H_out_units,
	       M_tens => M_out_tens,
	       M_units => M_out_units,
	       S_tens => S_out_tens,
	       S_units => S_out_units
	       );
	
UART : UART_Rx Port map(
           CLK => clk,
           Rx_in => Rx,
           
         Data_out => Data_out,   -- data out
         Data_rdy => Rx_data_ready    --data ready flag
);  
    
end behavioral; 
