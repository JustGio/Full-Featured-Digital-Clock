----------------------------------------------------------------------------------
-- Company: Digilent Inc.
-- Engineer: Ryan Kim
-- 
-- Create Date:    11:50:03 10/24/2011 
-- Module Name:    OledExample - Behavioral 
-- Project Name: 	 PmodOLED Demo
-- Tool versions:  ISE 13.2
-- Description: Demo for the PmodOLED.  First displays the alphabet for ~4 seconds and then
--				Clears the display, waits for a ~1 second and then displays "This is Digilent's
--				PmodOLED"
--
-- Revision: 1.2
-- Revision 0.01 - File Created
-- Additional Comments: 
--
----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
use work.package_state_machine.all;

entity OledEx is
    Port ( CLK 	: in  STD_LOGIC; --System CLK
           pulse_1sec : in STD_LOGIC;
			  RST 	: in	STD_LOGIC; --Synchronous Reset
			  EN		: in  STD_LOGIC; --Example block enable pin
			  CS  	: out STD_LOGIC; --SPI Chip Select
			  SDO		: out STD_LOGIC; --SPI Data out
			  SCLK	: out STD_LOGIC; --SPI Clock
			  DC		: out STD_LOGIC; --Data/Command Controller
			  state_in : in clock_state_type;
			  H_tens,H_units : in STD_LOGIC_VECTOR(3 downto 0);
              M_tens,M_units : in STD_LOGIC_VECTOR(3 downto 0);
              S_tens,S_units : in STD_LOGIC_VECTOR(3 downto 0)
			  );--Finish flag for example block
end OledEx;

architecture Behavioral of OledEx is

--SPI Controller Component
COMPONENT SpiCtrl
    PORT(
         CLK : IN  std_logic;
         RST : IN  std_logic;
         SPI_EN : IN  std_logic;
         SPI_DATA : IN  std_logic_vector(7 downto 0);
         CS : OUT  std_logic;
         SDO : OUT  std_logic;
         SCLK : OUT  std_logic;
         SPI_FIN : OUT  std_logic
        );
    END COMPONENT;

--Delay Controller Component
COMPONENT Delay
    PORT(
         CLK : IN  std_logic;
         RST : IN  std_logic;
         DELAY_MS : IN  std_logic_vector(11 downto 0);
         DELAY_EN : IN  std_logic;
         DELAY_FIN : OUT  std_logic
        );
    END COMPONENT;
	 
--Character Library, Latency = 1
COMPONENT charLib
  PORT (
    clka : IN STD_LOGIC; --Attach System Clock to it
    addra : IN STD_LOGIC_VECTOR(10 DOWNTO 0); --First 8 bits is the ASCII value of the character the last 3 bits are the parts of the char
    douta : OUT STD_LOGIC_VECTOR(7 DOWNTO 0) --Data byte out
  );
END COMPONENT;

--States for state machine
type states is (Idle,
				ClearDC,
				SetPage,
				PageNum,
				LeftColumn1,
				LeftColumn2,
				SetDC,
				HomeScreen,
				Wait1,
				ClearScreen,
				Wait2,
				TimeScreen,
				UpdateScreen,
				SendChar1,
				SendChar2,
				SendChar3,
				SendChar4,
				SendChar5,
				SendChar6,
				SendChar7,
				SendChar8,
				ReadMem,
				ReadMem2,
				WaitTime,
				Transition1,
				Transition2,
				Transition3,
				Transition4,
				Transition5
					);
type OledRowMem is array (0 to 15) of STD_LOGIC_VECTOR(7 downto 0);

type OledMem is array(0 to 3,0 to 15) of STD_LOGIC_VECTOR(7 downto 0);

--Variable that contains what the screen will be after the next UpdateScreen state
signal current_screen : OledMem; 

--Constant that contains the screen filled with the Home screen
--Row 1 = Blank
--Row 2 = Welcome to 
--Row 3 = Digital Clock! 
--Row 4 = Blank
constant home_screen : OledMem := ((X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20"),	
												(X"20",X"20",X"20",X"57",X"65",X"63",X"6f",X"6d",X"65",X"20",X"74",X"6f",X"20",X"20",X"20",X"20"),
												(X"20",X"44",X"69",X"67",X"69",X"74",X"61",X"6c",X"20",X"43",X"6c",X"6f",X"63",X"6b",X"21",X"20"),
												(X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20"));

--Constants for state machine row 
-- states headers 
-- Row = Time
constant time_row  : OledRowMem := (X"20",X"20",X"20",X"20",X"20",X"20",X"54",X"69",X"6d",X"65",X"20",X"20",X"20",X"20",X"20",X"20");

-- Row = Set time
constant set_time_row  : OledRowMem := (X"20",X"20",X"20",X"20",X"53",X"65",X"74",X"20",X"74",X"69",X"6d",X"65",X"20",X"20",X"20",X"20");

-- Row = Set alarm 
constant set_alarm_row  : OledRowMem := (X"20",X"20",X"20",X"20",X"53",X"65",X"74",X"20",X"61",X"6c",X"61",X"72",X"6d",X"20",X"20",X"20");

-- Row = Set timer                                                   
constant set_timer_row  : OledRowMem := (X"20",X"20",X"20",X"20",X"53",X"65",X"74",X"20",X"74",X"69",X"6d",X"65",X"72",X"20",X"20",X"20");

-- Row = Timer                                                   
constant timer_row  : OledRowMem := (X"20",X"20",X"20",X"20",X"20",X"54",X"69",X"6d",X"65",X"72",X"20",X"20",X"20",X"20",X"20",X"20");

-- Row = Timer paused                                                                             
constant timer_pause_row  : OledRowMem := (X"20",X"20",X"54",X"69",X"6d",X"65",X"72",X"20",X"70",X"61",X"75",X"73",X"65",X"64",X"20",X"20");

-- Row = Stop watch             
constant time_counter_row  : OledRowMem := (X"20",X"20",X"20",X"53",X"74",X"6f",X"70",X"20",X"77",X"61",X"74",X"63",X"68",X"20",X"20",X"20");

-- Row = Stop watch paused            
constant time_counter_pause_row  : OledRowMem := (X"53",X"74",X"6f",X"70",X"20",X"77",X"61",X"74",X"63",X"68",X"70",X"61",X"75",X"73",X"65",X"64");

--Constant that contains the screen filled with the time screen
--Row 1 = State Machine info
--Row 2 = Blank
--Row 3 = Hh : Mm : Ss   --Here HH,MM,SS corresponds to actual time from other module H position 3,4, M -> 8,9, S-> 13,14
--Row 4 = Hh   Mm   Ss   
signal time_screen : OledMem :=                ((X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20"),	
												(X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20"),
												(X"20",X"20",X"20",X"20",X"20",X"20",X"3a",X"20",X"20",X"20",X"20",X"3a",X"20",X"20",X"20",X"20"),
												(X"20",X"20",X"20",X"48",X"68",X"20",X"20",X"20",X"4d",X"6d",X"20",X"20",X"20",X"53",X"73",X"20"));
                                              --(0 ,  1  ,  2  , H_tens , H_unit  , 5  , : ,  7 .....)
--Constant that fills the screen with blank (spaces) entries
constant clear_screen : OledMem :=   ((X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20"),	
												(X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20"),
												(X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20"),
												(X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20",X"20"));

--Current overall state of the state machine
signal current_state : states := Idle;
--State to go to after the SPI transmission is finished
signal after_state : states;
--State to go to after the set page sequence
signal after_page_state : states;
--State to go to after sending the character sequence
signal after_char_state : states;
--State to go to after the UpdateScreen is finished
signal after_update_state : states;

--Contains the value to be outputted to DC
signal temp_dc : STD_LOGIC := '0';

--Variables used in the Delay Controller Block
signal temp_delay_ms : STD_LOGIC_VECTOR (11 downto 0); --amount of ms to delay
signal temp_delay_en : STD_LOGIC := '0'; --Enable signal for the delay block
signal temp_delay_fin : STD_LOGIC; --Finish signal for the delay block

--Variables used in the SPI controller block
signal temp_spi_en : STD_LOGIC := '0'; --Enable signal for the SPI block
signal temp_spi_data : STD_LOGIC_VECTOR (7 downto 0) := (others => '0'); --Data to be sent out on SPI
signal temp_spi_fin : STD_LOGIC; --Finish signal for the SPI block

signal temp_char : STD_LOGIC_VECTOR (7 downto 0) := (others => '0'); --Contains ASCII value for character
signal temp_addr : STD_LOGIC_VECTOR (10 downto 0) := (others => '0'); --Contains address to BYTE needed in memory
signal temp_dout : STD_LOGIC_VECTOR (7 downto 0); --Contains byte outputted from memory
signal temp_page : STD_LOGIC_VECTOR (1 downto 0) := (others => '0'); --Current page
signal temp_index : integer range 0 to 15 := 0; --Current character on page
---------------- edit ----
signal H_10_ascii,H_1_ascii : STD_LOGIC_VECTOR (7 downto 0);   -- added for time display Hour
signal M_10_ascii,M_1_ascii : STD_LOGIC_VECTOR (7 downto 0);    -- added for time display Minutes
signal S_10_ascii,S_1_ascii : STD_LOGIC_VECTOR (7 downto 0);     -- added for time display Seconds

begin
DC <= temp_dc;

--Instantiate SPI Block
 SPI_COMP: SpiCtrl PORT MAP (
          CLK => CLK,
          RST => RST,
          SPI_EN => temp_spi_en,
          SPI_DATA => temp_spi_data,
          CS => CS,
          SDO => SDO,
          SCLK => SCLK,
          SPI_FIN => temp_spi_fin
        );
--Instantiate Delay Block
   DELAY_COMP: Delay PORT MAP (
          CLK => CLK,
          RST => RST,
          DELAY_MS => temp_delay_ms,
          DELAY_EN => temp_delay_en,
          DELAY_FIN => temp_delay_fin
        );
--Instantiate Memory Block
	CHAR_LIB_COMP : charLib
  PORT MAP (
    clka => CLK,
    addra => temp_addr,
    douta => temp_dout
  );
	process (CLK)
	begin
		if(rising_edge(CLK)) then
			case(current_state) is
				--Idle until EN pulled high than intialize Page to 0 and go to state Alphabet afterwards
				when Idle => 
					if(EN = '1') then
						current_state <= ClearDC;
						after_page_state <= HomeScreen;
						temp_page <= "00";
					end if;
				--Set current_screen to constant home_screen and update the screen.  Go to state Wait1 afterwards
				when HomeScreen => 
					current_screen <= home_screen;
					current_state <= UpdateScreen;
					after_update_state <= Wait1;
				--Wait 4 seconds and go to ClearScreen
				when Wait1 => 
					temp_delay_ms <= "001111101000"; --1000
					after_state <= ClearScreen;
					current_state <= Transition3; --Transition3 = The delay transition states
				--set current_screen to constant clear_screen and update the screen. Go to state Wait2 afterwards
--					current_state <= Transition3; --Transition3 = The delay transition states
--				--set current_screen to constant clear_screen and update the screen. Go to state Wait2 afterwards
				when ClearScreen =>                       --******Loop of state times
					current_screen <= clear_screen;
					after_update_state <= Wait2;
					current_state <= UpdateScreen;
				--Wait 1ms and go to DigilentScreen
				when Wait2 =>
					temp_delay_ms <= "000000000001"; --1ms
					after_state <= TimeScreen;
					current_state <= Transition3; --Transition3 = The delay transition states
				--Set currentScreen to changing time_screen and update the screen. Go to state Done afterwards
				when TimeScreen =>                    -- time screen added
					current_screen <= time_screen;    -- time screen added
					after_update_state <= WaitTime;
					current_state <= UpdateScreen;
				--Do nothing until 1 second pulse comes and then current_state is Wait1 again to display time
				when WaitTime	=>               -- time of second added
					if(pulse_1sec = '1') then  --Update time every second						
                        current_state <= ClearScreen;
					end if;
					
				--UpdateScreen State
				--1. Gets ASCII value from current_screen at the current page and the current spot of the page
				--2. If on the last character of the page transition update the page number, if on the last page(3)
				--			then the updateScreen go to "after_update_state" after 
				when UpdateScreen =>
					temp_char <= current_screen(CONV_INTEGER(temp_page),temp_index);
					if(temp_index = 15) then	
						temp_index <= 0;
						temp_page <= temp_page + 1;
						after_char_state <= ClearDC;
						if(temp_page = "11") then
							after_page_state <= after_update_state;
						else	
							after_page_state <= UpdateScreen;
						end if;
					else
						temp_index <= temp_index + 1;
						after_char_state <= UpdateScreen;
					end if;
					current_state <= SendChar1;
				
				--Update Page states
				--1. Sets DC to command mode
				--2. Sends the SetPage Command
				--3. Sends the Page to be set to
				--4. Sets the start pixel to the left column
				--5. Sets DC to data mode
				when ClearDC =>
					temp_dc <= '0';
					current_state <= SetPage;
				when SetPage =>
					temp_spi_data <= "00100010";
					after_state <= PageNum;
					current_state <= Transition1;
				when PageNum =>
					temp_spi_data <= "000000" & temp_page;
					after_state <= LeftColumn1;
					current_state <= Transition1;
				when LeftColumn1 =>
					temp_spi_data <= "00000000";
					after_state <= LeftColumn2;
					current_state <= Transition1;
				when LeftColumn2 =>
					temp_spi_data <= "00010000";
					after_state <= SetDC;
					current_state <= Transition1;
				when SetDC =>
					temp_dc <= '1';
					current_state <= after_page_state;
				--End Update Page States

				--Send Character States
				--1. Sets the Address to ASCII value of char with the counter appended to the end
				--2. Waits a clock for the data to get ready by going to ReadMem and ReadMem2 states
				--3. Send the byte of data given by the block Ram
				--4. Repeat 7 more times for the rest of the character bytes
				when SendChar1 =>
					temp_addr <= temp_char & "000";
					after_state <= SendChar2;
					current_state <= ReadMem;
				when SendChar2 =>
					temp_addr <= temp_char & "001";
					after_state <= SendChar3;
					current_state <= ReadMem;
				when SendChar3 =>
					temp_addr <= temp_char & "010";
					after_state <= SendChar4;
					current_state <= ReadMem;
				when SendChar4 =>
					temp_addr <= temp_char & "011";
					after_state <= SendChar5;
					current_state <= ReadMem;
				when SendChar5 =>
					temp_addr <= temp_char & "100";
					after_state <= SendChar6;
					current_state <= ReadMem;
				when SendChar6 =>
					temp_addr <= temp_char & "101";
					after_state <= SendChar7;
					current_state <= ReadMem;
				when SendChar7 =>
					temp_addr <= temp_char & "110";
					after_state <= SendChar8;
					current_state <= ReadMem;
				when SendChar8 =>
					temp_addr <= temp_char & "111";
					after_state <= after_char_state;
					current_state <= ReadMem;
				when ReadMem =>
					current_state <= ReadMem2;
				when ReadMem2 =>
					temp_spi_data <= temp_dout;
					current_state <= Transition1;
				--End Send Character States
					
				--SPI transitions
				--1. Set SPI_EN to 1
				--2. Waits for SpiCtrl to finish
				--3. Goes to clear state (Transition5)
				when Transition1 =>
					temp_spi_en <= '1';
					current_state <= Transition2;
				when Transition2 =>
					if(temp_spi_fin = '1') then
						current_state <= Transition5;
					end if;
					
				--Delay Transitions
				--1. Set DELAY_EN to 1
				--2. Waits for Delay to finish
				--3. Goes to Clear state (Transition5)
				when Transition3 =>
					temp_delay_en <= '1';
					current_state <= Transition4;
				when Transition4 =>
					if(temp_delay_fin = '1') then
						current_state <= Transition5;
					end if;
				
				--Clear transition
				--1. Sets both DELAY_EN and SPI_EN to 0
				--2. Go to after state
				when Transition5 =>
					temp_spi_en <= '0';
					temp_delay_en <= '0';
					current_state <= after_state;
				--END SPI transitions
				--END Delay Transitions
				--END Clear transition
			
				when others 		=>
					current_state <= Idle;
			end case;
		end if;
	end process;
	
	process(CLK)        -- actual time display 
	begin
	  if(rising_edge(CLK)) then
	    -- Converting BCD  to Ascii by using concatenation
	    H_10_ascii <= X"3" & H_tens;   --(30 & 0011) = 33  => 3  need to convert to hex
	    H_1_ascii  <= X"3" & H_units; 
	    M_10_ascii <= X"3" & M_tens; 
	    M_1_ascii  <= X"3" & M_units; 
	    S_10_ascii <= X"3" & S_tens; 
	    S_1_ascii  <= X"3" & S_units; 
	    time_screen(2,3) <= H_10_ascii;  --(second row,third charecter Hour time) 
	    time_screen(2,4) <= H_1_ascii;
	    time_screen(2,8) <= M_10_ascii;
	    time_screen(2,9) <= M_1_ascii;
	    time_screen(2,13) <= S_10_ascii;
	    time_screen(2,14) <= S_1_ascii;
	    
	    case(state_in) is    -- header use
	    
	       when idle => 
	         for I in 0 to 15 loop           -- every row has 16 charecters 
	           time_screen(0,I) <= time_row(I);
	         end loop;
	       
	       when set_time => 
	         for I in 0 to 15 loop
               time_screen(0,I) <= set_time_row(I);
             end loop;

	       when set_alarm => 
	         for I in 0 to 15 loop
               time_screen(0,I) <= set_alarm_row(I);
             end loop;

	       when set_timer => 
	         for I in 0 to 15 loop
               time_screen(0,I) <= set_timer_row(I);
             end loop;

	       when timer => 
	         for I in 0 to 15 loop
               time_screen(0,I) <= timer_row(I);
             end loop;

	       when time_counter => 
	         for I in 0 to 15 loop
               time_screen(0,I) <= time_counter_row(I);
             end loop;
           
           when others =>
             for I in 0 to 15 loop
                time_screen(0,I) <= time_screen(0,I);         --All other states hold the previous state values
             end loop; 
	    end case;
	  end if;
	end process;
	
end Behavioral;