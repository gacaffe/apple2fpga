-------------------------------------------------------------------------------
--
-- DE2 top-level module for the Apple ][
--
-- Stephen A. Edwards, Columbia University, sedwards@cs.columbia.edu
--
-- From an original by Terasic Technology, Inc.
-- (DE2_TOP.v, part of the DE2 system board CD supplied by Altera)
--
-------------------------------------------------------------------------------
-- Adapted to DE2-115 board by Gabriel Caffarena (GCF), University CEU San Pablo
-- Pins adapted to DE2-115
-- RGB reduced to 8 bits
-- vga_controller.vhd modified to use 8 bits per RGB color

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity hex7seg is 
  port (
    input  : in  unsigned(3 downto 0);
    output : out unsigned(6 downto 0));
end hex7seg;

architecture combinational of hex7seg is
  signal output_n : unsigned(6 downto 0);
begin
  with input select
    output_n <=
    "0111111" when "0000",
    "0000110" when "0001",
    "1011011" when "0010",
    "1001111" when "0011",
    "1100110" when "0100",
    "1101101" when "0101",
    "1111101" when "0110",
    "0000111" when "0111",
    "1111111" when "1000",
    "1101111" when "1001", 
    "1110111" when "1010",
    "1111100" when "1011", 
    "0111001" when "1100", 
    "1011110" when "1101", 
    "1111001" when "1110",
    "1110001" when "1111",
    "XXXXXXX" when others;

  output <= not output_n;

end combinational;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity DE2_TOP is
  port (
    -- Clocks
    
 --   CLOCK_27,                                    -- 27 MHz           -- GCF: NOT USED AND NOT PRESENT IN DE2-115
 --   CLOCK_50,                                      -- 50 MHz
    --EXT_CLOCK : in std_logic;                      -- External Clock     -- GCF: NOT USED
	-- SMA_CLKIN : in std_logic;                      -- External Clock     -- GCF: NOT USED

		CLOCK_50: in std_logic;                      -- 50 MHz

    -- Buttons and switches
    
    KEY : in std_logic_vector(3 downto 0);         -- Push buttons			-- GCF: RESET + PADDLE BUTTONS (GAMEPORT)
    SW  : in unsigned(17 downto 0);                -- DPDT switches			-- GCF: 17 - BW VS COLOR_LINE; 9-0 - IMAGE

    -- LED displays

    HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, HEX6, HEX7 -- 7-segment displays
    : out unsigned(6 downto 0);
    LEDG : out std_logic_vector(8 downto 0);       -- Green LEDs			-- GCF: MISC INFO: PC COUNTER, SPEAKER, ETC
    LEDR : out std_logic_vector(17 downto 0);      -- Red LEDs

    -- RS-232 interface

   -- UART_TXD : out std_logic;                      -- UART transmitter   -- GCF: NOT USED
   -- UART_RXD : in std_logic;                       -- UART receiver

    -- IRDA interface

    -- IRDA_TXD : out std_logic;                      -- IRDA Transmitter
    -- IRDA_RXD : in std_logic;                       -- IRDA Receiver			-- GCF: NOT USED

    -- SRAM
    
    -- GCF: NOT USED
--	 DRAM_DQ : inout std_logic_vector(31 downto 0); -- Data Bus					-- GCF: 31 downto 0 
--    DRAM_ADDR : out std_logic_vector(12 downto 0); -- Address Bus    		-- GCF: 12 downto 0
--    DRAM_LDQM,                                     -- Low-byte Data Mask   
--    DRAM_UDQM,                                     -- High-byte Data Mask
--    DRAM_WE_N,                                     -- Write Enable
--    DRAM_CAS_N,                                    -- Column Address Strobe
--    DRAM_RAS_N,                                    -- Row Address Strobe
--    DRAM_CS_N,                                     -- Chip Select
--    DRAM_BA_0,                                     -- Bank Address 0
--    DRAM_BA_1,                                     -- Bank Address 0
--    DRAM_CLK,                                      -- Clock
--    DRAM_CKE : out std_logic;                      -- Clock Enable

    -- FLASH
    
--    FL_DQ : inout std_logic_vector(7 downto 0);      -- Data bus			-- GCF: NOT USED
--    FL_ADDR : out std_logic_vector(21 downto 0);  -- Address bus
--    FL_WE_N,                                         -- Write Enable
--    FL_RST_N,                                        -- Reset
--    FL_OE_N,                                         -- Output Enable
--    FL_CE_N : out std_logic;                         -- Chip Enable

    -- SRAM
    
    SRAM_DQ : inout unsigned(15 downto 0);         -- Data bus 16 Bits
    SRAM_ADDR : out unsigned(17 downto 0);         -- Address bus 18 Bits
    SRAM_UB_N,                                     -- High-byte Data Mask 
    SRAM_LB_N,                                     -- Low-byte Data Mask 
    SRAM_WE_N,                                     -- Write Enable
    SRAM_CE_N,                                     -- Chip Enable
    SRAM_OE_N : out std_logic;                     -- Output Enable

    -- USB controller
    
--    OTG_DATA : inout std_logic_vector(15 downto 0); -- Data bus					-- GCF: NOT USED
--    OTG_ADDR : out std_logic_vector(1 downto 0);    -- Address
--    OTG_CS_N,                                       -- Chip Select
--    OTG_RD_N,                                       -- Write
--    OTG_WR_N,                                       -- Read
--    OTG_RST_N,                                      -- Reset
--    OTG_FSPEED,                     -- USB Full Speed, 0 = Enable, Z = Disable
--    OTG_LSPEED : out std_logic;     -- USB Low Speed, 0 = Enable, Z = Disable
--    OTG_INT0,                                       -- Interrupt 0
--    OTG_INT1,                                       -- Interrupt 1
--    OTG_DREQ0,                                      -- DMA Request 0
--    OTG_DREQ1 : in std_logic;                       -- DMA Request 1   
--    OTG_DACK0_N,                                    -- DMA Acknowledge 0
--    OTG_DACK1_N : out std_logic;                    -- DMA Acknowledge 1

    -- 16 X 2 LCD Module
    
--    LCD_ON,                     -- Power ON/OFF											-- GCF: NOT USED
--    LCD_BLON,                   -- Back Light ON/OFF
--    LCD_RW,                     -- Read/Write Select, 0 = Write, 1 = Read
--    LCD_EN,                     -- Enable
--    LCD_RS : out std_logic;     -- Command/Data Select, 0 = Command, 1 = Data
--    LCD_DATA : inout std_logic_vector(7 downto 0); -- Data bus 8 bits

    -- SD card interface
    
   -- SD_DAT->SD_DAT[0] PIN_AE14 
	-- SD_DAT[1] PIN_AF13 SD 
	-- SD_DAT[2] PIN_AB14 SD 
	-- SD->DAT[3] -> SD_DAT[3] PIN_AC14 
	-- The .csv file does not match: so SD_DAT and SD_DAT3 must be assigned manually and any conflicts with SD_DAT[3 downto 0] must be removed
	
	 SD_DAT : in std_logic;      -- SD Card Data      SD pin 7 "DAT 0/DataOut"		 
    SD_DAT3 : out std_logic;    -- SD Card Data 3    SD pin 1 "DAT 3/nCS"
    SD_CMD : out std_logic;     -- SD Card Command   SD pin 2 "CMD/DataIn"
    SD_CLK : out std_logic;     -- SD Card Clock     SD pin 5 "CLK"

    -- USB JTAG link
    
--    TDI,                        -- CPLD -> FPGA (data in)			-- GCF: NOT USED
--    TCK,                        -- CPLD -> FPGA (clk)
--    TCS : in std_logic;         -- CPLD -> FPGA (CS)
--    TDO : out std_logic;        -- FPGA -> CPLD (data out)

    -- I2C bus
    
    I2C_SDAT : inout std_logic; -- I2C Data
    I2C_SCLK : out std_logic;   -- I2C Clock

    -- PS/2 port

    PS2_DAT,                    -- Data					-- GCF: KEYBOARD
    PS2_CLK : in std_logic;     -- Clock

    -- VGA output
    
    VGA_CLK,                                            -- Clock
    VGA_HS,                                             -- H_SYNC
    VGA_VS,                                             -- V_SYNC
    --VGA_BLANK,                                          -- BLANK
	 VGA_BLANK_N,                                          -- BLANK	 -- GCF: AS IN DE2-115 cvs FILE
    --VGA_SYNC : out std_logic;                           -- SYNC
	 VGA_SYNC_N : out std_logic;                           -- SYNC		 -- GCF: AS IN DE2-115 cvs FILE
    VGA_R,                                              -- Red[9:0]
    VGA_G,                                              -- Green[9:0]
    --VGA_B : out unsigned(9 downto 0);                   -- Blue[9:0]
	 VGA_B : out unsigned(7 downto 0);                   -- Blue[9:0]			-- GCF: The DE2-115 VGA chip REQUIRES ONLY 8 BITS (I'm afraid the color effects are disable)

    --  Ethernet Interface
    
--    ENET_DATA : inout std_logic_vector(15 downto 0);    -- DATA bus 16Bits		-- GCF: NOT USED
--    ENET_CMD,           -- Command/Data Select, 0 = Command, 1 = Data
--    ENET_CS_N,                                          -- Chip Select
--    ENET_WR_N,                                          -- Write
--    ENET_RD_N,                                          -- Read
--    ENET_RST_N,                                         -- Reset
--    ENET_CLK : out std_logic;                           -- Clock 25 MHz
--    ENET_INT : in std_logic;                            -- Interrupt
--    
    -- Audio CODEC
    
    AUD_ADCLRCK : inout std_logic;                      -- ADC LR Clock	
    AUD_ADCDAT : in std_logic;                          -- ADC Data
    AUD_DACLRCK : inout std_logic;                      -- DAC LR Clock
    AUD_DACDAT : out std_logic;                         -- DAC Data
    AUD_BCLK : inout std_logic;                         -- Bit-Stream Clock
    --AUD_XCK : out std_logic;                            -- Chip Clock
    AUD_XCK : out std_logic                            -- Chip Clock
	 
    -- Video Decoder
    
--    TD_DATA : in std_logic_vector(7 downto 0);  -- Data bus 8 bits			-- GCF: NOT USED
--    TD_HS,                                      -- H_SYNC
--    TD_VS : in std_logic;                       -- V_SYNC
--    TD_RESET : out std_logic;                   -- Reset
--    
    -- General-purpose I/O
    
--    GPIO_0,                                      -- GPIO Connection 0		-- GCF: NOT USED
--    GPIO_1 : inout std_logic_vector(35 downto 0) -- GPIO Connection 1   
    );
  
end DE2_TOP;

architecture datapath of DE2_TOP is

  component CLK28MPLL_DE2_115 is
    port (
      inclk0    : in std_logic;
      c0        : out std_logic;
      c1        : out std_logic);
  end component;
  
  signal SD_DAT3_DBG, SD_CMD_DBG: std_logic; 	-- GCF: for debugging purposes, it's used to read SD_DAT3
  signal CLK_28M, CLK_14M, CLK_2M, PRE_PHASE_ZERO : std_logic;
  signal IO_SELECT, DEVICE_SELECT : std_logic_vector(7 downto 0);
  signal ADDR : unsigned(15 downto 0);
  signal D, PD : unsigned(7 downto 0);

  signal ram_we : std_logic;
  signal VIDEO, HBL, VBL, LD194 : std_logic;
  signal COLOR_LINE : std_logic;
  signal COLOR_LINE_CONTROL : std_logic;
  signal GAMEPORT : std_logic_vector(7 downto 0);
  signal cpu_pc : unsigned(15 downto 0);

  signal K : unsigned(7 downto 0);
  signal read_key : std_logic;

  signal flash_clk : unsigned(22 downto 0) := (others => '0');
  signal power_on_reset : std_logic := '1';
  signal reset : std_logic;

  signal speaker : std_logic;

--  signal track : unsigned(5 downto 0);
  signal TRACK : unsigned(5 downto 0);	-- GCF
  signal image : unsigned(9 downto 0);
  signal TRACKMSB : unsigned(3 downto 0);
  signal D1_ACTIVE, D2_ACTIVE : std_logic;
  signal TRACK_ADDR : unsigned(13 downto 0);
  signal TRACK_RAM_ADDR : unsigned(13 downto 0);  
  signal tra : unsigned(15 downto 0);		-- GCF: ???
  signal TRACK_RAM_DI : unsigned(7 downto 0);
  signal TRACK_RAM_WE : std_logic;
  

  signal CS_N, MOSI, MISO, SCLK : std_logic;

begin

  reset <= (not KEY(3)) or power_on_reset;

  power_on : process(CLK_14M)
  begin
    if rising_edge(CLK_14M) then
      if flash_clk(22) = '1' then
        power_on_reset <= '0';
      end if;
    end if;
  end process;

  -- In the Apple ][, this was a 555 timer
  flash_clkgen : process (CLK_14M)
  begin
    if rising_edge(CLK_14M) then
      flash_clk <= flash_clk + 1;
    end if;     
  end process;

  -- Use a PLL to divide the 50 MHz down to 28 MHz and 14 MHz
  pll : CLK28MPLL_DE2_115 port map (
    inclk0 => CLOCK_50,
    c0     => CLK_28M,
    c1     => CLK_14M
    );

  -- Paddle buttons
  GAMEPORT <=  "0000" & (not KEY(2 downto 0)) & "0";

  COLOR_LINE_CONTROL <= COLOR_LINE and SW(17);  -- Color or B&W mode
  
  core : entity work.apple2 port map (
    CLK_14M        => CLK_14M,
    CLK_2M         => CLK_2M,
    PRE_PHASE_ZERO => PRE_PHASE_ZERO,
    FLASH_CLK      => flash_clk(22),
    reset          => reset,
    ADDR           => ADDR,
    ram_addr       => SRAM_ADDR(15 downto 0),
    D              => D,
    ram_do         => SRAM_DQ(7 downto 0),
    PD             => PD,
    ram_we         => ram_we,
    VIDEO          => VIDEO,
    COLOR_LINE     => COLOR_LINE,
    HBL            => HBL,
    VBL            => VBL,
    LD194          => LD194,
    K              => K,
    read_key       => read_key,
    AN             => LEDG(7 downto 4),
    GAMEPORT       => GAMEPORT,
    IO_SELECT      => IO_SELECT,
    DEVICE_SELECT  => DEVICE_SELECT,
    pcDebugOut     => cpu_pc,
    speaker        => speaker
    );

  vga : entity work.vga_controller port map (
    CLK_28M    => CLK_28M,
    VIDEO      => VIDEO,
    COLOR_LINE => COLOR_LINE_CONTROL,
    HBL        => HBL,
    VBL        => VBL,
    LD194      => LD194,
    VGA_CLK    => VGA_CLK,
    VGA_HS     => VGA_HS,
    VGA_VS     => VGA_VS,
    VGA_BLANK  => VGA_BLANK_N,
    VGA_R      => VGA_R,
    VGA_G      => VGA_G,
    VGA_B      => VGA_B
    );

  VGA_SYNC_N <= '0';

  keyboard : entity work.keyboard port map (
    PS2_Clk  => PS2_CLK,
    PS2_Data => PS2_DAT,
    CLK_14M  => CLK_14M,
    reset    => reset,
    read     => read_key,
    K        => K
    );

  disk : entity work.disk_ii port map (
    CLK_14M        => CLK_14M,
    CLK_2M         => CLK_2M,
    PRE_PHASE_ZERO => PRE_PHASE_ZERO,
    IO_SELECT      => IO_SELECT(6),
    DEVICE_SELECT  => DEVICE_SELECT(6),
    RESET          => reset,
    A              => ADDR,
    D_IN           => D,
    D_OUT          => PD,
    TRACK          => TRACK,
    TRACK_ADDR     => TRACK_ADDR,
    D1_ACTIVE      => D1_ACTIVE,
    D2_ACTIVE      => D2_ACTIVE,
    ram_write_addr => TRACK_RAM_ADDR,
    ram_di         => TRACK_RAM_DI,
    ram_we         => TRACK_RAM_WE
    );

  sdcard_interface : entity work.spi_controller port map (
    CLK_14M        => CLK_14M,
    RESET          => RESET,

    CS_N           => CS_N,
    MOSI           => MOSI,
    MISO           => MISO,
    SCLK           => SCLK,
    
    track          => TRACK,
    image          => image,
    
    ram_write_addr => TRACK_RAM_ADDR,
    ram_di         => TRACK_RAM_DI,
    ram_we         => TRACK_RAM_WE
    );


  image <= SW(9 downto 0);

  SD_DAT3 <= CS_N;
  SD_CMD  <= MOSI;
  MISO    <= SD_DAT;
  SD_CLK  <= SCLK;

  i2c : entity work.i2c_controller port map (
    CLK   => CLOCK_50,
    SCLK  => I2C_SCLK,
    SDAT  => I2C_SDAT,
    reset => reset
  );

  audio_output : entity work.wm8731_audio port map (
    clk          => CLK_14M,
    reset        => reset,
    data         => speaker & "000000000000000",
  
    -- Audio interface signals
    AUD_ADCLRCK  => AUD_ADCLRCK,
    AUD_ADCDAT   => AUD_ADCDAT,
    AUD_DACLRCK  => AUD_DACLRCK,
    AUD_DACDAT   => AUD_DACDAT,
    AUD_BCLK     => AUD_BCLK
  );

  AUD_XCK <= CLK_14M;

  -- Processor PC on the right four digits
  digit0 : entity work.hex7seg port map (cpu_pc( 3 downto  0), HEX0);
  digit1 : entity work.hex7seg port map (cpu_pc( 7 downto  4), HEX1);
  digit2 : entity work.hex7seg port map (cpu_pc(11 downto  8), HEX2);
  digit3 : entity work.hex7seg port map (cpu_pc(15 downto 12), HEX3);

  -- Current disk track on middle two digits 
  TRACKMSB <= "00" & TRACK(5 downto 4);
  digit4 : entity work.hex7seg port map (TRACK(3 downto 0), HEX4);  -- GCF
  digit5 : entity work.hex7seg port map (TRACKMSB, HEX5);			  -- GCF

  -- Current disk image on left two digits
  digit6 : entity work.hex7seg port map (image(3 downto 0), HEX6);
  digit7 : entity work.hex7seg port map (image(7 downto 4), HEX7);


  SRAM_DQ(7 downto 0) <= D when ram_we = '1' else (others => 'Z');
  SRAM_ADDR(17) <= '0';
  SRAM_ADDR(16) <= '0';
  SRAM_UB_N <= '1';
  SRAM_LB_N <= '0';
  SRAM_CE_N <= '0';
  SRAM_WE_N <= not ram_we;
  SRAM_OE_N <= ram_we;

  -- Decode the top four bits of the PC on the red LEDs
  with  cpu_pc(15 downto 12) select LEDR(15 downto 0) <=
    "0000000000000001" when x"0",
    "0000000000000010" when x"1",
    "0000000000000100" when x"2",
    "0000000000001000" when x"3",
    "0000000000010000" when x"4",
    "0000000000100000" when x"5",
    "0000000001000000" when x"6",
    "0000000010000000" when x"7",
    "0000000100000000" when x"8",
    "0000001000000000" when x"9",
    "0000010000000000" when x"A",
    "0000100000000000" when x"B",
    "0001000000000000" when x"C",
    "0010000000000000" when x"D",
    "0100000000000000" when x"E",
    "1000000000000000" when x"F",
    "XXXXXXXXXXXXXXXX" when others;

  
  LEDR(17 downto 16) <= (others => '0');
  
  
  LEDG(8) <= D1_ACTIVE;
  --LEDG(3 downto 1) <= (others => '0');
  --LEDG(1)<=SD_DAT;	-- GCF: 				Debugging SD card
  --LEDG(2)<=MOSI;		-- GCF: SD_DAT3 - Debugging SD card
  --LEDG(3)<=CS_N;		-- GCF: SD_CMD  - Debugging SD card
  LEDG(1)<=CLK_14M;	-- GCF: 				Debugging SD card
  LEDG(2)<=CLK_28M;		-- GCF: SD_DAT3 - Debugging SD card
  LEDG(3)<=CLOCK_50;		-- GCF: SD_CMD  - Debugging SD card
  LEDG(0)<= speaker;
  
--  UART_TXD <= '0';
--  DRAM_ADDR <= (others => '0');
--  DRAM_LDQM <= '0';
--  DRAM_UDQM <= '0';
--  DRAM_WE_N <= '1';
--  DRAM_CAS_N <= '1';
--  DRAM_RAS_N <= '1';
--  DRAM_CS_N <= '1';
--  DRAM_BA_0 <= '0';
--  DRAM_BA_1 <= '0';
--  DRAM_CLK <= '0';
--  DRAM_CKE <= '0';
--  FL_ADDR <= (others => '0');
--  FL_WE_N <= '1';
--  FL_RST_N <= '0';
--  FL_OE_N <= '1';
--  FL_CE_N <= '1';
--  OTG_ADDR <= (others => '0');
--  OTG_CS_N <= '1';
--  OTG_RD_N <= '1';
--  OTG_RD_N <= '1';
--  OTG_WR_N <= '1';
--  OTG_RST_N <= '1';
--  OTG_FSPEED <= '1';
--  OTG_LSPEED <= '1';
--  OTG_DACK0_N <= '1';
--  OTG_DACK1_N <= '1';
--
--  LCD_ON <= '0';
--  LCD_BLON <= '0';
--  LCD_RW <= '1';
--  LCD_EN <= '0';
--  LCD_RS <= '0';
--
--  TDO <= '0';
--
--  ENET_CMD <= '0';
--  ENET_CS_N <= '1';
--  ENET_WR_N <= '1';
--  ENET_RD_N <= '1';
--  ENET_RST_N <= '1';
--  ENET_CLK <= '0';
--
--  TD_RESET <= '0';

  -- Set all bidirectional ports to tri-state
 -- DRAM_DQ     <= (others => 'Z');
-- FL_DQ       <= (others => 'Z');
  SRAM_DQ(15 downto 8) <= (others => 'Z');
-- OTG_DATA    <= (others => 'Z');
-- LCD_DATA    <= (others => 'Z');
-- ENET_DATA   <= (others => 'Z');
-- GPIO_0      <= (others => 'Z');
-- GPIO_1      <= (others => 'Z');

end datapath;
