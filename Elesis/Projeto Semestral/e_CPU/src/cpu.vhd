-- Elementos de Sistemas
-- Developed by Luciano Soares
-- File: CPU.vhd
-- Date: 14/10/2024

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity CPU is
	port (
		clock       : in  STD_LOGIC;                         -- Clock signal for CPU
		reset       : in  STD_LOGIC;                         -- Resets the entire CPU (including Program Counter)
		inM         : in  STD_LOGIC_VECTOR(15 downto 0);     -- Data read from RAM
		instruction : in  STD_LOGIC_VECTOR(17 downto 0);     -- Instruction data from ROM
		outM        : out STD_LOGIC_VECTOR(15 downto 0);     -- Data to write to RAM
		writeM      : out STD_LOGIC;                         -- Triggers RAM to write input data
		addressM    : out STD_LOGIC_VECTOR(15 downto 0);     -- Sends address to RAM
		pcout       : out STD_LOGIC_VECTOR(15 downto 0)      -- Address to be sent to ROM
	);
end entity CPU;

architecture arch of CPU is

	-- Components
	component Mux16 is
		port (
			a   : in  STD_LOGIC_VECTOR(15 downto 0);
			b   : in  STD_LOGIC_VECTOR(15 downto 0);
			sel : in  STD_LOGIC;
			q   : out STD_LOGIC_VECTOR(15 downto 0)
		);
	end component;

	component ALU is
		port (
			x     : in  STD_LOGIC_VECTOR(15 downto 0);
			y     : in  STD_LOGIC_VECTOR(15 downto 0);
			zx    : in  STD_LOGIC;
			nx    : in  STD_LOGIC;
			zy    : in  STD_LOGIC;
			ny    : in  STD_LOGIC;
			f     : in  STD_LOGIC;
			no    : in  STD_LOGIC;
			zr    : out STD_LOGIC;
			ng    : out STD_LOGIC;
			saida : out STD_LOGIC_VECTOR(15 downto 0)
		);
	end component;

	component Register16 is
		port (
			clock  : in  STD_LOGIC;
			input  : in  STD_LOGIC_VECTOR(15 downto 0);
			load   : in  STD_LOGIC;
			output : out STD_LOGIC_VECTOR(15 downto 0)
		);
	end component;

	component pc is
		port (
			clock     : in  STD_LOGIC;
			increment : in  STD_LOGIC;
			load      : in  STD_LOGIC;
			reset     : in  STD_LOGIC;
			input     : in  STD_LOGIC_VECTOR(15 downto 0);
			output    : out STD_LOGIC_VECTOR(15 downto 0)
		);
	end component;

	component ControlUnit is
		port (
			instruction : in  STD_LOGIC_VECTOR(17 downto 0);
			zr          : in  STD_LOGIC;
			ng          : in  STD_LOGIC;
			muxALUI_A   : out STD_LOGIC;
			muxAM       : out STD_LOGIC;
			zx          : out STD_LOGIC;
			nx          : out STD_LOGIC;
			zy          : out STD_LOGIC;
			ny          : out STD_LOGIC;
			f           : out STD_LOGIC;
			no          : out STD_LOGIC;
			loadA       : out STD_LOGIC;
			loadD       : out STD_LOGIC;
			loadM       : out STD_LOGIC;
			loadPC      : out STD_LOGIC
		);
	end component;

	-- Signals
	signal c_muxALUI_A : STD_LOGIC;
	signal c_muxAM     : STD_LOGIC;
	signal c_zx        : STD_LOGIC;
	signal c_nx        : STD_LOGIC;
	signal c_zy        : STD_LOGIC;
	signal c_ny        : STD_LOGIC;
	signal c_f         : STD_LOGIC;
	signal c_no        : STD_LOGIC;
	signal c_loadA     : STD_LOGIC;
	signal c_loadD     : STD_LOGIC;
	signal c_loadPC    : STD_LOGIC;
	signal c_zr        : STD_LOGIC := '0';
	signal c_ng        : STD_LOGIC := '0';

	-- Data signals
	signal s_muxALUI_Aout : STD_LOGIC_VECTOR(15 downto 0);
	signal s_muxAM_out    : STD_LOGIC_VECTOR(15 downto 0);
	signal s_regAout      : STD_LOGIC_VECTOR(15 downto 0);
	signal s_regDout      : STD_LOGIC_VECTOR(15 downto 0);
	signal s_ALUout       : STD_LOGIC_VECTOR(15 downto 0);
	signal s_pcout        : STD_LOGIC_VECTOR(15 downto 0);

begin

	-- Control Unit
	CU: ControlUnit
		port map (
			instruction => instruction,
			zr          => c_zr,
			ng          => c_ng,
			muxALUI_A   => c_muxALUI_A,
			muxAM       => c_muxAM,
			zx          => c_zx,
			nx          => c_nx,
			zy          => c_zy,
			ny          => c_ny,
			f           => c_f,
			no          => c_no,
			loadA       => c_loadA,
			loadD       => c_loadD,
			loadM       => writeM,
			loadPC      => c_loadPC
		);

	-- Mux
	MuxALUI_A: Mux16
		port map (
			a   => s_ALUout,
			b   => instruction(15 downto 0),
			sel => c_muxALUI_A,
			q   => s_muxALUI_Aout
		);

	-- Register A
	RegA: Register16
		port map (
			clock  => clock,
			input  => s_muxALUI_Aout,
			load   => c_loadA,
			output => s_regAout
		);

	-- Mux
	MuxAM: Mux16
		port map (
			a   => s_regAout,
			b   => inM,
			sel => c_muxAM,
			q   => s_muxAM_out
		);

	-- Register D
	RegD: Register16
		port map (
			clock  => clock,
			input  => s_ALUout,
			load   => c_loadD,
			output => s_regDout
		);

	-- ALU
	ULA: ALU
		port map (
			x     => s_regDout,
			y     => s_muxAM_out,
			zx    => c_zx,
			nx    => c_nx,
			zy    => c_zy,
			ny    => c_ny,
			f     => c_f,
			no    => c_no,
			zr    => c_zr,
			ng    => c_ng,
			saida => s_ALUout
		);

	-- Program Counter
	ProgramCounter: pc
		port map (
			clock     => clock,
			increment => '1',
			load      => c_loadPC,
			reset     => reset,
			input     => s_regAout,
			output    => s_pcout
		);

	-- Outputs
	outM     <= s_ALUout;
	addressM <= s_regAout;
	pcout    <= s_pcout;

end architecture;
