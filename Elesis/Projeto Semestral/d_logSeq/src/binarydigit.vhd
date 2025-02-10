-- Elementos de Sistemas
-- by Luciano Soares
-- BinaryDigit.vhd

Library ieee;
use ieee.std_logic_1164.all;

entity BinaryDigit is
	port(
		clock:   in STD_LOGIC;
		input:   in STD_LOGIC;
		load:    in STD_LOGIC;
		output: out STD_LOGIC
	);
end entity;

architecture arch of BinaryDigit is

	component FlipFlopD is
		port(
			clock:  in std_logic;
			d:      in std_logic;
			clear:  in std_logic;
			preset: in std_logic;
			q:     out std_logic
		);
	end component;

	component Mux2Way is
		port (
			a:   in  STD_LOGIC;
			b:   in  STD_LOGIC;
			sel: in  STD_LOGIC;
			q:   out STD_LOGIC);
	end component;

	signal dffout, muxout: std_logic;

begin

	-- Multiplexador 2:1, seleciona entre o valor atual (dffout) e a nova entrada (input)
	U1: Mux2Way
		port map (
			a => dffout,   -- valor armazenado (bit atual)
			b => input,    -- novo bit de entrada
			sel => load,   -- quando load for 1, carrega o novo bit
			q => muxout    -- saída do mux
		);

	-- Flip-flop tipo D, armazena o valor selecionado
	U2: FlipFlopD
		port map (
			clock => clock,
			d => muxout,   -- valor selecionado pelo mux
			clear => '0',  -- clear não utilizado, fixado em 0
			preset => '0', -- preset não utilizado, fixado em 0
			q => dffout    -- saída do flip-flop
		);

	-- A saída do módulo é o valor armazenado no flip-flop
	output <= dffout;

end architecture;
