-- Elementos de Sistemas
-- FlipFlopJK.vhd

library ieee;
use ieee.std_logic_1164.all;

entity FlipFlopJK is
	port(
		clock:  in std_logic;
		J:      in std_logic;
		K:      in std_logic;
		q:      out std_logic:= '0';
		notq:   out std_logic:= '1'
	);
end entity;

architecture arch of FlipFlopJK is

SIGNAL x: STD_LOGIC:= '0';
begin

	process(clock, J, K)
	begin
		if rising_edge(clock) then
			if(J='0' and K = '0') then
				x <= x;
			elsif (J='1' and K='0') then
				x <= '1';
			elsif (J='0' and K='1') then
				x <= '0';
			elsif (J='1' and K='1') then
				x <= not x;
			end if;
		end if;
	end process;

	q <= x;
	notq <= not x;
end architecture;
