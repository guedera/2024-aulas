--
-- Elementos de Sistemas - Aula 5 - Logica Combinacional
-- Rafael . Corsi @ insper . edu . br 
--
-- Arquivo exemplo para acionar os LEDs e ler os bottoes
-- da placa DE0-CV utilizada no curso de elementos de 
-- sistemas do 3s da eng. da computacao

----------------------------
-- Bibliotecas ieee       --
----------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.all;

----------------------------
-- Entrada e saidas do bloco
----------------------------
entity TopLevel2 is
	port(
		SW      : in  std_logic_vector(9 downto 0);
		KEY     : in  std_logic_vector(3 downto 0);
		LEDR    : out std_logic_vector(9 downto 0)
	);
end entity;

----------------------------
-- Implementacao do bloco -- 
----------------------------
architecture rtl of TopLevel2 is


component Ram8 is
	port(
		clock:   in  STD_LOGIC;
		input:   in  STD_LOGIC_VECTOR(15 downto 0);
		load:    in  STD_LOGIC;
		address: in  STD_LOGIC_VECTOR( 2 downto 0);
		output:  out STD_LOGIC_VECTOR(15 downto 0)
	);
end component;

--------------
-- signals
--------------

signal clock, clear, set : std_logic;
signal ram_input   : std_logic_vector(15 downto 0):= "1010101100111001";
signal ram_address : std_logic_vector(2 downto 0);
signal ram_output  : std_logic_vector(15 downto 0);
---------------
-- implementacao
---------------
begin

Clock <= not KEY(0); -- os botoes quando nao apertado vale 1
                     -- e apertado 0, essa logica inverte isso
clear <= not KEY(1);
set	<= not KEY(2);

-- Address for RAM, using SW(2 downto 0) as the address input
ram_address <= SW(2 downto 0);


u0 : Ram8 port map (
		clock   => clock,
		input   => ram_input,
		load    => set,
		address => ram_address,
		output  => ram_output
	);		
-- Display RAM output on LEDs
LEDR <= ram_output(9 downto 0); -- Mostra os 10 bits menos significativos do output


end rtl;