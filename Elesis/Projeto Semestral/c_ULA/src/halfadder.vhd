-- Elementos de Sistemas
-- by Luciano Soares
-- HalfAdder.vhd

-- Implementa Half Adder

Library ieee;
use ieee.std_logic_1164.all;

entity HalfAdder is
	port(
		a,b:         in STD_LOGIC;   -- entradas
		soma,carry: out STD_LOGIC   -- sum e carry
	);
end entity;

architecture rtl of HalfAdder is
  -- Aqui declaramos sinais (fios auxiliares)
  -- e componentes (outros módulos) que serao
  -- utilizados nesse modulo.

begin
  -- Implementação vem aqui!
  soma <= a xor b; -- xor é a operação de soma 
  carry <= a and b; -- and é a operação de carry

end architecture;