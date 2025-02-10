-- Elementos de Sistemas
-- by Luciano Soares
-- inversor16.vhd

library IEEE;
use IEEE.STD_LOGIC_1164.all;

entity inversor16 is
  port(
        z   : in STD_LOGIC;
        a   : in STD_LOGIC_VECTOR(15 downto 0);
        y   : out STD_LOGIC_VECTOR(15 downto 0)
      );
end entity;

architecture rtl of inversor16 is
  -- Aqui declaramos sinais (fios auxiliares)
  -- e componentes (outros módulos) que serao
  -- utilizados nesse modulo.

begin
  -- Implementação vem aqui!
  y(0)  <= a(0)  xor z;
  y(1)  <= a(1)  xor z;
  y(2)  <= a(2)  xor z;
  y(3)  <= a(3)  xor z;
  y(4)  <= a(4)  xor z;
  y(5)  <= a(5)  xor z;
  y(6)  <= a(6)  xor z;
  y(7)  <= a(7)  xor z;
  y(8)  <= a(8)  xor z;
  y(9)  <= a(9)  xor z;
  y(10) <= a(10) xor z;
  y(11) <= a(11) xor z;
  y(12) <= a(12) xor z;
  y(13) <= a(13) xor z;
  y(14) <= a(14) xor z;
  y(15) <= a(15) xor z;
end architecture;

