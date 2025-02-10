-- Elementos de Sistemas
-- by Luciano Soares
-- Add16.vhd

-- Soma dois valores de 16 bits
-- ignorando o carry mais significativo

library IEEE;
use IEEE.STD_LOGIC_1164.all;

entity Add16 is
	port(
		a   :  in STD_LOGIC_VECTOR(15 downto 0);
		b   :  in STD_LOGIC_VECTOR(15 downto 0);
		q   : out STD_LOGIC_VECTOR(15 downto 0);
    xor_ula : out std_logic

	);
end entity;

architecture rtl of Add16 is
  
  -- Declaração dos sinais internos
    signal carry: std_logic_vector(15 downto 0); -- Sinais de carry entre os bits

  component FullAdder is
    port(
      a,b,c:      in STD_LOGIC;   -- entradas
      soma,carry: out STD_LOGIC   -- sum e carry
    );
  end component;

begin
  -- Implementação vem aqui!
  U0: FullAdder
    port map (
      a => a(0),
      b => b(0),
      c => '0',
      soma => q(0),
      carry => carry(0)
    );

  U1: FullAdder
    port map (
      a => a(1),
      b => b(1),
      c => carry(0),
      soma => q(1),
      carry => carry(1)
    );

  U2: FullAdder
    port map (
      a => a(2),
      b => b(2),
      c => carry(1),
      soma => q(2),
      carry => carry(2)
    );

  U3: FullAdder
    port map (
      a => a(3),
      b => b(3),
      c => carry(2),
      soma => q(3),
      carry => carry(3)
    );

  U4: FullAdder
    port map (
      a => a(4),
      b => b(4),
      c => carry(3),
      soma => q(4),
      carry => carry(4)
    );

  U5: FullAdder
    port map (
      a => a(5),
      b => b(5),
      c => carry(4),
      soma => q(5),
      carry => carry(5)
    );

  U6: FullAdder
    port map (
      a => a(6),
      b => b(6),
      c => carry(5),
      soma => q(6),
      carry => carry(6)
    );

  U7: FullAdder
    port map (
      a => a(7),
      b => b(7),
      c => carry(6),
      soma => q(7),
      carry => carry(7)
    );

  U8: FullAdder
    port map (
      a => a(8),
      b => b(8),
      c => carry(7),
      soma => q(8),
      carry => carry(8)
    );

  U9: FullAdder
    port map (
      a => a(9),
      b => b(9),
      c => carry(8),
      soma => q(9),
      carry => carry(9)
    );

  U10: FullAdder
    port map (
      a => a(10),
      b => b(10),
      c => carry(9),
      soma => q(10),
      carry => carry(10)
    );

  U11: FullAdder
    port map (
      a => a(11),
      b => b(11),
      c => carry(10),
      soma => q(11),
      carry => carry(11)
    );

  U12: FullAdder
    port map (
      a => a(12),
      b => b(12),
      c => carry(11),
      soma => q(12),
      carry => carry(12)
    );

  U13: FullAdder
    port map (
      a => a(13),
      b => b(13),
      c => carry(12),
      soma => q(13),
      carry => carry(13)
    );

  U14: FullAdder
    port map (
      a => a(14),
      b => b(14),
      c => carry(13),
      soma => q(14),
      carry => carry(14)
    );

  U15: FullAdder
    port map (
      a => a(15),
      b => b(15),
      c => carry(14),
      soma => q(15),
      carry => carry(15)  -- O carry final é ignorado
    );

    xor_ula <= carry(14) xor carry(15);

    

    
    
end architecture;
