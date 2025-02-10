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

----------------------------
-- Entrada e saidas do bloco
----------------------------
entity TopLevel is
	port(
		SW      : in  std_logic_vector(9 downto 0);
		LEDR    : out std_logic_vector(9 downto 0);
    HEX0    : out STD_LOGIC_VECTOR(6 downto 0);
    HEX1    :out STD_LOGIC_VECTOR(6 downto 0);
    HEX2    :out STD_LOGIC_VECTOR(6 downto 0);
    HEX3    :out STD_LOGIC_VECTOR(6 downto 0)
	);
end entity;

----------------------------
-- Implementacao do bloco --
----------------------------
architecture rtl of TopLevel is

--------------
-- signals
--------------

  signal x : std_logic_vector(15 downto 0) := x"0073"; -- 115
  signal y : std_logic_vector(15 downto 0) := x"005F"; -- 95

--------------
-- component
--------------
  component HalfAdder is
    port(
      a,b:         in STD_LOGIC;   -- entradas
      soma,vaium: out STD_LOGIC   -- sum e carry
      );
  end component;

  component FullAdder is
      port(
          a,b,c:      in STD_LOGIC;   -- entradas
          soma,vaium: out STD_LOGIC   -- sum e carry
          );
    end component;

    component ALU is
      port (
          x,y:   in STD_LOGIC_VECTOR(15 downto 0); -- entradas de dados da ALU
          zx:    in STD_LOGIC;                     -- zera a entrada x
          nx:    in STD_LOGIC;                     -- inverte a entrada x
          zy:    in STD_LOGIC;                     -- zera a entrada y
          ny:    in STD_LOGIC;                     -- inverte a entrada y
          f:     in STD_LOGIC_VECTOR(1 downto 0);                     -- se 0 calcula x & y, senão x + y
          no:    in STD_LOGIC;                     -- inverte o valor da saída
          zr:    out STD_LOGIC;                    -- setado se saída igual a zero
          ng:    out STD_LOGIC;                    -- setado se saída é negativa
          saida: out STD_LOGIC_VECTOR(15 downto 0);-- saída de dados da ALU
          estouro : out std_logic
      );
    end component;

    component sevenseg is
      port (
          
          bcd : in  STD_LOGIC_VECTOR(3 downto 0);
          leds: out STD_LOGIC_VECTOR(6 downto 0));
    end component;
    

---------------
-- implementacao
---------------
 signal coisa : std_logic_vector(15 downto 0); -- 115
begin
  ula1 : ALU port map(
    x => x, y => y, zx => SW(0), nx => SW(1), zy => SW(2), ny => Sw(3), f => SW(5 downto 4), no => SW(6), zr => LEDR(0), ng => LEDR(1), saida => coisa, estouro => LEDR(2) 
  );

  seven0 : sevenseg port map(
    bcd => coisa(3 downto 0),
    leds => HEX0
  );

  seven1 : sevenseg port map(
    bcd => coisa(7 downto 4),
    leds => HEX1
  );

  seven2 : sevenseg port map(
    bcd => coisa(11 downto 8),
    leds => HEX2
  );

  seven3 : sevenseg port map(
    bcd => coisa(15 downto 12),
    leds => HEX3
  );
end rtl;


