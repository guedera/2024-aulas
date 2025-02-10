-- Elementos de Sistemas
-- developed by Luciano Soares
-- file: PC.vhd
-- date: 4/4/2017

-- Contador de 16bits
-- if (reset[t] == 1) out[t+1] = 0
-- else if (load[t] == 1)  out[t+1] = in[t]
-- else if (inc[t] == 1) out[t+1] = out[t] + 1
-- else out[t+1] = out[t]

library ieee;
use ieee.std_logic_1164.all;
use IEEE.NUMERIC_STD.ALL;

entity PC is
    port(
        clock     : in  STD_LOGIC;
        increment : in  STD_LOGIC;
        load      : in  STD_LOGIC;
        reset     : in  STD_LOGIC;
        input     : in  STD_LOGIC_VECTOR(15 downto 0);
        output    : out STD_LOGIC_VECTOR(15 downto 0)
    );
end entity;

architecture arch of PC is

  signal outputReg : std_logic_vector(15 downto 0);
  signal load_reg: std_logic;
  signal sel_mux: std_logic;

  component Inc16 is
      port(
          a   :  in STD_LOGIC_VECTOR(15 downto 0);
          q   : out STD_LOGIC_VECTOR(15 downto 0)
          );
  end component;

  component Register16 is
      port(
          clock:   in STD_LOGIC;
          input:   in STD_LOGIC_VECTOR(15 downto 0);
          load:    in STD_LOGIC;
          output: out STD_LOGIC_VECTOR(15 downto 0)
        );
    end component;

    component Mux16 is
		port (
			a:   in STD_LOGIC_VECTOR(15 downto 0);
			b:   in STD_LOGIC_VECTOR(15 downto 0);
			sel: in  STD_LOGIC;
			q:   out STD_LOGIC_VECTOR(15 downto 0)
            );
	end component;

    signal muxout_inc, muxout_load, muxout_reset, incout : std_logic_vector(15 downto 0);  -- Saída do Mux


    begin

    load_reg <= load or reset or increment;

    incmux: Mux16 port map(
        a => outputReg, --entrada de cima
        b => incout, --entrada de baixo
        sel => increment,
        q => muxout_inc
    );


    loadmux: Mux16 port map(
        a => muxout_inc,--entrada de cima
        b => input, --entrada de baixo
        sel => load,
        q => muxout_load
    );

    resetmux : Mux16 port map(
        a => muxout_load, --entrada de cima
        b => x"0000" , --entrada de baixo, recebe zero pois ira resetar tudo para zero
        sel => reset,
        q => muxout_reset
    );

    reg : Register16 port map(
        clock => clock,
        input => muxout_reset,
        load => load_reg,
        output => outputReg
    );

    inc : Inc16 port map(
        a => outputReg,
        q => incout
    );

    output <= outputReg; -- output do pc


    end architecture;