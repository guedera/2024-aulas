library ieee;
use ieee.std_logic_1164.all;

entity CounterDown is
    port(
        clock:  in std_logic;
        q:      out std_logic_vector(2 downto 0) := (others => '0')
    );
end entity;

architecture arch of CounterDown is

    component FlipFlopT is
        port(
            clock:  in std_logic;
            t:      in std_logic;
            q:      out std_logic := '0';
            notq:   out std_logic := '1'
        );
    end component;

    signal q0, q1, q2: std_logic;

begin

    -- Instanciação dos FlipFlops do tipo T
    ff0: FlipFlopT port map(
        clock => clock,
        t => '1',  -- T flip-flop sempre alterna
        q => q0,
        notq => open
    );

    ff1: FlipFlopT port map(
        clock => q0,
        t => '1',  -- T flip-flop sempre alterna
        q => q1,
        notq => open
    );

    ff2: FlipFlopT port map(
        clock => q1,
        t => '1',  -- T flip-flop sempre alterna
        q => q2,
        notq => open
    );

    -- Atribuição da saída
    q <= q2 & q1 & q0;

end architecture;