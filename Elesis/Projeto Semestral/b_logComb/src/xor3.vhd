library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity xor3 is
	port ( 
		A : in  STD_LOGIC;
        B : in  STD_LOGIC;
        C : in  STD_LOGIC;
        Y : out STD_LOGIC);
end xor3;

architecture arch of xor3 is
component mux8way is 
    port (
            a:   in  STD_LOGIC;
			b:   in  STD_LOGIC;
			c:   in  STD_LOGIC;
			d:   in  STD_LOGIC;
			e:   in  STD_LOGIC;
			f:   in  STD_LOGIC;
			g:   in  STD_LOGIC;
			h:   in  STD_LOGIC;
			sel: in  STD_LOGIC_VECTOR(2 downto 0);
			q:   out STD_LOGIC);
begin

mux8way : entity work.mux8way
    Port map (
            a   => '0',  
            b   => '1',  
            c   => '1',  
            d   => '0',  
            e   => '1',  
            f   => '0',  
            g   => '0',  
            h   => '1',  
            sel => (A & B & C), -- Entradas de seleção são A, B e C
            q   => Y            -- Saída do MUX
        );

end architecture;
