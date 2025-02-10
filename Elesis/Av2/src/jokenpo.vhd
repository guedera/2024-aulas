-- jokenpo.vhd
-- Q1

Library ieee;
use ieee.std_logic_1164.all;

entity jokenpo is
	port(
		x,y:      in STD_LOGIC_VECTOR(1 downto 0);
    	ganhador:      out STD_LOGIC_VECTOR(1 downto 0)
	);
end entity;

architecture rtl of jokenpo is

begin

    process(x, y)

    begin

        if x = y then

            ganhador <= "11";

        elsif (x = "00" and y = "10") or (x = "01" and y = "00") or (x = "10" and y = "01") then

            ganhador <= "10";

        else

            ganhador <= "01";

        end if;

    end process;
    
end architecture;
