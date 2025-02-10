library IEEE; 
use IEEE.STD_LOGIC_1164.ALL;

entity cadeado is
	port(

		clock     : in  STD_LOGIC;
		SW      : in  std_logic_vector(9 downto 0);
		KEY     : in  std_logic_vector(3 downto 0);
		LEDR    : out std_logic_vector(9 downto 0);
		HEX0     : out std_logic_vector(6 downto 0);
		HEX1     : out std_logic_vector(6 downto 0);
		HEX2     : out std_logic_vector(6 downto 0)
		
	);
end entity;

architecture rtl of cadeado is

	signal senha    : std_logic_vector(9 downto 0);
	signal led_out    : std_logic_vector(9 downto 0);
	signal hex0_out    : std_logic_vector(6 downto 0);
	signal hex1_out    : std_logic_vector(6 downto 0);
	signal hex2_out    : std_logic_vector(6 downto 0);

begin

	process(clock)

	begin

		if rising_edge(clock) then

			if KEY(0) = '1' then

				senha <= SW;
				led_out <= (0 => '0', 1 => '0', 2 => '0', 3 => '0', 4 => '0', 5 => '0', 6 => '0', 7 => '0', 8 => '0', 9 => '0');
				hex0_out <= "1111111";
				hex1_out <= "1111111";
				hex2_out <= "1111111";

			elsif KEY(1) = '1' then

				if SW = senha then

					led_out <= SW;
					hex0_out <= "0010010";
					hex1_out <= "0000110";
					hex2_out <= "0010001";

				else

					led_out <= (0 => '0', 1 => '0', 2 => '0', 3 => '0', 4 => '0', 5 => '0', 6 => '0', 7 => '0', 8 => '0', 9 => '0');
					hex0_out <= "1111111";
					hex1_out <= "1111111";
					hex2_out <= "1111111";

				end if;

			else

				led_out <= (0 => '0', 1 => '0', 2 => '0', 3 => '0', 4 => '0', 5 => '0', 6 => '0', 7 => '0', 8 => '0', 9 => '0');
				hex0_out <= "1111111";
				hex1_out <= "1111111";
				hex2_out <= "1111111";

			end if;

		end if;

	end process;

	LEDR <= led_out;
	HEX0 <= hex0_out;
	HEX1 <= hex1_out;
	HEX2 <= hex2_out;

end;