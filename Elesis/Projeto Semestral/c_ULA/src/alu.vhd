-- Elementos de Sistemas
-- by Luciano Soares
-- ALU.vhd

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity ALU is
	port (

			x,y:   in STD_LOGIC_VECTOR(15 downto 0);
			zx:    in STD_LOGIC;               
			nx:    in STD_LOGIC;                 
			zy:    in STD_LOGIC;             
			ny:    in STD_LOGIC;             
			f:     in STD_LOGIC;               
			no:    in STD_LOGIC;           
			zr:    out STD_LOGIC;              
			ng:    out STD_LOGIC;                    
			saida: out STD_LOGIC_VECTOR(15 downto 0) 

	);

end entity;

architecture  rtl OF alu is

	component zerador16 is

		port(z   : in STD_LOGIC;
			 a   : in STD_LOGIC_VECTOR(15 downto 0);
			 y   : out STD_LOGIC_VECTOR(15 downto 0)
			);

	end component;

	component inversor16 is

		port(z   : in STD_LOGIC;
			 a   : in STD_LOGIC_VECTOR(15 downto 0);
			 y   : out STD_LOGIC_VECTOR(15 downto 0)
		);

	end component;

	component Add16 is

		port(
			a   :  in STD_LOGIC_VECTOR(15 downto 0);
			b   :  in STD_LOGIC_VECTOR(15 downto 0);
			q   : out STD_LOGIC_VECTOR(15 downto 0)
		);

	end component;

	component And16 is

		port (
			a:   in  STD_LOGIC_VECTOR(15 downto 0);
			b:   in  STD_LOGIC_VECTOR(15 downto 0);
			q:   out STD_LOGIC_VECTOR(15 downto 0)
		);

	end component;

	component comparador16 is

		port(
			a   : in STD_LOGIC_VECTOR(15 downto 0);
			zr   : out STD_LOGIC;
			ng   : out STD_LOGIC
    );

	end component;

	component Mux16 is

		port (
			a:   in  STD_LOGIC_VECTOR(15 downto 0);
			b:   in  STD_LOGIC_VECTOR(15 downto 0);
			sel: in  STD_LOGIC;
			q:   out STD_LOGIC_VECTOR(15 downto 0)
		);

	end component;

   SIGNAL zxout,zyout,nxout,nyout,andout,adderout,muxout,precomp: std_logic_vector(15 downto 0);

begin

	zerador_roxo: zerador16 
	port map(
		z => zx,
		a => x,
		y => zxout
		);


	zerador_cinza: zerador16 
	port map(
		z => zy,
		a => y,
		y => zyout
		);


	inversor_preto: inversor16 
	port map(
		z => nx,
		a => zxout,
		y => nxout
		);


	inversor_branco: inversor16 
	port map(
		z => ny,
		a => zyout,
		y => nyout
		);


	and_vermelho: And16 
	port map(
		a => nxout,
		b => nyout,
		q => andout
		);


	add_azul: Add16 
	port map(
		a => nxout,
		b => nyout,
		q => adderout
		);


	mux_verde: Mux16 
	port map(
		a => andout,
		b => adderout,
		sel => f,
		q => muxout
		);
	

	inversor_amarelo: inversor16 
	port map(
		a => muxout,
		z => no,
		y => precomp
		);


	comparador_laranja: comparador16 
	port map(
		a => precomp,
		zr => zr,
		ng => ng
		);

	saida <= precomp;

end architecture;