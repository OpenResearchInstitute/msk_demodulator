
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;


ENTITY msk_demodulator IS 
	GENERIC (
		NCO_W 			: NATURAL := 32;
		PHASE_W 		: NATURAL := 10;
		SINUSOID_W 		: NATURAL := 12;
		SAMPLE_W 		: NATURAL := 12
	);
	PORT (
		clk 				: IN  std_logic;
		init 				: IN  std_logic;

		rx_freq_word_f1 	: IN  std_logic_vector(NCO_W -1 DOWNTO 0);
		rx_freq_word_f2	 	: IN  std_logic_vector(NCO_W -1 DOWNTO 0);

		rx_samples 			: IN  std_logic_vector(SAMPLE_W -1 DOWNTO 0);

		rx_data 			: OUT std_logic;
		rx_valid 			: OUT std_logic
	);
END ENTITY msk_demodulator;

ARCHITECTURE rtl OF msk_demodulator IS 

	SIGNAL tclk 			: std_logic;
	SIGNAL data_f1  		: std_logic_vector(SAMPLE_W -1 DOWNTO 0);
	SIGNAL data_f2  		: std_logic_vector(SAMPLE_W -1 DOWNTO 0);
	SIGNAL data_f1_d 		: signed(15 DOWNTO 0);
	SIGNAL data_f2_d 		: signed(15 DOWNTO 0);
	SIGNAL data_f1_sum		: signed(15 DOWNTO 0);
	SIGNAL data_f2_sum		: signed(15 DOWNTO 0);
	SIGNAL data_f1_T		: signed(15 DOWNTO 0);
	SIGNAL data_f2_T		: signed(15 DOWNTO 0);
	SIGNAL data_sum 		: signed(15 DOWNTO 0);
	SIGNAL data_bit 		: std_logic;
	SIGNAL data_bit_dly 	: std_logic;
	SIGNAL data_dec 		: std_logic;
	SIGNAL tclk_dly 		: std_logic_vector(0 TO 3);
	SIGNAL rx_cos_f1 		: std_logic_vector(SINUSOID_W -1 DOWNTO 0);
	SIGNAL rx_cos_f2 		: std_logic_vector(SINUSOID_W -1 DOWNTO 0);
	SIGNAL rx_sin_f1 		: std_logic_vector(SINUSOID_W -1 DOWNTO 0);
	SIGNAL rx_sin_f2 		: std_logic_vector(SINUSOID_W -1 DOWNTO 0);
	SIGNAL rx_cos_f1_sin_f2 : signed(2*SINUSOID_W -1 DOWNTO 0);
	SIGNAL rx_cos_f2_sin_f1 : signed(2*SINUSOID_W -1 DOWNTO 0);
	SIGNAL rx_cos_f1_cos_f2 : signed(2*SINUSOID_W -1 DOWNTO 0);
	SIGNAL rx_sin_f1_sin_f2 : signed(2*SINUSOID_W -1 DOWNTO 0);
	SIGNAL dclk_slv			: signed(2*SINUSOID_W -1 DOWNTO 0);
	SIGNAL cclk_slv			: signed(2*SINUSOID_W -1 DOWNTO 0);
	SIGNAL dclk 			: std_logic;
	SIGNAL dclk_d 			: std_logic;
	SIGNAL cclk 			: std_logic;
	SIGNAL error_valid_f1 	: std_logic;
	SIGNAL error_valid_f2 	: std_logic;

BEGIN

	data_proc : PROCESS (clk)
	BEGIN
		IF clk'EVENT AND clk = '1' THEN

			tclk_dly <= tclk & tclk_dly(0 TO 2);

			IF tclk = '1' THEN

				data_f1_d <= resize(signed(data_f1), 16);

				IF cclk = '1' THEN
					data_f2_d <= resize(signed(NOT data_f2) + 1, 16);
				ELSE
					data_f2_d <= resize(signed(data_f2), 16);
				END IF;

				data_f1_T <= data_f1_d;
				data_f2_T <= data_f2_d;

			END IF;

			IF tclk_dly(0) = '1' THEN

				data_f1_sum <= signed(data_f1_d) + data_f1_T;
				data_f2_sum <= signed(data_f2_d) - data_f2_T;

			END IF;

			IF tclk_dly(1) = '1' THEN

				data_sum <= signed(data_f1_sum) - signed(data_f2_sum);

			END IF;

			IF data_sum(SAMPLE_W +1) = '0' THEN
				data_bit <= '0';
			ELSE
				data_bit <= '1';
			END IF;

			IF init = '1' THEN
				data_sum <= (OTHERS => '0');
			END IF;

		END IF;
	END PROCESS data_proc;

	rx_data 	<= data_bit;
	rx_valid 	<= tclk_dly(3);

	error_valid_f2 <= tclk_dly(3) AND data_bit;
	error_valid_f1 <= tclk_dly(3) AND NOT data_bit;

	tclk <= dclk XOR dclk_d;

	clock_rec_process : PROCESS (clk)
	BEGIN
		IF clk'EVENT AND clk = '1' THEN

			rx_cos_f1_sin_f2 <= signed(rx_cos_f1) * signed(rx_sin_f2);
			rx_cos_f2_sin_f1 <= signed(rx_cos_f2) * signed(rx_sin_f1);
			rx_cos_f1_cos_f2 <= signed(rx_cos_f1) * signed(rx_cos_f2);
			rx_sin_f1_sin_f2 <= signed(rx_sin_f2) * signed(rx_sin_f1);

			dclk_slv <= rx_cos_f1_sin_f2 - rx_cos_f2_sin_f1;
			cclk_slv <= rx_cos_f1_cos_f2 + rx_sin_f1_sin_f2;

			dclk <= NOT dclk_slv(2*SINUSOID_W -1);
			cclk <= NOT cclk_slv(2*SINUSOID_W -1);

			dclk_d <= dclk;

		END IF;
	END PROCESS clock_rec_process;

	U_f1 : ENTITY work.costas_loop(rtl)
		GENERIC MAP (
			NCO_W 			=> NCO_W,
			PHASE_W 		=> PHASE_W,
			SINUSOID_W 		=> SINUSOID_W,
			SAMPLE_W 		=> SAMPLE_W 
		)
		PORT MAP (
			clk 			=> clk,
			init 			=> init,

			tclk 			=> tclk,

			freq_word 		=> rx_freq_word_f1,
			cos_samples 	=> rx_cos_f1,
			sin_samples 	=> rx_sin_f1,

			error_valid 	=> error_valid_f1,

			rx_samples 		=> rx_samples,

			data_out 		=> data_f1
		);

	U_f2 : ENTITY work.costas_loop(rtl)
		GENERIC MAP (
			NCO_W 			=> NCO_W,
			PHASE_W 		=> PHASE_W,
			SINUSOID_W 		=> SINUSOID_W,
			SAMPLE_W 		=> SAMPLE_W 
		)
		PORT MAP (
			clk 			=> clk,
			init 			=> init,

			tclk 			=> tclk,

			freq_word 		=> rx_freq_word_f2,
			cos_samples 	=> rx_cos_f2,
			sin_samples 	=> rx_sin_f2,

			error_valid 	=> error_valid_f2,

			rx_samples 		=> rx_samples,

			data_out 		=> data_f2
		);

END ARCHITECTURE rtl;