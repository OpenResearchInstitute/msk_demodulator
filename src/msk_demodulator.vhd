------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------
--  _______                             ________                                            ______
--  __  __ \________ _____ _______      ___  __ \_____ _____________ ______ ___________________  /_
--  _  / / /___  __ \_  _ \__  __ \     __  /_/ /_  _ \__  ___/_  _ \_  __ `/__  ___/_  ___/__  __ \
--  / /_/ / __  /_/ //  __/_  / / /     _  _, _/ /  __/_(__  ) /  __// /_/ / _  /    / /__  _  / / /
--  \____/  _  .___/ \___/ /_/ /_/      /_/ |_|  \___/ /____/  \___/ \__,_/  /_/     \___/  /_/ /_/
--          /_/
--                   ________                _____ _____ _____         _____
--                   ____  _/_______ __________  /____(_)__  /_____  ____  /______
--                    __  /  __  __ \__  ___/_  __/__  / _  __/_  / / /_  __/_  _ \
--                   __/ /   _  / / /_(__  ) / /_  _  /  / /_  / /_/ / / /_  /  __/
--                   /___/   /_/ /_/ /____/  \__/  /_/   \__/  \__,_/  \__/  \___/
--
------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------
-- Copyright
------------------------------------------------------------------------------------------------------
--
-- Copyright 2024 by M. Wishek <matthew@wishek.com>
--
------------------------------------------------------------------------------------------------------
-- License
------------------------------------------------------------------------------------------------------
--
-- This source describes Open Hardware and is licensed under the CERN-OHL-W v2.
--
-- You may redistribute and modify this source and make products using it under
-- the terms of the CERN-OHL-W v2 (https://ohwr.org/cern_ohl_w_v2.txt).
--
-- This source is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY, INCLUDING
-- OF MERCHANTABILITY, SATISFACTORY QUALITY AND FITNESS FOR A PARTICULAR PURPOSE.
-- Please see the CERN-OHL-W v2 for applicable conditions.
--
-- Source location: TBD
--
-- As per CERN-OHL-W v2 section 4.1, should You produce hardware based on this
-- source, You must maintain the Source Location visible on the external case of
-- the products you make using this source.
--
------------------------------------------------------------------------------------------------------
-- Block name and description
------------------------------------------------------------------------------------------------------
--
-- This block implements and MSK Demodulator.
--
-- Documentation location: TBD
--
------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------


------------------------------------------------------------------------------------------------------
-- ╦  ┬┌┐ ┬─┐┌─┐┬─┐┬┌─┐┌─┐
-- ║  │├┴┐├┬┘├─┤├┬┘│├┤ └─┐
-- ╩═╝┴└─┘┴└─┴ ┴┴└─┴└─┘└─┘
------------------------------------------------------------------------------------------------------
-- Libraries

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;


------------------------------------------------------------------------------------------------------
-- ╔═╗┌┐┌┌┬┐┬┌┬┐┬ ┬
-- ║╣ │││ │ │ │ └┬┘
-- ╚═╝┘└┘ ┴ ┴ ┴  ┴ 
------------------------------------------------------------------------------------------------------
-- Entity

ENTITY msk_demodulator IS 
	GENERIC (
		NCO_W 			: NATURAL := 32;
		PHASE_W 		: NATURAL := 10;
		SINUSOID_W 		: NATURAL := 12;
		SAMPLE_W 		: NATURAL := 12;
		DATA_W 			: NATURAL := 16;
		GAIN_W 			: NATURAL := 16
	);
	PORT (
		clk 				: IN  std_logic;
		init 				: IN  std_logic;

		rx_freq_word_f1 	: IN  std_logic_vector(NCO_W -1 DOWNTO 0);
		rx_freq_word_f2	 	: IN  std_logic_vector(NCO_W -1 DOWNTO 0);

		lpf_p_gain 			: IN  std_logic_vector(GAIN_W -1 DOWNTO 0);
		lpf_i_gain 			: IN  std_logic_vector(GAIN_W -1 DOWNTO 0);
		lpf_freeze 			: IN  std_logic;
		lpf_zero   			: IN  std_logic;
		lpf_alpha  			: IN  std_logic_vector(GAIN_W -1 DOWNTO 0);

		rx_enable 			: IN  std_logic;
		rx_svalid 			: IN  std_logic;
		rx_samples 			: IN  std_logic_vector(SAMPLE_W -1 DOWNTO 0);

		rx_data 			: OUT std_logic;
		rx_dvalid 			: OUT std_logic
	);
END ENTITY msk_demodulator;


------------------------------------------------------------------------------------------------------
-- ╔═╗┬─┐┌─┐┬ ┬┬┌┬┐┌─┐┌─┐┌┬┐┬ ┬┬─┐┌─┐
-- ╠═╣├┬┘│  ├─┤│ │ ├┤ │   │ │ │├┬┘├┤ 
-- ╩ ╩┴└─└─┘┴ ┴┴ ┴ └─┘└─┘ ┴ └─┘┴└─└─┘
------------------------------------------------------------------------------------------------------
-- Architecture

ARCHITECTURE rtl OF msk_demodulator IS 

	CONSTANT full_scale  	: unsigned(NCO_W -1 DOWNTO 0) := (OTHERS => '1');

	CONSTANT NCO_PId2 		: unsigned(NCO_W -1 DOWNTO 0) := full_scale/4;
	CONSTANT NCO_PI  		: unsigned(NCO_W -1 DOWNTO 0) := full_scale/2;
	CONSTANT NCO_3PId2		: unsigned(NCO_W -1 DOWNTO 0) := resize((full_scale*3)/2, NCO_W);
	CONSTANT NCO_2PI 		: unsigned(NCO_W -1 DOWNTO 0) := (OTHERS => '0');

	SIGNAL rx_init 			: std_logic;
	SIGNAL tclk 			: std_logic;
	SIGNAL data_f1  		: std_logic_vector(DATA_W -1 DOWNTO 0);
	SIGNAL data_f2  		: std_logic_vector(DATA_W -1 DOWNTO 0);
	SIGNAL data_f1_signed	: signed(DATA_W -1 DOWNTO 0);
	SIGNAL data_f2_signed	: signed(DATA_W -1 DOWNTO 0);
	SIGNAL data_f1_d 		: signed(DATA_W -1 DOWNTO 0);
	SIGNAL data_f2_d 		: signed(DATA_W -1 DOWNTO 0);
	SIGNAL data_f1_sum		: signed(DATA_W -1 DOWNTO 0);
	SIGNAL data_f2_sum		: signed(DATA_W -1 DOWNTO 0);
	SIGNAL data_f1_T		: signed(DATA_W -1 DOWNTO 0);
	SIGNAL data_f2_T		: signed(DATA_W -1 DOWNTO 0);
	SIGNAL data_sum 		: signed(DATA_W -1 DOWNTO 0);
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
	SIGNAL f1_f2_sel 		: std_logic;

BEGIN

	rx_init 		<= init OR NOT rx_enable;

------------------------------------------------------------------------------------------------------
--  __       ___         __   __  __  __   __   __ 
-- |  \  /\   |   /\    |  \ |_  /   /  \ |  \ |_  
-- |__/ /--\  |  /--\   |__/ |__ \__ \__/ |__/ |__ 
--                                                 
------------------------------------------------------------------------------------------------------
-- Data Decode

 	data_f1_signed 	<= signed(data_f1);
	data_f2_signed 	<= signed(NOT data_f2) + 1 WHEN cclk = '0' ELSE signed(data_f2);

	data_proc : PROCESS (clk)
	BEGIN
		IF clk'EVENT AND clk = '1' THEN

			IF rx_svalid = '1' THEN

				tclk_dly <= tclk & tclk_dly(0 TO 2);

				IF tclk = '1' THEN
	
					data_f1_T <= data_f1_signed;
					data_f2_T <= data_f2_signed;
	
				END IF;

				IF tclk_dly(0) = '1' THEN
	
					data_f1_sum <= signed(data_f1_signed) + data_f1_T;
					data_f2_sum <= signed(data_f2_signed) + data_f2_T;
	
				END IF;

				IF tclk_dly(0) = '1' THEN 
					data_bit_dly <= data_bit;
				END IF;

			END IF;

			IF rx_init = '1' THEN
				data_f1_T 		<= (OTHERS => '0');
				data_f2_T 		<= (OTHERS => '0');
				data_f1_sum		<= (OTHERS => '0');
				data_f2_sum		<= (OTHERS => '0');
				data_bit_dly 	<= '0';
			END IF;

		END IF;
	END PROCESS data_proc;

	data_sum <= signed(data_f1_sum) - signed(data_f2_sum);
	data_bit <= data_sum(DATA_W -1);

	f1_f2_sel 	<= NOT(data_bit XOR data_bit_dly);

	rx_data 	<= data_bit;
	rx_dvalid 	<= tclk_dly(1);

	error_valid_f1 <= tclk_dly(1) AND f1_f2_sel;
	error_valid_f2 <= tclk_dly(1) AND NOT f1_f2_sel;


------------------------------------------------------------------------------------------------------
--  __           __   __         __      __   __        __   __  __  __        __  __      
-- (_  \_/ |\/| |__) /  \ |     /   |   /  \ /   |_/   |__) |_  /   /  \ \  / |_  |__) \_/ 
-- __)  |  |  | |__) \__/ |__   \__ |__ \__/ \__ | \   | \  |__ \__ \__/  \/  |__ | \   |  
--                                                                                         
------------------------------------------------------------------------------------------------------
-- Symbol Clock Recovery

	tclk <= dclk XOR dclk_d;

	clock_rec_process : PROCESS (clk)
	BEGIN
		IF clk'EVENT AND clk = '1' THEN

			IF rx_svalid = '1' THEN

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

		END IF;
	END PROCESS clock_rec_process;


------------------------------------------------------------------------------------------------------
--  __  __   __ ___       __        __   __   __     __     
-- /   /  \ (_   |   /\  (_    |   /  \ /  \ |__)   |_   /| 
-- \__ \__/ __)  |  /--\ __)   |__ \__/ \__/ |      |     | 
--                                                          
------------------------------------------------------------------------------------------------------
-- Costas Loop F1

	U_f1 : ENTITY work.costas_loop(rtl)
		GENERIC MAP (
			NCO_W 			=> NCO_W,
			PHASE_W 		=> PHASE_W,
			SINUSOID_W 		=> SINUSOID_W,
			SAMPLE_W 		=> SAMPLE_W,
			DATA_W 			=> DATA_W,
			PHASE_INIT 		=> NCO_2PI
		)
		PORT MAP (
			clk 			=> clk,
			init 			=> rx_init,

			enable 			=> rx_svalid,

			tclk 			=> tclk,

			lpf_p_gain 		=> lpf_p_gain,
			lpf_i_gain 		=> lpf_i_gain,
			lpf_freeze 	 	=> lpf_freeze,
			lpf_zero 		=> lpf_zero,
			lpf_alpha 		=> lpf_alpha,

			freq_word 		=> rx_freq_word_f1,
			cos_samples 	=> rx_cos_f1,
			sin_samples 	=> rx_sin_f1,

			error_valid		=> error_valid_f1,

			rx_samples 		=> rx_samples,

			data_out 		=> data_f1
		);


------------------------------------------------------------------------------------------------------
--  __  __   __ ___       __        __   __   __     __  __  
-- /   /  \ (_   |   /\  (_    |   /  \ /  \ |__)   |_    _) 
-- \__ \__/ __)  |  /--\ __)   |__ \__/ \__/ |      |    /__ 
--                                                           
------------------------------------------------------------------------------------------------------
-- Costas Loop F2
                                                                              
	U_f2 : ENTITY work.costas_loop(rtl)
		GENERIC MAP (
			NCO_W 			=> NCO_W,
			PHASE_W 		=> PHASE_W,
			SINUSOID_W 		=> SINUSOID_W,
			SAMPLE_W 		=> SAMPLE_W,
			DATA_W 			=> DATA_W,
			PHASE_INIT 		=> NCO_2PI
		)
		PORT MAP (
			clk 			=> clk,
			init 			=> rx_init,

			enable 			=> rx_svalid,

			tclk 			=> tclk,

			lpf_p_gain 		=> lpf_p_gain,
			lpf_i_gain 		=> lpf_i_gain,
			lpf_freeze 	 	=> lpf_freeze,
			lpf_zero 		=> lpf_zero,
			lpf_alpha 		=> lpf_alpha,

			freq_word 		=> rx_freq_word_f2,
			cos_samples 	=> rx_cos_f2,
			sin_samples 	=> rx_sin_f2,

			error_valid 	=> error_valid_f2,

			rx_samples 		=> rx_samples,

			data_out 		=> data_f2
		);

END ARCHITECTURE rtl;