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
-- Copyright 2026 by M. Wishek <matthew@wishek.com>
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
-- This block provides a Lock Detector for the Costas Loop component of the MSK Demodulator.
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

ENTITY costas_lock_detect IS 
	GENERIC (
		ACC_W 			: NATURAL := 16;
		THR_W 			: NATURAL := 32;
		ICNT_W 			: NATURAL := 10;
		TCNT_W 			: NATURAL := 16
	);
	PORT (
		clk 			: IN  std_logic;
		init 			: IN  std_logic;

		tclk 			: IN  std_logic;
		acc_valid 		: IN  std_logic;

		cst_i_acc 		: IN  std_logic_vector(ACC_W/2 -1 DOWNTO 0);
		cst_q_acc 		: IN  std_logic_vector(ACC_W/2 -1 DOWNTO 0);

		cst_lock_thresh : IN  std_logic_vector(THR_W -1 DOWNTO 0);
		cst_lock_count 	: IN  std_logic_vector(ICNT_W -1 DOWNTO 0);

		cst_lock 		: OUT std_logic;
		cst_lock_time	: OUT std_logic_vector(TCNT_W -1 DOWNTO 0);
		cst_unlock 		: OUT std_logic
	);
END ENTITY costas_lock_detect;


------------------------------------------------------------------------------------------------------
-- ╔═╗┬─┐┌─┐┬ ┬┬┌┬┐┌─┐┌─┐┌┬┐┬ ┬┬─┐┌─┐
-- ╠═╣├┬┘│  ├─┤│ │ ├┤ │   │ │ │├┬┘├┤ 
-- ╩ ╩┴└─└─┘┴ ┴┴ ┴ └─┘└─┘ ┴ └─┘┴└─└─┘
------------------------------------------------------------------------------------------------------
-- Architecture

ARCHITECTURE rtl OF costas_lock_detect IS 

	SIGNAL i_sqr 		: signed(ACC_W -1 DOWNTO 0);
	SIGNAL q_sqr 		: signed(ACC_W -1 DOWNTO 0);

	SIGNAL icntr  		: unsigned(ICNT_W -1 DOWNTO 0);
	SIGNAL acc_i 		: signed(ACC_W -1 DOWNTO 0);
	SIGNAL acc_q 		: signed(ACC_W -1 DOWNTO 0);

	SIGNAL acc_iq_delta	: signed(ACC_W/2 -1 DOWNTO 0);

	SIGNAL lock 		: std_logic;
	SIGNAL lock_d 		: std_logic;
	SIGNAL tcntr 	  	: unsigned(TCNT_W -1 DOWNTO 0);
	SIGNAL lock_once 	: std_logic;

BEGIN

	cst_lock 		<= lock;
	cst_lock_time 	<= std_logic_vector(tcntr);
	cst_unlock 		<= '1' WHEN lock_d = '1' AND lock = '0' ELSE '0';

	lock_proc : PROCESS (clk)
		VARIABLE v_acc_iq_delta : signed(31 DOWNTO 0);
	BEGIN
		IF clk'EVENT AND clk = '1' THEN
			IF init = '1' THEN
				i_sqr 	<= (OTHERS => '0');
				q_sqr 	<= (OTHERS => '0');
				icntr  	<= (OTHERS => '0');
				acc_i	<= (OTHERS => '0');
				acc_q	<= (OTHERS => '0');
				tcntr 	<= (OTHERS => '0');
				lock 	<= '0';
				lock_d 	<= '0';
				lock_once <= '0';
			ELSE

				IF tclk = '1' AND acc_valid = '1' THEN

					i_sqr <= signed(cst_i_acc) * signed(cst_i_acc);
					q_sqr <= signed(cst_q_acc) * signed(cst_q_acc);

					IF icntr > 0 THEN
						acc_i <= acc_i + i_sqr;
						acc_q <= acc_q + q_sqr;
						icntr <= icntr -1;
					ELSE
						icntr 		<= unsigned(cst_lock_count);
						acc_i  		<= (OTHERS => '0');
						acc_q  		<= (OTHERS => '0');
					END IF;

				END IF;

				v_acc_iq_delta := acc_i - acc_q;

				IF icntr = 0 THEN
					acc_iq_delta <= resize(shift_right(v_acc_iq_delta, acc_iq_delta'LENGTH), acc_iq_delta'LENGTH);
				END IF;

				IF acc_iq_delta > signed(cst_lock_thresh) THEN
					lock <= '1';
				ELSE
					lock <= '0';
				END IF;

				lock_d <= lock;

				IF tclk = '1' AND lock = '0' AND lock_once = '0' THEN
					tcntr <= tcntr + 1;
				END IF;

				IF tclk = '1' AND lock = '1' THEN
					lock_once <= '1';
				END IF;

			END IF;
		END IF;
	END PROCESS lock_proc;

END ARCHITECTURE rtl;
