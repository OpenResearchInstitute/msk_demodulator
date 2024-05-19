


LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

ENTITY costas_loop IS 
	GENERIC (
		NCO_W 			: NATURAL := 32;
		PHASE_W 		: NATURAL := 10;
		SINUSOID_W 		: NATURAL := 12;
		SAMPLE_W 		: NATURAL := 12;
		ACC_W 			: NATURAL := 32;
		ERR_W 			: NATURAL := 16;
		GAIN_W  		: NATURAL := 16;
		DATA_W 			: NATURAL := 16;
		PHASE_INIT 		: UNSIGNED(NCO_W -1 DOWNTO 0) := (OTHERS => '0')
	);
	PORT (
		clk 			: IN  std_logic;
		init 			: IN  std_logic;

		tclk 			: IN  std_logic;

		lpf_p_gain 		: IN  std_logic_vector(GAIN_W -1 DOWNTO 0);
		lpf_i_gain 		: IN  std_logic_vector(GAIN_W -1 DOWNTO 0);
		lpf_freeze 		: IN  std_logic;
		lpf_zero   		: IN  std_logic;
		lpf_alpha  		: IN  std_logic_vector(GAIN_W -1 DOWNTO 0);

		freq_word 		: IN  std_logic_vector(NCO_W -1 DOWNTO 0);
		cos_samples 	: OUT std_logic_vector(SINUSOID_W -1 DOWNTO 0);
		sin_samples 	: OUT std_logic_vector(SINUSOID_W -1 DOWNTO 0);

		error_valid		: IN  std_logic;

		rx_samples 		: IN  std_logic_vector(SAMPLE_W -1 DOWNTO 0);

		data_out 		: OUT std_logic_vector(DATA_W -1 DOWNTO 0)
	);
END ENTITY costas_loop;

ARCHITECTURE rtl OF costas_loop IS 

	SIGNAL car_phase 		: std_logic_vector(NCO_W -1 DOWNTO 0);
	SIGNAL car_sin 			: std_logic_vector(SINUSOID_W -1 DOWNTO 0);
	SIGNAL car_cos 			: std_logic_vector(SINUSOID_W -1 DOWNTO 0);
	SIGNAL car_sin_d		: std_logic_vector(SINUSOID_W -1 DOWNTO 0);
	SIGNAL car_cos_d		: std_logic_vector(SINUSOID_W -1 DOWNTO 0);

	SIGNAL rx_sin			: signed(2*SINUSOID_W -1 DOWNTO 0);
	SIGNAL rx_cos			: signed(2*SINUSOID_W -1 DOWNTO 0);
	SIGNAL rx_sin_filt_sum 	: signed(ACC_W -1 DOWNTO 0);
	SIGNAL rx_cos_filt_sum 	: signed(ACC_W -1 DOWNTO 0);
	SIGNAL rx_sin_filt_acc 	: signed(ACC_W -1 DOWNTO 0);
	SIGNAL rx_cos_filt_acc 	: signed(ACC_W -1 DOWNTO 0);
	SIGNAL rx_sin_acc		: signed(ACC_W -1 DOWNTO 0);
	SIGNAL rx_cos_acc		: signed(ACC_W -1 DOWNTO 0);
	SIGNAL rx_sin_dump		: signed(ACC_W -1 DOWNTO 0);
	SIGNAL rx_cos_dump		: signed(ACC_W -1 DOWNTO 0);
	SIGNAL rx_sin_T			: signed(ACC_W -1 DOWNTO 0);
	SIGNAL rx_cos_T			: signed(ACC_W -1 DOWNTO 0);
	SIGNAL rx_sin_T_neg 	: signed(ACC_W -1 DOWNTO 0);

	SIGNAL rx_samples_d 	: std_logic_vector(SAMPLE_W -1 DOWNTO 0);

	SIGNAL rx_cos_slice 	: std_logic;
	SIGNAL rx_error 		: signed(ACC_W -1 DOWNTO 0);
	SIGNAL rx_error_w 		: signed(ERR_W -1 DOWNTO 0);

	SIGNAL tclk_d			: std_logic;

	SIGNAL lpf_adj_valid 	: std_logic;
	SIGNAL lpf_adjust 		: std_logic_vector(NCO_W -1 DOWNTO 0);

BEGIN

	mix_proc : PROCESS (clk)
	BEGIN

		IF clk'EVENT AND clk = '1' THEN

			rx_samples_d	<= rx_samples;
			car_sin_d 		<= car_sin;
			car_cos_d 		<= car_cos;

			rx_sin 			<= signed(car_sin_d) * signed(rx_samples_d);
			rx_cos 			<= signed(car_cos_d) * signed(rx_samples_d);

			IF init = '1' THEN
				rx_samples_d	<= (OTHERS => '0');
				car_sin_d 		<= (OTHERS => '0');
				car_cos_d 		<= (OTHERS => '0');
				rx_sin 			<= (OTHERS => '0');
				rx_cos 			<= (OTHERS => '0');
			END IF; 

		END IF;

	END PROCESS mix_proc;

	rx_sin_filt_sum <= shift_right(signed(rx_sin) - rx_sin_filt_acc, to_integer(signed(lpf_alpha)));
	rx_cos_filt_sum <= shift_right(signed(rx_cos) - rx_cos_filt_acc, to_integer(signed(lpf_alpha)));

	filter_proc : PROCESS (clk)
	BEGIN
		IF clk'EVENT AND clk = '1' THEN

			rx_sin_filt_acc <= rx_sin_filt_acc + rx_sin_filt_sum;
			rx_cos_filt_acc <= rx_cos_filt_acc + rx_cos_filt_sum;

			IF init = '1' THEN
				rx_sin_filt_acc <= (OTHERS => '0');
				rx_cos_filt_acc <= (OTHERS => '0');
			END IF;

		END IF;
	END PROCESS filter_proc;

	integrate_process : PROCESS (clk)
	BEGIN 

		IF clk'EVENT AND clk = '1' THEN 

			tclk_d <= tclk;

			IF tclk = '1' THEN 
				rx_sin_acc 		<= to_signed(0, ACC_W); 
				rx_cos_acc 		<= to_signed(0, ACC_W); 
				rx_sin_dump 	<= resize(shift_right(signed(rx_sin_acc), SINUSOID_W), ACC_W);
				rx_cos_dump 	<= resize(shift_right(signed(rx_cos_acc), SINUSOID_W), ACC_W);
				rx_sin_T 		<= rx_sin_dump;
				rx_sin_T_neg 	<= (NOT rx_sin_dump) + 1;
				rx_cos_T 		<= rx_cos_dump;
			ELSE
				rx_sin_acc 		<= signed(rx_sin_acc) + signed(rx_sin_filt_acc);
				rx_cos_acc 		<= signed(rx_cos_acc) + signed(rx_cos_filt_acc);
			END IF;

			IF init = '1' THEN
				rx_sin_acc 		<= (OTHERS => '0');
				rx_cos_acc 		<= (OTHERS => '0');
				rx_sin_dump 	<= (OTHERS => '0');
				rx_cos_dump 	<= (OTHERS => '0');
				rx_sin_T 		<= (OTHERS => '0');
				rx_sin_T_neg 	<= (OTHERS => '0');
				rx_cos_T 		<= (OTHERS => '0');
			END IF;

		END IF; 

	END PROCESS integrate_process;

	rx_cos_slice <= '1' WHEN rx_cos_T(ACC_W -1) = '0' ELSE -- -1
	                '0';                                        -- +1

	rx_error <= rx_sin_T WHEN rx_cos_slice = '0' ELSE    -- multiply by +1/-1
			    rx_sin_T_neg;

	rx_error_w <= resize(rx_error, ERR_W);

	data_out <= std_logic_vector(resize(shift_right(rx_cos_dump, 0), DATA_W));

	cos_samples	<= car_cos;
	sin_samples <= car_sin;

	u_loopfilter : ENTITY work.loop_filter(rtl)
	GENERIC MAP (
		NCO_W 			=> NCO_W,
		ERR_W 			=> ERR_W,
		GAIN_W  		=> GAIN_W
	)
	PORT MAP (
		clk 			=> clk,
		init 			=> init,

		lpf_p_gain 		=> lpf_p_gain,
		lpf_i_gain 		=> lpf_i_gain,
		lpf_freeze 	 	=> lpf_freeze,
		lpf_zero 		=> lpf_zero,

		lpf_err_valid 	=> error_valid,
		lpf_err 		=> std_logic_vector(rx_error_w),

		lpf_adj_valid   => lpf_adj_valid,
		lpf_adjust		=> lpf_adjust
	);

	U_carrier_nco : ENTITY work.nco(rtl)
	GENERIC MAP (
		NCO_W 			=> NCO_W,
		PHASE_INIT 		=> PHASE_INIT
	)
	PORT MAP(
		clk 			=> clk,
		init 			=> init,
	
		freq_word 		=> freq_word,

		freq_adj_zero   => '0',
		freq_adj_valid  => lpf_adj_valid,
		freq_adjust 	=> lpf_adjust,
	
		phase    		=> car_phase,
		rollover_pi2 	=> OPEN,
		rollover_pi 	=> OPEN,
		rollover_3pi2 	=> OPEN,
		rollover_2pi 	=> OPEN,
		tclk_even		=> OPEN,
		tclk_odd		=> OPEN
	);

	U_carrier_sin_cos_lut : ENTITY work.sin_cos_lut(lut_based)
	GENERIC MAP(
		PHASE_W 		=> PHASE_W,
		PHASES 			=> 2**PHASE_W,
		SINUSOID_W 		=> SINUSOID_W
	)
	PORT MAP(
		clk 			=> clk,
		init 			=> init,
	
		phase 			=> car_phase(NCO_W -1 DOWNTO NCO_W - PHASE_W),

		sin_out			=> car_sin,
		cos_out			=> car_cos
	);

END ARCHITECTURE rtl;