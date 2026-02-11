"""
Python model of msk_demodulator.vhd, costas_loop.vhd, costas_lock_detect.vhd.

Architecture overview:
  MskDemodulator:
    - Two CostasLoop instances (F1 and F2)
    - Symbol clock recovery from cross-multiplication of F1/F2 NCO outputs
    - Differential decoding: data_sum = data_f1_sum - data_f2_sum
    - Hard decision from sign bit, soft decision is full data_sum

  CostasLoop:
    - SIN/COS mixers (multiply rx_samples by NCO carrier)
    - Low-pass filter (IIR with alpha shift)
    - Integrate-and-dump (accumulate per symbol, dump on tclk)
    - Error calculation (sign(cos) * sin for Costas discriminator)
    - PI controller loop filter -> NCO frequency adjustment
    - Lock detector

  CostasLockDetect:
    - Computes I^2, Q^2 from accumulator outputs
    - Accumulates over lock_count symbols
    - Lock when I^2 - Q^2 > threshold (scaled)
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from model_utils import signed, unsigned, vhdl_saturate
from nco.model.nco import Nco
from pi_controller.model.pi_controller import PiController


class CostasLockDetect:
    """Model of costas_lock_detect.vhd."""

    def __init__(self, acc_w=16, thr_w=32, icnt_w=10, tcnt_w=16, fixed_point=False):
        self.ACC_W = acc_w
        self.THR_W = thr_w
        self.ICNT_W = icnt_w
        self.TCNT_W = tcnt_w
        self.fixed_point = fixed_point
        self.reset()

    def reset(self):
        self.i_sqr = 0
        self.q_sqr = 0
        self.icntr = 0
        self.acc_i = 0
        self.acc_q = 0
        self.acc_iq_delta = 0
        self.lock = 0
        self.lock_d = 0
        self.tcntr = 0
        self.lock_once = 0

    def step(self, tclk, acc_valid, cst_i_acc, cst_q_acc, cst_lock_thresh, cst_lock_count):
        """One clock cycle. Returns dict(locked, lock_time, unlock_pulse)."""
        AW = self.ACC_W
        half_w = AW // 2

        unlock_pulse = 1 if (self.lock_d and not self.lock) else 0

        # Squaring and accumulation
        if tclk and acc_valid:
            new_i_sqr = signed(cst_i_acc, half_w) * signed(cst_i_acc, half_w)
            new_q_sqr = signed(cst_q_acc, half_w) * signed(cst_q_acc, half_w)
            print(new_i_sqr, new_i_sqr)

            if self.icntr > 0:
                self.acc_i = signed(self.acc_i + self.i_sqr, AW)
                self.acc_q = signed(self.acc_q + self.q_sqr, AW)
                self.icntr -= 1
            else:
                self.icntr = cst_lock_count & ((1 << self.ICNT_W) - 1)
                self.acc_i = 0
                self.acc_q = 0

            self.i_sqr = signed(new_i_sqr, AW)
            self.q_sqr = signed(new_q_sqr, AW)

        # Delta calculation (when counter reaches 0)
        v_acc_iq_delta = signed(self.acc_i - self.acc_q, 32)
        if self.icntr == 0:
            self.acc_iq_delta = signed(v_acc_iq_delta >> half_w, half_w)

        # Lock decision
        if self.acc_iq_delta > signed(cst_lock_thresh, self.THR_W):
            self.lock = 1
        else:
            self.lock = 0

        self.lock_d = self.lock

        # Time counter
        if tclk and not self.lock and not self.lock_once:
            self.tcntr = unsigned(self.tcntr + 1, self.TCNT_W)

        if tclk and self.lock:
            self.lock_once = 1

        return {
            'locked': self.lock,
            'lock_time': self.tcntr,
            'unlock_pulse': unlock_pulse
        }


class CostasLoop:
    """Model of costas_loop.vhd — single-frequency Costas carrier recovery loop."""

    def __init__(self, nco_w=32, phase_w=10, sinusoid_w=12, sample_w=12,
                 acc_w=32, err_w=32, gain_w=24, shift_w=8, data_w=16,
                 phase_init=0, fixed_point=False):
        self.NCO_W = nco_w
        self.PHASE_W = phase_w
        self.SINUSOID_W = sinusoid_w
        self.SAMPLE_W = sample_w
        self.ACC_W = acc_w
        self.ERR_W = err_w
        self.GAIN_W = gain_w
        self.DATA_W = data_w
        self.fixed_point = fixed_point

        self.MAX_ACC_POS = 1 << (acc_w - 2)
        self.MAX_ACC_NEG = -(1 << (acc_w - 2))

        self.nco = Nco(nco_w=nco_w, phase_w=phase_w, sinusoid_w=sinusoid_w,
                       phase_init=phase_init, fixed_point=fixed_point)
        self.pi = PiController(nco_w=nco_w, acc_w=acc_w, err_w=err_w,
                               gain_w=gain_w, shift_w=shift_w,
                               fixed_point=fixed_point)
        self.lock_detect = CostasLockDetect(acc_w=acc_w, fixed_point=fixed_point)

        self.reset()

    def reset(self):
        AW = self.ACC_W
        SW = self.SINUSOID_W
        self.nco.reset()
        self.pi.reset()
        self.lock_detect.reset()

        # Mixer pipeline
        self.rx_samples_d = 0
        self.car_sin_d = 0
        self.car_cos_d = 0
        self.rx_sin = 0  # 2*SINUSOID_W wide
        self.rx_cos = 0

        # Low-pass filter accumulators
        self.rx_sin_filt_acc = 0
        self.rx_cos_filt_acc = 0

        # Integrate-and-dump
        self.rx_sin_acc = 0
        self.rx_cos_acc = 0
        self.rx_sin_dump = 0
        self.rx_cos_dump = 0
        self.rx_sin_T = 0
        self.rx_sin_T_neg = 0
        self.rx_cos_T = 0

        self.tclk_dly = [0, 0, 0, 0]

        # PI filter outputs
        self.lpf_adj_valid = 0
        self.lpf_adjust = 0

        # Observation registers
        self._loop_error = 0
        self._nco_adjust = 0
        self._data_out = 0

        self._car_cos = 0
        self._car_sin = 0

    def step(self, rx_samples, enable, tclk, rx_svalid, error_valid,
             freq_word, discard_rxnco,
             lpf_p_gain, lpf_i_gain, lpf_p_shift, lpf_i_shift,
             lpf_alpha, lpf_freeze, lpf_zero,
             symbol_lock_count=0, symbol_lock_threshold=0):
        """One clock cycle. Returns dict of outputs."""
        if self.fixed_point:
            return self._step_fixed(rx_samples, enable, tclk, rx_svalid, error_valid,
                                    freq_word, discard_rxnco,
                                    lpf_p_gain, lpf_i_gain, lpf_p_shift, lpf_i_shift,
                                    lpf_alpha, lpf_freeze, lpf_zero,
                                    symbol_lock_count, symbol_lock_threshold)
        else:
            return self._step_float(rx_samples, enable, tclk, rx_svalid, error_valid,
                                    freq_word, discard_rxnco,
                                    lpf_p_gain, lpf_i_gain, lpf_p_shift, lpf_i_shift,
                                    lpf_alpha, lpf_freeze, lpf_zero,
                                    symbol_lock_count, symbol_lock_threshold)

    def _step_float(self, rx_samples, enable, tclk, rx_svalid, error_valid,
                    freq_word, discard_rxnco,
                    lpf_p_gain, lpf_i_gain, lpf_p_shift, lpf_i_shift,
                    lpf_alpha, lpf_freeze, lpf_zero,
                    symbol_lock_count, symbol_lock_threshold):
        """Floating-point Costas loop — simplified behavioral model."""
        import numpy as np

        # Step NCO
        nco_out = self.nco.step(enable=enable, freq_word=freq_word)
        car_sin = nco_out['sin']
        car_cos = nco_out['cos']

        if enable:
            # Mixer
            self.rx_sin = car_sin * rx_samples
            self.rx_cos = car_cos * rx_samples

            # Simple IIR filter
            alpha = 2.0 ** (-lpf_alpha) if lpf_alpha > 0 else 1.0
            self.rx_sin_filt_acc += alpha * (self.rx_sin - self.rx_sin_filt_acc)
            self.rx_cos_filt_acc += alpha * (self.rx_cos - self.rx_cos_filt_acc)

            # Integrate-and-dump
            if tclk:
                self.rx_sin_dump = self.rx_sin_acc
                self.rx_cos_dump = self.rx_cos_acc
                self.rx_sin_T = self.rx_sin_dump
                self.rx_cos_T = self.rx_cos_dump
                self.rx_sin_acc = 0.0
                self.rx_cos_acc = 0.0
            else:
                if rx_svalid:
                    self.rx_sin_acc += self.rx_sin_filt_acc
                    self.rx_cos_acc += self.rx_cos_filt_acc

            # Error
            cos_sign = 1.0 if self.rx_cos_T >= 0 else -1.0
            error = self.rx_sin_T * cos_sign

            self._data_out = self.rx_cos_dump

        self._car_sin = car_sin
        self._car_cos = car_cos

        return {
            'data_out': self._data_out,
            'cos_samples': self._car_cos,
            'sin_samples': self._car_sin,
            'lpf_accum': self.pi.i_acc,
            'nco_adjust': 0,
            'loop_error': 0,
            'cst_lock': 0,
            'cst_lock_time': 0,
            'cst_unlock': 0,
        }

    def _step_fixed(self, rx_samples, enable, tclk, rx_svalid, error_valid,
                    freq_word, discard_rxnco,
                    lpf_p_gain, lpf_i_gain, lpf_p_shift, lpf_i_shift,
                    lpf_alpha, lpf_freeze, lpf_zero,
                    symbol_lock_count, symbol_lock_threshold):
        """Fixed-point Costas loop matching RTL pipeline."""
        AW = self.ACC_W
        SW = self.SINUSOID_W
        SampW = self.SAMPLE_W
        NW = self.NCO_W
        EW = self.ERR_W
        DW = self.DATA_W
        GW = self.GAIN_W
        MIX_W = 2 * SW  # mixer output width

        # Step PI controller (uses previous cycle's error)
        pi_out = self.pi.step(
            enable=enable,
            lpf_err=signed(self._loop_error, EW),
            lpf_err_valid=error_valid,
            lpf_p_gain=lpf_p_gain,
            lpf_i_gain=lpf_i_gain,
            lpf_p_shift=lpf_p_shift,
            lpf_i_shift=lpf_i_shift,
            lpf_freeze=lpf_freeze,
            lpf_zero=lpf_zero
        )
        self.lpf_adj_valid = pi_out['lpf_adj_valid']
        self.lpf_adjust = pi_out['lpf_adjust']

        # Step NCO with frequency adjustment from PI
        nco_out = self.nco.step(
            enable=enable,
            freq_word=freq_word,
            freq_adjust=self.lpf_adjust,
            freq_adj_valid=self.lpf_adj_valid,
            freq_adj_zero=0,
            discard_nco=discard_rxnco
        )

        car_sin = nco_out['sin']
        car_cos = nco_out['cos']
        self._car_sin = car_sin
        self._car_cos = car_cos

        if enable:
            # SIN/COS Mixers (1 cycle pipeline delay)
            new_rx_sin = signed(self.car_sin_d, SW) * signed(self.rx_samples_d, SampW)
            new_rx_cos = signed(self.car_cos_d, SW) * signed(self.rx_samples_d, SampW)

            self.rx_samples_d = rx_samples
            self.car_sin_d = car_sin
            self.car_cos_d = car_cos

            self.rx_sin = signed(new_rx_sin, MIX_W)
            self.rx_cos = signed(new_rx_cos, MIX_W)

            # Low-pass filter: IIR with arithmetic right shift by lpf_alpha
            alpha_shift = signed(lpf_alpha, GW)
            sin_diff = signed(self.rx_sin - self.rx_sin_filt_acc, AW)
            cos_diff = signed(self.rx_cos - self.rx_cos_filt_acc, AW)
            sin_filt_sum = signed(self.rx_sin_filt_acc + (sin_diff >> alpha_shift), AW)
            cos_filt_sum = signed(self.rx_cos_filt_acc + (cos_diff >> alpha_shift), AW)

            # Saturate
            self.rx_sin_filt_acc = vhdl_saturate(sin_filt_sum, AW)
            self.rx_cos_filt_acc = vhdl_saturate(cos_filt_sum, AW)

            # Integrate-and-dump
            self.tclk_dly = [tclk] + self.tclk_dly[:3]

            if tclk:
                # Dump
                self.rx_sin_dump = signed(self.rx_sin_acc >> SW, AW)
                self.rx_cos_dump = signed(self.rx_cos_acc >> SW, AW)
                self.rx_sin_T = self.rx_sin_dump
                self.rx_sin_T_neg = signed(-self.rx_sin_dump, AW)
                self.rx_cos_T = self.rx_cos_dump
                self.rx_sin_acc = 0
                self.rx_cos_acc = 0
            else:
                if rx_svalid:
                    # Accumulate with saturation
                    sin_sum = signed(self.rx_sin_acc + self.rx_sin_filt_acc, AW)
                    cos_sum = signed(self.rx_cos_acc + self.rx_cos_filt_acc, AW)
                    self.rx_sin_acc = vhdl_saturate(sin_sum, AW)
                    self.rx_cos_acc = vhdl_saturate(cos_sum, AW)

            # Error calculation
            # rx_cos_slice = '1' when cos_T MSB = '0' (positive), '0' when negative
            rx_cos_slice = 1 if (unsigned(self.rx_cos_T, AW) >> (AW - 1)) == 0 else 0
            if rx_cos_slice == 0:
                self._loop_error = signed(self.rx_sin_T, AW)
            else:
                self._loop_error = signed(self.rx_sin_T_neg, AW)

            # Data output
            self._data_out = signed(self.rx_cos_dump >> 5, DW)

        # Lock detect (uses tclk_dly[1])
        lock_out = self.lock_detect.step(
            tclk=self.tclk_dly[1] if len(self.tclk_dly) > 1 else 0,
            acc_valid=error_valid,
            cst_i_acc=self.rx_cos_T,
            cst_q_acc=self.rx_sin_T,
            cst_lock_thresh=symbol_lock_threshold,
            cst_lock_count=symbol_lock_count
        )

        # Observation registers
        if error_valid:
            obs_error = signed(self._loop_error, EW)
        else:
            obs_error = signed(self._loop_error, 32)
        if self.lpf_adj_valid:
            self._nco_adjust = self.lpf_adjust

        return {
            'data_out': self._data_out,
            'cos_samples': self._car_cos,
            'sin_samples': self._car_sin,
            'lpf_accum': pi_out['lpf_accum'],
            'nco_adjust': self._nco_adjust,
            'loop_error': self._loop_error,
            'cst_lock': lock_out['locked'],
            'cst_lock_time': lock_out['lock_time'],
            'cst_unlock': lock_out['unlock_pulse'],
        }


class MskDemodulator:
    """Model of msk_demodulator.vhd — dual Costas loop MSK demodulator."""

    def __init__(self, nco_w=32, acc_w=32, phase_w=10, sinusoid_w=12,
                 sample_w=12, data_w=16, gain_w=24, shift_w=8,
                 fixed_point=False):
        self.NCO_W = nco_w
        self.ACC_W = acc_w
        self.SINUSOID_W = sinusoid_w
        self.DATA_W = data_w
        self.fixed_point = fixed_point

        # Phase init = NCO_2PI = 0 for both loops
        self.costas_f1 = CostasLoop(
            nco_w=nco_w, phase_w=phase_w, sinusoid_w=sinusoid_w,
            sample_w=sample_w, acc_w=acc_w, gain_w=gain_w, shift_w=shift_w,
            data_w=data_w, phase_init=0, fixed_point=fixed_point
        )
        self.costas_f2 = CostasLoop(
            nco_w=nco_w, phase_w=phase_w, sinusoid_w=sinusoid_w,
            sample_w=sample_w, acc_w=acc_w, gain_w=gain_w, shift_w=shift_w,
            data_w=data_w, phase_init=0, fixed_point=fixed_point
        )

        self.reset()

    def reset(self):
        self.costas_f1.reset()
        self.costas_f2.reset()

        DW = self.DATA_W
        SW = self.SINUSOID_W

        # Symbol clock recovery
        self.rx_cos_f1_sin_f2 = 0
        self.rx_cos_f2_sin_f1 = 0
        self.rx_cos_f1_cos_f2 = 0
        self.rx_sin_f1_sin_f2 = 0
        self.dclk_slv = 0
        self.cclk_slv = 0
        self.dclk = 0
        self.cclk = 0
        self.dclk_d = 0
        self.tclk = 0

        # Data decode
        self.tclk_dly = [0, 0, 0, 0]
        self.data_f1_T = 0
        self.data_f2_T = 0
        self.data_f1_sum = 0
        self.data_f2_sum = 0
        self.data_bit_enc_t = 0

        # Outputs
        self._rx_data = 0
        self._rx_data_soft = 0
        self._rx_dvalid = 0

    def step(self, rx_samples, rx_enable, rx_svalid,
             rx_freq_word_f1, rx_freq_word_f2, discard_rxnco,
             lpf_p_gain, lpf_i_gain, lpf_p_shift, lpf_i_shift,
             lpf_alpha, lpf_freeze, lpf_zero,
             symbol_lock_count=0, symbol_lock_threshold=0):
        """One clock cycle. Returns dict of outputs."""

        rx_init = not rx_enable
        DW = self.DATA_W
        SW = self.SINUSOID_W
        MIX_W = 2 * SW

        # Symbol clock recovery from cross-products of F1/F2 NCO outputs
        cos_f1 = self.costas_f1._car_cos
        sin_f1 = self.costas_f1._car_sin
        cos_f2 = self.costas_f2._car_cos
        sin_f2 = self.costas_f2._car_sin

        if self.fixed_point:
            # Cross products
            new_cos_f1_sin_f2 = signed(cos_f1, SW) * signed(sin_f2, SW)
            new_cos_f2_sin_f1 = signed(cos_f2, SW) * signed(sin_f1, SW)
            new_cos_f1_cos_f2 = signed(cos_f1, SW) * signed(cos_f2, SW)
            new_sin_f1_sin_f2 = signed(sin_f2, SW) * signed(sin_f1, SW)

            dclk_slv = signed(self.rx_cos_f1_sin_f2 - self.rx_cos_f2_sin_f1, MIX_W)
            cclk_slv = signed(self.rx_cos_f1_cos_f2 + self.rx_sin_f1_sin_f2, MIX_W)

            self.rx_cos_f1_sin_f2 = signed(new_cos_f1_sin_f2, MIX_W)
            self.rx_cos_f2_sin_f1 = signed(new_cos_f2_sin_f1, MIX_W)
            self.rx_cos_f1_cos_f2 = signed(new_cos_f1_cos_f2, MIX_W)
            self.rx_sin_f1_sin_f2 = signed(new_sin_f1_sin_f2, MIX_W)

            self.dclk = 1 if (unsigned(dclk_slv, MIX_W) >> (MIX_W - 1)) == 0 else 0
            self.cclk = 1 if (unsigned(cclk_slv, MIX_W) >> (MIX_W - 1)) == 0 else 0
        else:
            dclk_val = cos_f1 * sin_f2 - cos_f2 * sin_f1
            cclk_val = cos_f1 * cos_f2 + sin_f1 * sin_f2
            self.dclk = 1 if dclk_val >= 0 else 0
            self.cclk = 1 if cclk_val >= 0 else 0

        # tclk = dclk XOR dclk_d
        self.tclk = self.dclk ^ self.dclk_d
        self.dclk_d = self.dclk

        # Compute error_valid signals for each Costas loop
        # These depend on decoded data bit, computed from previous cycle
        error_valid_f1 = self.tclk_dly[1] and not self._rx_data
        error_valid_f2 = self.tclk_dly[1] and self._rx_data

        # Step both Costas loops
        f1_out = self.costas_f1.step(
            rx_samples=rx_samples, enable=rx_enable, tclk=self.tclk,
            rx_svalid=rx_svalid, error_valid=error_valid_f1,
            freq_word=rx_freq_word_f1, discard_rxnco=discard_rxnco,
            lpf_p_gain=lpf_p_gain, lpf_i_gain=lpf_i_gain,
            lpf_p_shift=lpf_p_shift, lpf_i_shift=lpf_i_shift,
            lpf_alpha=lpf_alpha, lpf_freeze=lpf_freeze, lpf_zero=lpf_zero,
            symbol_lock_count=symbol_lock_count,
            symbol_lock_threshold=symbol_lock_threshold
        )
        f2_out = self.costas_f2.step(
            rx_samples=rx_samples, enable=rx_enable, tclk=self.tclk,
            rx_svalid=rx_svalid, error_valid=error_valid_f2,
            freq_word=rx_freq_word_f2, discard_rxnco=discard_rxnco,
            lpf_p_gain=lpf_p_gain, lpf_i_gain=lpf_i_gain,
            lpf_p_shift=lpf_p_shift, lpf_i_shift=lpf_i_shift,
            lpf_alpha=lpf_alpha, lpf_freeze=lpf_freeze, lpf_zero=lpf_zero,
            symbol_lock_count=symbol_lock_count,
            symbol_lock_threshold=symbol_lock_threshold
        )

        # Data decode
        if self.fixed_point:
            data_f1_signed = signed(f1_out['data_out'], DW)
            # data_f2 is conditionally negated based on cclk
            data_f2_raw = signed(f2_out['data_out'], DW)
            data_f2_signed = signed(-data_f2_raw, DW) if self.cclk == 0 else data_f2_raw
        else:
            data_f1_signed = f1_out['data_out']
            data_f2_signed = -f2_out['data_out'] if self.cclk == 0 else f2_out['data_out']

        # Decode process
        self.tclk_dly = [self.tclk] + self.tclk_dly[:3]

        if self.tclk:
            self.data_f1_T = data_f1_signed
            self.data_f2_T = data_f2_signed

        if self.tclk_dly[0]:
            if self.fixed_point:
                self.data_f1_sum = signed(data_f1_signed + self.data_f1_T, DW)
                self.data_f2_sum = signed(data_f2_signed + self.data_f2_T, DW)
            else:
                self.data_f1_sum = data_f1_signed + self.data_f1_T
                self.data_f2_sum = data_f2_signed + self.data_f2_T
            self.data_bit_enc_t_new = self.data_bit_enc_t  # will be updated below

        # data_sum = f1_sum - f2_sum
        if self.fixed_point:
            data_sum = signed(self.data_f1_sum - self.data_f2_sum, DW)
            data_bit_enc = 1 if (unsigned(data_sum, DW) >> (DW - 1)) else 0
        else:
            data_sum = self.data_f1_sum - self.data_f2_sum
            data_bit_enc = 1 if data_sum < 0 else 0

        # Differential decode
        data_bit_dec = data_bit_enc if self.data_bit_enc_t == 0 else (1 - data_bit_enc)

        if self.tclk_dly[0]:
            self.data_bit_enc_t = data_bit_enc

        self._rx_data = data_bit_dec
        # Soft decision: differentially decode by flipping sign when data_bit_enc_t=1
        if self.fixed_point:
            self._rx_data_soft = data_sum if self.data_bit_enc_t == 0 else signed(-data_sum, 16)
        else:
            self._rx_data_soft = data_sum if self.data_bit_enc_t == 0 else -data_sum
        self._rx_dvalid = self.tclk_dly[1]

        if rx_init:
            self.reset()

        return {
            'rx_data': self._rx_data,
            'rx_data_soft': self._rx_data_soft,
            'rx_dvalid': self._rx_dvalid,
            'lpf_accum_f1': f1_out['lpf_accum'],
            'lpf_accum_f2': f2_out['lpf_accum'],
            'f1_nco_adjust': f1_out['nco_adjust'],
            'f2_nco_adjust': f2_out['nco_adjust'],
            'f1_error': f1_out['loop_error'],
            'f2_error': f2_out['loop_error'],
            'cst_lock_f1': f1_out['cst_lock'],
            'cst_lock_f2': f2_out['cst_lock'],
        }


if __name__ == "__main__":
    import numpy as np

    # Simple demodulator test using floating-point mode
    demod = MskDemodulator(fixed_point=False)

    fs = 61440000
    f1 = 1200
    f2 = 1800
    baud = 4800

    fw_f1 = f1 / fs
    fw_f2 = f2 / fs

    print("MSK Demodulator floating-point test")
    print(f"  fs={fs}, f1={f1}, f2={f2}, baud={baud}")

    # Generate a simple MSK signal
    t = np.arange(1000)
    signal = np.sin(2 * np.pi * f1 * t / fs) + np.sin(2 * np.pi * f2 * t / fs)
    signal = signal / np.max(np.abs(signal))

    for i in range(100):
        out = demod.step(
            rx_samples=signal[i],
            rx_enable=1, rx_svalid=1,
            rx_freq_word_f1=fw_f1, rx_freq_word_f2=fw_f2,
            discard_rxnco=0,
            lpf_p_gain=0.1, lpf_i_gain=0.01,
            lpf_p_shift=8, lpf_i_shift=12,
            lpf_alpha=4, lpf_freeze=0, lpf_zero=0
        )
        if out['rx_dvalid'] or i < 10:
            print(f"  [{i:3d}] data={out['rx_data']}  valid={out['rx_dvalid']}  "
                  f"soft={out['rx_data_soft']:.4f}")
