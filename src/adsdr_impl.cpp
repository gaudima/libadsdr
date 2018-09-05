/*
 * Copyright 2017 by Lukas Lao Beyer <lukas@electronics.kitchen>
 *
 * This file is part of libfreesrp.
 *
 * libfreesrp is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * libfreesrp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with libfreesrp.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <adsdr.hpp>
#include <cstring>

#include <fstream>
#include <iostream>
#include "adsdr_impl.h"


#define ADSDR_SERIAL_DSCR_INDEX 3
#define MAX_SERIAL_LENGTH 256


using namespace ADSDR;

moodycamel::ReaderWriterQueue<sample> ADSDR_impl::_rx_buf(ADSDR_RX_TX_QUEUE_SIZE);
moodycamel::ReaderWriterQueue<sample> ADSDR_impl::_tx_buf(ADSDR_RX_TX_QUEUE_SIZE);
std::vector<sample> ADSDR_impl::_rx_decoder_buf(ADSDR_RX_TX_BUF_SIZE / ADSDR_BYTES_PER_SAMPLE);
std::function<void(const std::vector<sample> &)> ADSDR_impl::_rx_custom_callback;
std::vector<sample> ADSDR_impl::_tx_encoder_buf(ADSDR_RX_TX_BUF_SIZE / ADSDR_BYTES_PER_SAMPLE);
std::function<void(std::vector<sample> &)> ADSDR_impl::_tx_custom_callback;

ADSDR_impl::ADSDR_impl(std::string serial_number)
{
    ad_default_param = {
        /* Device selection */
        ID_AD9361,	// dev_sel
        /* Identification number */
        0,		//id_no
        /* Reference Clock */
        40000000UL,	//reference_clk_rate
        /* Base Configuration */
        1,		//two_rx_two_tx_mode_enable *** adi,2rx-2tx-mode-enable
        1,		//one_rx_one_tx_mode_use_rx_num *** adi,1rx-1tx-mode-use-rx-num
        1,		//one_rx_one_tx_mode_use_tx_num *** adi,1rx-1tx-mode-use-tx-num
        1,		//frequency_division_duplex_mode_enable *** adi,frequency-division-duplex-mode-enable
        0,		//frequency_division_duplex_independent_mode_enable *** adi,frequency-division-duplex-independent-mode-enable
        0,		//tdd_use_dual_synth_mode_enable *** adi,tdd-use-dual-synth-mode-enable
        0,		//tdd_skip_vco_cal_enable *** adi,tdd-skip-vco-cal-enable
        0,		//tx_fastlock_delay_ns *** adi,tx-fastlock-delay-ns
        0,		//rx_fastlock_delay_ns *** adi,rx-fastlock-delay-ns
        0,		//rx_fastlock_pincontrol_enable *** adi,rx-fastlock-pincontrol-enable
        0,		//tx_fastlock_pincontrol_enable *** adi,tx-fastlock-pincontrol-enable
        0,		//external_rx_lo_enable *** adi,external-rx-lo-enable
        0,		//external_tx_lo_enable *** adi,external-tx-lo-enable
        5,		//dc_offset_tracking_update_event_mask *** adi,dc-offset-tracking-update-event-mask
        6,		//dc_offset_attenuation_high_range *** adi,dc-offset-attenuation-high-range
        5,		//dc_offset_attenuation_low_range *** adi,dc-offset-attenuation-low-range
        0x28,	//dc_offset_count_high_range *** adi,dc-offset-count-high-range
        0x32,	//dc_offset_count_low_range *** adi,dc-offset-count-low-range
        0,		//split_gain_table_mode_enable *** adi,split-gain-table-mode-enable
        MAX_SYNTH_FREF,	//trx_synthesizer_target_fref_overwrite_hz *** adi,trx-synthesizer-target-fref-overwrite-hz
        0,		// qec_tracking_slow_mode_enable *** adi,qec-tracking-slow-mode-enable
        /* ENSM Control */
        0,		//ensm_enable_pin_pulse_mode_enable *** adi,ensm-enable-pin-pulse-mode-enable
        0,		//ensm_enable_txnrx_control_enable *** adi,ensm-enable-txnrx-control-enable
        /* LO Control */
        2400000000UL,	//rx_synthesizer_frequency_hz *** adi,rx-synthesizer-frequency-hz
        2400000000UL,	//tx_synthesizer_frequency_hz *** adi,tx-synthesizer-frequency-hz
        1,				//tx_lo_powerdown_managed_enable *** adi,tx-lo-powerdown-managed-enable
        /* Rate & BW Control */
        {983040000, 245760000, 122880000, 61440000, 30720000, 30720000},// rx_path_clock_frequencies[6] *** adi,rx-path-clock-frequencies
        {983040000, 122880000, 122880000, 61440000, 30720000, 30720000},// tx_path_clock_frequencies[6] *** adi,tx-path-clock-frequencies
        18000000,//rf_rx_bandwidth_hz *** adi,rf-rx-bandwidth-hz
        18000000,//rf_tx_bandwidth_hz *** adi,rf-tx-bandwidth-hz
        /* RF Port Control */
        0,		//rx_rf_port_input_select *** adi,rx-rf-port-input-select
        0,		//tx_rf_port_input_select *** adi,tx-rf-port-input-select
        /* TX Attenuation Control */
        10000,	//tx_attenuation_mdB *** adi,tx-attenuation-mdB
        0,		//update_tx_gain_in_alert_enable *** adi,update-tx-gain-in-alert-enable
        /* Reference Clock Control */
        0,		//xo_disable_use_ext_refclk_enable *** adi,xo-disable-use-ext-refclk-enable
        {8, 5920},	//dcxo_coarse_and_fine_tune[2] *** adi,dcxo-coarse-and-fine-tune
        CLKOUT_DISABLE,	//clk_output_mode_select *** adi,clk-output-mode-select
        /* Gain Control */
        2,		//gc_rx1_mode *** adi,gc-rx1-mode
        2,		//gc_rx2_mode *** adi,gc-rx2-mode
        58,		//gc_adc_large_overload_thresh *** adi,gc-adc-large-overload-thresh
        4,		//gc_adc_ovr_sample_size *** adi,gc-adc-ovr-sample-size
        47,		//gc_adc_small_overload_thresh *** adi,gc-adc-small-overload-thresh
        8192,	//gc_dec_pow_measurement_duration *** adi,gc-dec-pow-measurement-duration
        0,		//gc_dig_gain_enable *** adi,gc-dig-gain-enable
        800,	//gc_lmt_overload_high_thresh *** adi,gc-lmt-overload-high-thresh
        704,	//gc_lmt_overload_low_thresh *** adi,gc-lmt-overload-low-thresh
        24,		//gc_low_power_thresh *** adi,gc-low-power-thresh
        15,		//gc_max_dig_gain *** adi,gc-max-dig-gain
        /* Gain MGC Control */
        2,		//mgc_dec_gain_step *** adi,mgc-dec-gain-step
        2,		//mgc_inc_gain_step *** adi,mgc-inc-gain-step
        0,		//mgc_rx1_ctrl_inp_enable *** adi,mgc-rx1-ctrl-inp-enable
        0,		//mgc_rx2_ctrl_inp_enable *** adi,mgc-rx2-ctrl-inp-enable
        0,		//mgc_split_table_ctrl_inp_gain_mode *** adi,mgc-split-table-ctrl-inp-gain-mode
        /* Gain AGC Control */
        10,		//agc_adc_large_overload_exceed_counter *** adi,agc-adc-large-overload-exceed-counter
        2,		//agc_adc_large_overload_inc_steps *** adi,agc-adc-large-overload-inc-steps
        0,		//agc_adc_lmt_small_overload_prevent_gain_inc_enable *** adi,agc-adc-lmt-small-overload-prevent-gain-inc-enable
        10,		//agc_adc_small_overload_exceed_counter *** adi,agc-adc-small-overload-exceed-counter
        4,		//agc_dig_gain_step_size *** adi,agc-dig-gain-step-size
        3,		//agc_dig_saturation_exceed_counter *** adi,agc-dig-saturation-exceed-counter
        1000,	// agc_gain_update_interval_us *** adi,agc-gain-update-interval-us
        0,		//agc_immed_gain_change_if_large_adc_overload_enable *** adi,agc-immed-gain-change-if-large-adc-overload-enable
        0,		//agc_immed_gain_change_if_large_lmt_overload_enable *** adi,agc-immed-gain-change-if-large-lmt-overload-enable
        10,		//agc_inner_thresh_high *** adi,agc-inner-thresh-high
        1,		//agc_inner_thresh_high_dec_steps *** adi,agc-inner-thresh-high-dec-steps
        12,		//agc_inner_thresh_low *** adi,agc-inner-thresh-low
        1,		//agc_inner_thresh_low_inc_steps *** adi,agc-inner-thresh-low-inc-steps
        10,		//agc_lmt_overload_large_exceed_counter *** adi,agc-lmt-overload-large-exceed-counter
        2,		//agc_lmt_overload_large_inc_steps *** adi,agc-lmt-overload-large-inc-steps
        10,		//agc_lmt_overload_small_exceed_counter *** adi,agc-lmt-overload-small-exceed-counter
        5,		//agc_outer_thresh_high *** adi,agc-outer-thresh-high
        2,		//agc_outer_thresh_high_dec_steps *** adi,agc-outer-thresh-high-dec-steps
        18,		//agc_outer_thresh_low *** adi,agc-outer-thresh-low
        2,		//agc_outer_thresh_low_inc_steps *** adi,agc-outer-thresh-low-inc-steps
        1,		//agc_attack_delay_extra_margin_us; *** adi,agc-attack-delay-extra-margin-us
        0,		//agc_sync_for_gain_counter_enable *** adi,agc-sync-for-gain-counter-enable
        /* Fast AGC */
        64,		//fagc_dec_pow_measuremnt_duration ***  adi,fagc-dec-pow-measurement-duration
        260,	//fagc_state_wait_time_ns ***  adi,fagc-state-wait-time-ns
        /* Fast AGC - Low Power */
        0,		//fagc_allow_agc_gain_increase ***  adi,fagc-allow-agc-gain-increase-enable
        5,		//fagc_lp_thresh_increment_time ***  adi,fagc-lp-thresh-increment-time
        1,		//fagc_lp_thresh_increment_steps ***  adi,fagc-lp-thresh-increment-steps
        /* Fast AGC - Lock Level (Lock Level is set via slow AGC inner high threshold) */
        1,		//fagc_lock_level_lmt_gain_increase_en ***  adi,fagc-lock-level-lmt-gain-increase-enable
        5,		//fagc_lock_level_gain_increase_upper_limit ***  adi,fagc-lock-level-gain-increase-upper-limit
        /* Fast AGC - Peak Detectors and Final Settling */
        1,		//fagc_lpf_final_settling_steps ***  adi,fagc-lpf-final-settling-steps
        1,		//fagc_lmt_final_settling_steps ***  adi,fagc-lmt-final-settling-steps
        3,		//fagc_final_overrange_count ***  adi,fagc-final-overrange-count
        /* Fast AGC - Final Power Test */
        0,		//fagc_gain_increase_after_gain_lock_en ***  adi,fagc-gain-increase-after-gain-lock-enable
        /* Fast AGC - Unlocking the Gain */
        0,		//fagc_gain_index_type_after_exit_rx_mode ***  adi,fagc-gain-index-type-after-exit-rx-mode
        1,		//fagc_use_last_lock_level_for_set_gain_en ***  adi,fagc-use-last-lock-level-for-set-gain-enable
        1,		//fagc_rst_gla_stronger_sig_thresh_exceeded_en ***  adi,fagc-rst-gla-stronger-sig-thresh-exceeded-enable
        5,		//fagc_optimized_gain_offset ***  adi,fagc-optimized-gain-offset
        10,		//fagc_rst_gla_stronger_sig_thresh_above_ll ***  adi,fagc-rst-gla-stronger-sig-thresh-above-ll
        1,		//fagc_rst_gla_engergy_lost_sig_thresh_exceeded_en ***  adi,fagc-rst-gla-engergy-lost-sig-thresh-exceeded-enable
        1,		//fagc_rst_gla_engergy_lost_goto_optim_gain_en ***  adi,fagc-rst-gla-engergy-lost-goto-optim-gain-enable
        10,		//fagc_rst_gla_engergy_lost_sig_thresh_below_ll ***  adi,fagc-rst-gla-engergy-lost-sig-thresh-below-ll
        8,		//fagc_energy_lost_stronger_sig_gain_lock_exit_cnt ***  adi,fagc-energy-lost-stronger-sig-gain-lock-exit-cnt
        1,		//fagc_rst_gla_large_adc_overload_en ***  adi,fagc-rst-gla-large-adc-overload-enable
        1,		//fagc_rst_gla_large_lmt_overload_en ***  adi,fagc-rst-gla-large-lmt-overload-enable
        0,		//fagc_rst_gla_en_agc_pulled_high_en ***  adi,fagc-rst-gla-en-agc-pulled-high-enable
        0,		//fagc_rst_gla_if_en_agc_pulled_high_mode ***  adi,fagc-rst-gla-if-en-agc-pulled-high-mode
        64,		//fagc_power_measurement_duration_in_state5 ***  adi,fagc-power-measurement-duration-in-state5
        /* RSSI Control */
        1,		//rssi_delay *** adi,rssi-delay
        1000,	//rssi_duration *** adi,rssi-duration
        3,		//rssi_restart_mode *** adi,rssi-restart-mode
        0,		//rssi_unit_is_rx_samples_enable *** adi,rssi-unit-is-rx-samples-enable
        1,		//rssi_wait *** adi,rssi-wait
        /* Aux ADC Control */
        256,	//aux_adc_decimation *** adi,aux-adc-decimation
        40000000UL,	//aux_adc_rate *** adi,aux-adc-rate
        /* AuxDAC Control */
        1,		//aux_dac_manual_mode_enable ***  adi,aux-dac-manual-mode-enable
        0,		//aux_dac1_default_value_mV ***  adi,aux-dac1-default-value-mV
        0,		//aux_dac1_active_in_rx_enable ***  adi,aux-dac1-active-in-rx-enable
        0,		//aux_dac1_active_in_tx_enable ***  adi,aux-dac1-active-in-tx-enable
        0,		//aux_dac1_active_in_alert_enable ***  adi,aux-dac1-active-in-alert-enable
        0,		//aux_dac1_rx_delay_us ***  adi,aux-dac1-rx-delay-us
        0,		//aux_dac1_tx_delay_us ***  adi,aux-dac1-tx-delay-us
        0,		//aux_dac2_default_value_mV ***  adi,aux-dac2-default-value-mV
        0,		//aux_dac2_active_in_rx_enable ***  adi,aux-dac2-active-in-rx-enable
        0,		//aux_dac2_active_in_tx_enable ***  adi,aux-dac2-active-in-tx-enable
        0,		//aux_dac2_active_in_alert_enable ***  adi,aux-dac2-active-in-alert-enable
        0,		//aux_dac2_rx_delay_us ***  adi,aux-dac2-rx-delay-us
        0,		//aux_dac2_tx_delay_us ***  adi,aux-dac2-tx-delay-us
        /* Temperature Sensor Control */
        256,	//temp_sense_decimation *** adi,temp-sense-decimation
        1000,	//temp_sense_measurement_interval_ms *** adi,temp-sense-measurement-interval-ms
        (int8_t)0xCE,	//temp_sense_offset_signed *** adi,temp-sense-offset-signed
        1,		//temp_sense_periodic_measurement_enable *** adi,temp-sense-periodic-measurement-enable
        /* Control Out Setup */
        0xFF,	//ctrl_outs_enable_mask *** adi,ctrl-outs-enable-mask
        0,		//ctrl_outs_index *** adi,ctrl-outs-index
        /* External LNA Control */
        0,		//elna_settling_delay_ns *** adi,elna-settling-delay-ns
        0,		//elna_gain_mdB *** adi,elna-gain-mdB
        0,		//elna_bypass_loss_mdB *** adi,elna-bypass-loss-mdB
        0,		//elna_rx1_gpo0_control_enable *** adi,elna-rx1-gpo0-control-enable
        0,		//elna_rx2_gpo1_control_enable *** adi,elna-rx2-gpo1-control-enable
        0,		//elna_gaintable_all_index_enable *** adi,elna-gaintable-all-index-enable
        /* Digital Interface Control */
        0,		//digital_interface_tune_skip_mode *** adi,digital-interface-tune-skip-mode
        0,		//digital_interface_tune_fir_disable *** adi,digital-interface-tune-fir-disable
        1,		//pp_tx_swap_enable *** adi,pp-tx-swap-enable
        1,		//pp_rx_swap_enable *** adi,pp-rx-swap-enable
        0,		//tx_channel_swap_enable *** adi,tx-channel-swap-enable
        0,		//rx_channel_swap_enable *** adi,rx-channel-swap-enable
        1,		//rx_frame_pulse_mode_enable *** adi,rx-frame-pulse-mode-enable
        0,		//two_t_two_r_timing_enable *** adi,2t2r-timing-enable
        0,		//invert_data_bus_enable *** adi,invert-data-bus-enable
        0,		//invert_data_clk_enable *** adi,invert-data-clk-enable
        0,		//fdd_alt_word_order_enable *** adi,fdd-alt-word-order-enable
        0,		//invert_rx_frame_enable *** adi,invert-rx-frame-enable
        0,		//fdd_rx_rate_2tx_enable *** adi,fdd-rx-rate-2tx-enable
        0,		//swap_ports_enable *** adi,swap-ports-enable
        1,		//single_data_rate_enable *** adi,single-data-rate-enable
        0,		//lvds_mode_enable *** adi,lvds-mode-enable
        0,		//half_duplex_mode_enable *** adi,half-duplex-mode-enable
        0,		//single_port_mode_enable *** adi,single-port-mode-enable
        1,		//full_port_enable *** adi,full-port-enable
        0,		//full_duplex_swap_bits_enable *** adi,full-duplex-swap-bits-enable
        0,		//delay_rx_data *** adi,delay-rx-data
        0,		//rx_data_clock_delay *** adi,rx-data-clock-delay
        4,		//rx_data_delay *** adi,rx-data-delay
        7,		//tx_fb_clock_delay *** adi,tx-fb-clock-delay
        0,		//tx_data_delay *** adi,tx-data-delay
        150,	//lvds_bias_mV *** adi,lvds-bias-mV
        1,		//lvds_rx_onchip_termination_enable *** adi,lvds-rx-onchip-termination-enable
        0,		//rx1rx2_phase_inversion_en *** adi,rx1-rx2-phase-inversion-enable
        0xFF,	//lvds_invert1_control *** adi,lvds-invert1-control
        0x0F,	//lvds_invert2_control *** adi,lvds-invert2-control
        /* GPO Control */
        0,		//gpo0_inactive_state_high_enable *** adi,gpo0-inactive-state-high-enable
        0,		//gpo1_inactive_state_high_enable *** adi,gpo1-inactive-state-high-enable
        0,		//gpo2_inactive_state_high_enable *** adi,gpo2-inactive-state-high-enable
        0,		//gpo3_inactive_state_high_enable *** adi,gpo3-inactive-state-high-enable
        0,		//gpo0_slave_rx_enable *** adi,gpo0-slave-rx-enable
        0,		//gpo0_slave_tx_enable *** adi,gpo0-slave-tx-enable
        0,		//gpo1_slave_rx_enable *** adi,gpo1-slave-rx-enable
        0,		//gpo1_slave_tx_enable *** adi,gpo1-slave-tx-enable
        0,		//gpo2_slave_rx_enable *** adi,gpo2-slave-rx-enable
        0,		//gpo2_slave_tx_enable *** adi,gpo2-slave-tx-enable
        0,		//gpo3_slave_rx_enable *** adi,gpo3-slave-rx-enable
        0,		//gpo3_slave_tx_enable *** adi,gpo3-slave-tx-enable
        0,		//gpo0_rx_delay_us *** adi,gpo0-rx-delay-us
        0,		//gpo0_tx_delay_us *** adi,gpo0-tx-delay-us
        0,		//gpo1_rx_delay_us *** adi,gpo1-rx-delay-us
        0,		//gpo1_tx_delay_us *** adi,gpo1-tx-delay-us
        0,		//gpo2_rx_delay_us *** adi,gpo2-rx-delay-us
        0,		//gpo2_tx_delay_us *** adi,gpo2-tx-delay-us
        0,		//gpo3_rx_delay_us *** adi,gpo3-rx-delay-us
        0,		//gpo3_tx_delay_us *** adi,gpo3-tx-delay-us
        /* Tx Monitor Control */
        37000,	//low_high_gain_threshold_mdB *** adi,txmon-low-high-thresh
        0,		//low_gain_dB *** adi,txmon-low-gain
        24,		//high_gain_dB *** adi,txmon-high-gain
        0,		//tx_mon_track_en *** adi,txmon-dc-tracking-enable
        0,		//one_shot_mode_en *** adi,txmon-one-shot-mode-enable
        511,	//tx_mon_delay *** adi,txmon-delay
        8192,	//tx_mon_duration *** adi,txmon-duration
        2,		//tx1_mon_front_end_gain *** adi,txmon-1-front-end-gain
        2,		//tx2_mon_front_end_gain *** adi,txmon-2-front-end-gain
        48,		//tx1_mon_lo_cm *** adi,txmon-1-lo-cm
        48,		//tx2_mon_lo_cm *** adi,txmon-2-lo-cm
        /* GPIO definitions */
        GPIO_RESET_PIN,		//gpio_resetb *** reset-gpios
        /* MCS Sync */
        -1,		//gpio_sync *** sync-gpios
        -1,		//gpio_cal_sw1 *** cal-sw1-gpios
        -1,		//gpio_cal_sw2 *** cal-sw2-gpios
        /* External LO clocks */
        NULL,	//(*ad9361_rfpll_ext_recalc_rate)()
        NULL,	//(*ad9361_rfpll_ext_round_rate)()
        NULL	//(*ad9361_rfpll_ext_set_rate)()
    };

    rx_fir_config = {	// BPF PASSBAND 3/20 fs to 1/4 fs
        3, // rx
        0, // rx_gain
        1, // rx_dec
        {-4, -6, -37, 35, 186, 86, -284, -315,
         107, 219, -4, 271, 558, -307, -1182, -356,
         658, 157, 207, 1648, 790, -2525, -2553, 748,
         865, -476, 3737, 6560, -3583, -14731, -5278, 14819,
         14819, -5278, -14731, -3583, 6560, 3737, -476, 865,
         748, -2553, -2525, 790, 1648, 207, 157, 658,
         -356, -1182, -307, 558, 271, -4, 219, 107,
         -315, -284, 86, 186, 35, -37, -6, -4,
         0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0}, // rx_coef[128]
        64, // rx_coef_size
        {0, 0, 0, 0, 0, 0}, //rx_path_clks[6]
        0 // rx_bandwidth
    };

    tx_fir_config = {	// BPF PASSBAND 3/20 fs to 1/4 fs
        3, // tx
        -6, // tx_gain
        1, // tx_int
        {-4, -6, -37, 35, 186, 86, -284, -315,
         107, 219, -4, 271, 558, -307, -1182, -356,
         658, 157, 207, 1648, 790, -2525, -2553, 748,
         865, -476, 3737, 6560, -3583, -14731, -5278, 14819,
         14819, -5278, -14731, -3583, 6560, 3737, -476, 865,
         748, -2553, -2525, 790, 1648, 207, 157, 658,
         -356, -1182, -307, 558, 271, -4, 219, 107,
         -315, -284, 86, 186, 35, -37, -6, -4,
         0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0}, // tx_coef[128]
        64, // tx_coef_size
        {0, 0, 0, 0, 0, 0}, // tx_path_clks[6]
        0 // tx_bandwidth
    };

    //----------------------------------------------------
    m_cmd_list.push_back(&ADSDR_impl::get_tx_lo_freq);
    m_cmd_list.push_back(&ADSDR_impl::set_tx_lo_freq);
    m_cmd_list.push_back(&ADSDR_impl::get_tx_samp_freq);
    m_cmd_list.push_back(&ADSDR_impl::set_tx_samp_freq);
    m_cmd_list.push_back(&ADSDR_impl::get_tx_rf_bandwidth);
    m_cmd_list.push_back(&ADSDR_impl::set_tx_rf_bandwidth);
    m_cmd_list.push_back(&ADSDR_impl::get_tx_attenuation);
    m_cmd_list.push_back(&ADSDR_impl::set_tx_attenuation);
    m_cmd_list.push_back(&ADSDR_impl::get_tx_fir_en);
    m_cmd_list.push_back(&ADSDR_impl::set_tx_fir_en);
    m_cmd_list.push_back(&ADSDR_impl::get_rx_lo_freq);
    m_cmd_list.push_back(&ADSDR_impl::set_rx_lo_freq);
    m_cmd_list.push_back(&ADSDR_impl::get_rx_samp_freq);
    m_cmd_list.push_back(&ADSDR_impl::set_rx_samp_freq);
    m_cmd_list.push_back(&ADSDR_impl::get_rx_rf_bandwidth);
    m_cmd_list.push_back(&ADSDR_impl::set_rx_rf_bandwidth);
    m_cmd_list.push_back(&ADSDR_impl::get_rx_gc_mode);
    m_cmd_list.push_back(&ADSDR_impl::set_rx_gc_mode);
    m_cmd_list.push_back(&ADSDR_impl::get_rx_rf_gain);
    m_cmd_list.push_back(&ADSDR_impl::set_rx_rf_gain);
    m_cmd_list.push_back(&ADSDR_impl::get_rx_fir_en);
    m_cmd_list.push_back(&ADSDR_impl::set_rx_fir_en);
    m_cmd_list.push_back(&ADSDR_impl::set_datapath_en);
    m_cmd_list.push_back(&ADSDR_impl::get_version);
    m_cmd_list.push_back(&ADSDR_impl::set_loopback_en);
    // ---------------------------------------------------

    libusb_device **devs;

    int ret = libusb_init(&_ctx);

    if(ret < 0)
    {
        throw ConnectionError("libusb init error %d: error " + std::to_string(ret));
    }

    // Set verbosity level
    libusb_set_debug(_ctx, 3);

    // Retrieve device list
    int num_devs = (int) libusb_get_device_list(_ctx, &devs);
    if(num_devs < 0)
    {
        throw ConnectionError("libusb device list retrieval error");
    }

    // Find ADSDR device
    bool no_match = false;
    
    for(int i = 0; i < num_devs; i++)
    {
        libusb_device_descriptor desc;
        int ret = libusb_get_device_descriptor(devs[i], &desc);
        if(ret < 0)
        {
            throw ConnectionError("libusb error getting device descriptor %d: error " + std::to_string(ret));
        }

        if(desc.idVendor == ADSDR_VENDOR_ID && desc.idProduct == ADSDR_PRODUCT_ID)
        {
            int ret = libusb_open(devs[i], &_adsdr_handle);
            if(ret != 0)
            {
                throw ConnectionError("libusb could not open found ADSDR USB device %d: error " + std::to_string(ret));
            }


            // Check if correct serial number
            if(desc.iSerialNumber)
            {
                char serial_num_buf[MAX_SERIAL_LENGTH];
                ret = libusb_get_string_descriptor_ascii(_adsdr_handle, /*ADSDR_SERIAL_DSCR_INDEX*/desc.iSerialNumber, (unsigned char*)serial_num_buf, MAX_SERIAL_LENGTH);
                if(ret < 0)
                {
                    libusb_close(_adsdr_handle);
                    _adsdr_handle = nullptr;
                    throw ConnectionError("1)libusb could not read ADSDR serial number %d: error " + std::to_string(ret) + "___" + std::to_string(desc.iSerialNumber));
                }
                else
                {
                    std::string dev_serial = std::string(serial_num_buf);
                    if(dev_serial.find(serial_number) != std::string::npos)
                    {
                        // Found!
                        break;
                    }
                    else
                    {
                        no_match = true;
                        libusb_close(_adsdr_handle);
                        _adsdr_handle = nullptr;
                    }
                }
            }
            else
            {
                std::cout << "ADSDR serial number not found !" << std::endl;
            }
        }
    }

    if(no_match && _adsdr_handle == nullptr)
    {
        throw ConnectionError("ADSDR device(s) were found, but did not match specified serial number");
    }
    
    if(_adsdr_handle == nullptr)
    {
        throw ConnectionError("no ADSDR device found");
    }

    // Free the list, unref the devices in it
    libusb_free_device_list(devs, 1);

    // Found a ADSDR device and opened it. Now claim its interface (ID 0).
    ret = libusb_claim_interface(_adsdr_handle, 0);
    if(ret < 0)
    {
        throw ConnectionError("could not claim ADSDR interface");
    }

    // Request ADSDR version number
#if 0
    std::array<unsigned char, ADSDR_USB_CTRL_SIZE> data{};
    ret = libusb_control_transfer(_adsdr_handle, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_IN, ADSDR_GET_VERSION_REQ, 0, 0, data.data(), (uint16_t) data.size(), ADSDR_USB_TIMEOUT);
    if(ret < 0)
    {
        throw ConnectionError("1) ADSDR not responding %d: error " + std::to_string(ret));
    }
    int transferred = ret;
    _fx3_fw_version = std::string(std::begin(data), std::begin(data) + transferred);
#endif

    for(size_t i = 0; i < _rx_transfers.size(); i++)
    {
        _rx_transfers[i] = create_rx_transfer(&ADSDR_impl::rx_callback);
    }

#if 0
    for(int size_t = 0; i < _tx_transfers.size(); i++)
    {
        _tx_transfers[i] = create_tx_transfer(&ADSDR_impl::tx_callback);
    }
#endif

    // Start libusb event handling
    _run_rx_tx.store(true);

    _rx_tx_worker.reset(new std::thread([this]() {
        run_rx_tx();
    }));

    set_libusb_params(_ctx, _adsdr_handle);
}

ADSDR_impl::~ADSDR_impl()
{
    // TODO: Properly stop all active transfers
    stop_rx();
    stop_tx();

    if(_adsdr_handle != nullptr)
    {
        libusb_release_interface(_adsdr_handle, 0);

        _run_rx_tx.store(false);

        // This will cause libusb_handle_events() in run_rx_tx() to return once
        libusb_close(_adsdr_handle);

        // libusb_handle_events should have returned and the thread can now be joined
        if(_rx_tx_worker != nullptr)
        {
            _rx_tx_worker->join();
        }
    }

    for(libusb_transfer *transfer : _rx_transfers)
    {
        libusb_free_transfer(transfer);
    }

#if 0
    for(libusb_transfer *transfer : _tx_transfers)
    {
        libusb_free_transfer(transfer);
    }
#endif

    if(_ctx != nullptr)
    {
        libusb_exit(_ctx); // close the session
    }
}

bool ADSDR_impl::init_sdr()
{
    ad9361_init(&phy, &ad_default_param);
    ad9361_set_rx_fir_config(phy, rx_fir_config);
    ad9361_set_tx_fir_config(phy, tx_fir_config);
    ad9361_set_no_ch_mode(phy, 1);
    print_ensm_state(phy);
//    ad9361_set_en_state_machine_mode(phy, ENSM_MODE_WAIT);
//    print_ensm_state(phy);
    return true;
}

std::vector<std::string> ADSDR_impl::list_connected()
{
    libusb_device **devs;
    libusb_context *list_ctx;

    std::vector<std::string> list;

    int ret = libusb_init(&list_ctx);
    if(ret < 0)
    {
        throw ConnectionError("libusb init error %d: error " + std::to_string(ret));
    }

    // Set verbosity level
    libusb_set_debug(list_ctx, 3);

    // Retrieve device list
    int num_devs = (int) libusb_get_device_list(list_ctx, &devs);
    if(num_devs < 0)
    {
        throw ConnectionError("libusb device list retrieval error");
    }

    // Find all ADSDR devices
    for(int i = 0; i < num_devs; i++)
    {
        libusb_device_descriptor desc;
        int ret = libusb_get_device_descriptor(devs[i], &desc);
        if(ret < 0)
        {
            throw ConnectionError("libusb error getting device descriptor %d: error " + std::to_string(ret));
        }

        if(desc.idVendor == ADSDR_VENDOR_ID && desc.idProduct == ADSDR_PRODUCT_ID)
        {
	    libusb_device_handle *temp_handle;
            int ret = libusb_open(devs[i], &temp_handle);
            if(ret != 0)
            {
                throw ConnectionError("libusb could not open found ADSDR USB device %d: error " + std::to_string(ret));
            }

        // Check if correct serial number
        if(desc.iSerialNumber)
        {
            char serial_num_buf[MAX_SERIAL_LENGTH];
            ret = libusb_get_string_descriptor_ascii(temp_handle, /*ADSDR_SERIAL_DSCR_INDEX*/desc.iSerialNumber, (unsigned char *) serial_num_buf, MAX_SERIAL_LENGTH);
            if(ret < 0)
            {
                throw ConnectionError("2) libusb could not read ADSDR serial number %d: error " + std::to_string(ret));
            }
            else
            {
                list.push_back(std::string(serial_num_buf));
            }
        }
        else
            list.push_back(std::string("12345"));
	    
	    libusb_close(temp_handle);
        }
    }

    libusb_exit(list_ctx);

    return list;
}

bool ADSDR_impl::fpga_loaded()
{
    std::array<unsigned char, ADSDR_USB_CTRL_SIZE> stat_buf{};
    int ret = libusb_control_transfer(_adsdr_handle, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_IN, ADSDR_FPGA_CONFIG_STATUS, 0, 1, stat_buf.data(), (uint16_t) stat_buf.size(), ADSDR_USB_TIMEOUT);
    if(ret < 0)
    {
        throw ConnectionError("2) ADSDR not responding %d: error " + std::to_string(ret));
    }
    //int transferred = ret;
    bool fpga_load_success = (bool) stat_buf[0];
    return fpga_load_success;
}

fpga_status ADSDR_impl::load_fpga(std::string filename)
{
    filename = filename;

    return FPGA_CONFIG_SKIPPED; // @camry
}

libusb_transfer* ADSDR_impl::create_rx_transfer(libusb_transfer_cb_fn callback)
{
    libusb_transfer *transfer = libusb_alloc_transfer(0);
    unsigned char *buf = new unsigned char[ADSDR_RX_TX_BUF_SIZE];
    libusb_fill_bulk_transfer(transfer, _adsdr_handle, ADSDR_RX_IN, buf, ADSDR_RX_TX_BUF_SIZE, callback, nullptr, ADSDR_USB_TIMEOUT);

    return transfer;
}

libusb_transfer* ADSDR_impl::create_tx_transfer(libusb_transfer_cb_fn callback)
{
    callback = callback; // warning
#if 0
    libusb_transfer *transfer = libusb_alloc_transfer(0);
    unsigned char *buf = new unsigned char[ADSDR_TX_BUF_SIZE];
    libusb_fill_bulk_transfer(transfer, _adsdr_handle, ADSDR_TX_OUT, buf, ADSDR_TX_BUF_SIZE, callback, nullptr, ADSDR_USB_TIMEOUT);
    //TODO: transfer size
    return transfer;
#endif

    return 0;
}

void ADSDR_impl::rx_callback(libusb_transfer *transfer)
{
    if(transfer->status == LIBUSB_TRANSFER_COMPLETED)
    {         
        // Transfer succeeded

        // Decode samples from transfer buffer into _rx_decoder_buf
        decode_rx_transfer(transfer->buffer, transfer->actual_length, _rx_decoder_buf);

        if(_rx_custom_callback)
        {
            // Run the callback function
            _rx_custom_callback(_rx_decoder_buf);
        }
        else
        {
            // No callback function specified, add samples to queue
            for(sample s : _rx_decoder_buf)
            {
                bool success = _rx_buf.try_enqueue(s);
                if(!success)
                {
                    // TODO: overflow! handle this
                }
            }
        }
    }
    else
    {
        // TODO: Handle error

    }

    // Resubmit the transfer
    if(transfer->status != LIBUSB_TRANSFER_CANCELLED)
    {
        int ret = libusb_submit_transfer(transfer);

        if(ret < 0)
        {
            // TODO: Handle error
        }
    }
}

void ADSDR_impl::tx_callback(libusb_transfer* transfer)
{
    if(transfer->status == LIBUSB_TRANSFER_COMPLETED)
    {
        // Success
        if(transfer->actual_length != transfer->length)
        {
            std::cout << "actual length != length: " << transfer->actual_length << "; " << transfer->length << std::endl;
        }
    }
    else
    {
        // TODO: Handle error
        if(transfer->status != LIBUSB_TRANSFER_CANCELLED)
        {
            std::cerr << "transfer error with status " << transfer->status << std::endl;
        }
    }

    // Resubmit the transfer with new data
    if(transfer->status != LIBUSB_TRANSFER_CANCELLED)
    {
        fill_tx_transfer(transfer);
        int ret = libusb_submit_transfer(transfer);

        if(ret < 0)
        {
            // TODO: Handle error
            std::cerr << "transfer submission error with status " << transfer->status << std::endl;
        }
    }
}

void ADSDR_impl::start_rx(std::function<void(const std::vector<sample> &)> rx_callback)
{
    _rx_custom_callback = rx_callback;

    deviceStop();

    //deviceReset();

    for(libusb_transfer *transfer: _rx_transfers)
    {
        int ret = libusb_submit_transfer(transfer);

        if(ret < 0)
        {
            throw ConnectionError("Could not submit RX transfer. libusb error: " + std::to_string(ret));
        }
    }

    deviceStart();
}

void ADSDR_impl::stop_rx()
{
    for(libusb_transfer *transfer: _rx_transfers)
    {
        int ret = libusb_cancel_transfer(transfer);
        if(ret == LIBUSB_ERROR_NOT_FOUND || ret == 0)
        {
            // Transfer cancelled
        }
        else
        {
            // Error
            throw ConnectionError("Could not cancel RX transfer. libusb error: " + std::to_string(ret));
        }
    }
}

void ADSDR_impl::start_tx(std::function<void(std::vector<sample> &)> tx_callback)
{
    _tx_custom_callback = tx_callback;

    // Fill the tx buffer with empty samples
    sample empty_sample{0, 0};
    while(_tx_buf.try_enqueue(empty_sample)) {}

    for(libusb_transfer *transfer: _tx_transfers)
    {
        fill_tx_transfer(transfer);
        int ret = libusb_submit_transfer(transfer);

        if(ret < 0)
        {
            throw ConnectionError("Could not submit TX transfer. libusb error: " + std::to_string(ret));
        }
    }
}

void ADSDR_impl::stop_tx()
{
#if 0
    for(libusb_transfer *transfer: _tx_transfers)
    {
        int ret = libusb_cancel_transfer(transfer);
        if(ret == LIBUSB_ERROR_NOT_FOUND || ret == 0)
        {
            // Transfer cancelled
        }
        else
        {
            // Error
            throw ConnectionError("Could not cancel TX transfer. libusb error: " + std::to_string(ret));
        }
    }
#endif
}

int ADSDR_impl::fill_tx_transfer(libusb_transfer* transfer)
{
    // Fill the transfer buffer with available samples
    transfer->length = ADSDR_TX_BUF_SIZE;

    _tx_encoder_buf.resize(transfer->length/ADSDR_BYTES_PER_SAMPLE);

    if(_tx_custom_callback)
    {
        _tx_custom_callback(_tx_encoder_buf);
    }
    else
    {
        for(sample &s : _tx_encoder_buf)
        {
            int success = _tx_buf.try_dequeue(s);
            if(!success)
            {
                // TODO: Notify of this? Do something else?
                // No data available, fill with zeros
                s.i = 0;
                s.q = 0;
            }
        }
    }

    for(int i = 0; i < transfer->length; i+=ADSDR_BYTES_PER_SAMPLE)
    {
        // SKIP THIS: Convert -1.0 to 1.0 float sample value to signed 16-bit int with range -2048 to 2048
        int16_t signed_i = _tx_encoder_buf[i/ADSDR_BYTES_PER_SAMPLE].i;
        int16_t signed_q = _tx_encoder_buf[i/ADSDR_BYTES_PER_SAMPLE].q;

        // Unsigned 16-bit ints holding the two's-complement 12-bit sample values
        uint16_t raw_i;
        uint16_t raw_q;

        if(signed_i >= 0)
        {
            raw_i = (uint16_t) signed_i;
        }
        else
        {
            raw_i = (((uint16_t) (-signed_i)) ^ ((uint16_t) 0xFFF)) + (uint16_t) 1;
        }

        if(signed_q >= 0)
        {
            raw_q = (uint16_t) signed_q;
        }
        else
        {
            raw_q = (((uint16_t) (-signed_q)) ^ ((uint16_t) 0xFFF)) + (uint16_t) 1;
        }

        // Copy raw i/q data into the buffer
        memcpy(transfer->buffer + i, &raw_q, sizeof(raw_q));
        memcpy(transfer->buffer + i + sizeof(raw_q), &raw_i, sizeof(raw_i));
    }

    return transfer->length;
}

void ADSDR_impl::decode_rx_transfer(unsigned char *buffer, int actual_length, std::vector<sample> &destination)
{
    int16_t* pSamplesIn = (short*)buffer;
    int lenght = actual_length / (sizeof(short) * 2 * 2);
    destination.resize(lenght);
    sample* pSamplesOut = destination.data();

    for(int i = 0; i < lenght; i++)
    {
        pSamplesOut[i].i = pSamplesIn[4*i+0]>>4;
        pSamplesOut[i].q = pSamplesIn[4*i+1]>>4;
    }
}

void ADSDR_impl::run_rx_tx()
{
    while(_run_rx_tx.load())
    {
        libusb_handle_events(_ctx);
    }
}

unsigned long ADSDR_impl::available_rx_samples()
{
    return _rx_buf.size_approx();
}

bool ADSDR_impl::get_rx_sample(sample &s)
{
    return _rx_buf.try_dequeue(s);
}

bool ADSDR_impl::submit_tx_sample(sample &s)
{
    return _tx_buf.try_enqueue(s);
}

command ADSDR_impl::make_command(command_id id, double param) const
{
    command cmd;

    cmd.cmd = id;
    switch(cmd.cmd)
    {
    case SET_TX_LO_FREQ:
    {
        uint64_t cast_param = static_cast<uint64_t>(param);
        memcpy(&cmd.param, &cast_param, sizeof(cast_param));
    }
        break;
    case SET_TX_SAMP_FREQ:
    {
        uint32_t cast_param = static_cast<uint32_t>(param);
        memcpy(&cmd.param, &cast_param, sizeof(cast_param));
    }
        break;
    case SET_TX_RF_BANDWIDTH:
    {
        uint32_t cast_param = static_cast<uint32_t>(param);
        memcpy(&cmd.param, &cast_param, sizeof(cast_param));
    }
        break;
    case SET_TX_ATTENUATION:
    {
        uint32_t cast_param = static_cast<uint32_t>(param);
        memcpy(&cmd.param, &cast_param, sizeof(cast_param));
    }
        break;
    case SET_TX_FIR_EN:
    {
        uint8_t cast_param = static_cast<uint8_t>(param);
        memcpy(&cmd.param, &cast_param, sizeof(cast_param));
    }
        break;
    case SET_RX_LO_FREQ:
    {
        uint64_t cast_param = static_cast<uint64_t>(param);
        memcpy(&cmd.param, &cast_param, sizeof(cast_param));
    }
        break;
    case SET_RX_SAMP_FREQ:
    {
        uint32_t cast_param = static_cast<uint32_t>(param);
        memcpy(&cmd.param, &cast_param, sizeof(cast_param));
    }
        break;
    case SET_RX_RF_BANDWIDTH:
    {
        std::

        uint32_t cast_param = static_cast<uint32_t>(param);
        memcpy(&cmd.param, &cast_param, sizeof(cast_param));
    }
        break;
    case SET_RX_GC_MODE:
    {
        uint8_t cast_param = static_cast<uint8_t>(param);
        memcpy(&cmd.param, &cast_param, sizeof(cast_param));
    }
        break;
    case SET_RX_RF_GAIN:
    {
        int32_t cast_param = static_cast<int32_t>(param);
        memcpy(&cmd.param, &cast_param, sizeof(cast_param));
    }
        break;
    case SET_RX_FIR_EN:
    {
        uint8_t cast_param = static_cast<uint8_t>(param);
        memcpy(&cmd.param, &cast_param, sizeof(cast_param));
    }
        break;
    case SET_DATAPATH_EN:
    {
        uint8_t cast_param = static_cast<uint8_t>(param);
        memcpy(&cmd.param, &cast_param, sizeof(cast_param));
    }
        break;
    case SET_LOOPBACK_EN:
    {
        uint8_t cast_param = static_cast<uint8_t>(param);
        memcpy(&cmd.param, &cast_param, sizeof(cast_param));
    }
        break;
    default:
        throw std::runtime_error("make_command error: " + std::to_string(id));
    }

    return cmd;
}

response ADSDR_impl::send_cmd(command cmd)
{    
    response reply;

    std::cout << " Send cmd: " << cmd.cmd << " param: " << cmd.param << std::endl;
    if(cmd.cmd < COMMAND_SIZE)
    {
        reply = ad9364_cmd(cmd.cmd, cmd.param);
    }

    return reply;
}

adsdr_version ADSDR_impl::version()
{
    response res = send_cmd({GET_FPGA_VERSION});
    uint8_t fpga_major_version = ((uint8_t*) &res.param)[0];
    uint8_t fpga_minor_version = ((uint8_t*) &res.param)[1];
    uint8_t fpga_patch_version = ((uint8_t*) &res.param)[2];

    adsdr_version v;
    v.fx3 = _fx3_fw_version;
    v.fpga = std::to_string(fpga_major_version) + "." + std::to_string(fpga_minor_version) + "." + std::to_string(fpga_patch_version);

    return v;
}


//========================================================================================================

int ADSDR_impl::deviceStart()
{
    uint8_t buf[1] = {0};
    return txControlToDevice(buf, 1, DEVICE_START, 0, 0);
}

int ADSDR_impl::deviceStop()
{
    uint8_t buf[1] = {0};
    return txControlToDevice(buf, 1, DEVICE_STOP, 0, 0);
}

int ADSDR_impl::deviceReset()
{
    uint8_t buf[3] = {0, 0, 0xFF};
    return txControlToDevice(buf, 3, DEVICE_RESET, 0, 0);
}

int ADSDR_impl::txControlToDevice(uint8_t* src, uint32_t size8, uint8_t cmd, uint16_t wValue, uint16_t wIndex)
{
    if(_adsdr_handle != 0)
    {
        /* From libusb-1.0 documentation:
         * Bits 0:4 determine recipient, see libusb_request_recipient.
         * Bits 5:6 determine type, see libusb_request_type.
         * Bit 7 determines data transfer direction, see libusb_endpoint_direction. */
        uint8_t bmRequestType = LIBUSB_RECIPIENT_DEVICE | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT;

        /* From libusb-1.0 documentation:
         * If the type bits of bmRequestType are equal to LIBUSB_REQUEST_TYPE_STANDARD
         * then this field refers to libusb_standard_request.
         * For other cases, use of this field is application-specific. */
        uint8_t bRequest = cmd;

        /* From libusb-1.0 documentation:
         * timeout (in millseconds) that this function should wait before giving up
         * due to no response being received.
         * For an unlimited timeout, use value 0. */
        uint32_t timeout_ms = DEV_UPLOAD_TIMEOUT_MS;

        int res = libusb_control_transfer(_adsdr_handle, bmRequestType, bRequest, wValue, wIndex, src, size8, timeout_ms );
        if ( res != ( int ) size8 ) {
            fprintf( stderr, "FX3Dev::txControlToDevice() error %d %s\n", res, libusb_error_name(res) );
            return FX3_ERR_CTRL_TX_FAIL;
        }
        return FX3_ERR_OK;
    }

    return FX3_ERR_NO_DEVICE_FOUND;
}

int ADSDR_impl::txControlFromDevice(uint8_t* dest, uint32_t size8 , uint8_t cmd, uint16_t wValue, uint16_t wIndex)
{
    if(_adsdr_handle != 0)
    {
        /* From libusb-1.0 documentation:
         * Bits 0:4 determine recipient, see libusb_request_recipient.
         * Bits 5:6 determine type, see libusb_request_type.
         * Bit 7 determines data transfer direction, see libusb_endpoint_direction. */
        uint8_t bmRequestType = LIBUSB_RECIPIENT_DEVICE | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN;
        uint8_t bRequest = cmd;
        uint32_t timeout_ms = DEV_UPLOAD_TIMEOUT_MS;

        int res = libusb_control_transfer(_adsdr_handle, bmRequestType, bRequest, wValue, wIndex, dest, size8, timeout_ms );
        if ( res != ( int ) size8 ) {
            fprintf( stderr, "FX3Dev::transferDataFromDevice() error %d %s\n", res, libusb_error_name(res) );
            return FX3_ERR_CTRL_TX_FAIL;
        }
        return FX3_ERR_OK;
    }

    return FX3_ERR_NO_DEVICE_FOUND;
}

//=====================================  Commands  ======================================================

response ADSDR_impl::ad9364_cmd(int cmd, uint64_t param)
{
    char param_no = 1;
    char error = CMD_OK;
    uint64_t ret_value = 0;

    (this->*(m_cmd_list[cmd]))(&param, param_no, &error, &ret_value);

    response reply;
    reply.cmd = (command_id)cmd;
    reply.error = (command_err)error;
    reply.param = ret_value;

    return reply;
}

/**************************************************************************//***
 * @brief Gets current TX LO frequency [Hz].
 *
 * @return None.
*******************************************************************************/
void ADSDR_impl::get_tx_lo_freq(uint64_t *param, char param_no, char* error, uint64_t* response) // "tx_lo_freq?" command
{
    uint64_t lo_freq_hz;

    ad9361_get_tx_lo_freq(phy, &lo_freq_hz);

    *error = CMD_OK;
    memcpy(response, &lo_freq_hz, sizeof(lo_freq_hz));

    printf("tx_lo_freq=%llu Hz\n\r", lo_freq_hz);
}

/**************************************************************************//***
 * @brief Sets the TX LO frequency [Hz].
 *
 * @return None.
*******************************************************************************/
void ADSDR_impl::set_tx_lo_freq(uint64_t *param, char param_no, char* error, uint64_t* response) // "tx_lo_freq=" command
{
    uint64_t lo_freq_hz;
    uint32_t port = TXA;

    if(param_no >= 1)
    {
        memcpy(&lo_freq_hz, param, sizeof(lo_freq_hz));

        // Select appropriate signal port/path
        if(lo_freq_hz >= 3000000000ULL) // 3000-6000 MHz: Port A
        {
            port = TXA;
            printf("INFO: using TX port A\n\r");
        }
        else // 70-3000 MHz: Port B
        {
            port = TXB;
            printf("INFO: using TX port B\n\r");
        }
        ad9361_set_tx_rf_port_output(phy, port);
//        tx_band_select(port);

        ad9361_set_tx_lo_freq(phy, lo_freq_hz);
        ad9361_get_tx_lo_freq(phy, &lo_freq_hz);

        *error = CMD_OK;
        memcpy(response, &lo_freq_hz, sizeof(lo_freq_hz));

        printf("tx_lo_freq=%llu Hz\n\r", lo_freq_hz);
    }
    else
    {
        *error = CMD_INVALID_PARAM;
        printf("ERROR: set_tx_lo_freq: invalid parameter!\n\r");
    }
}

/**************************************************************************//***
 * @brief Gets current sampling frequency [Hz].
 *
 * @return None.
*******************************************************************************/
void ADSDR_impl::get_tx_samp_freq(uint64_t *param, char param_no, char* error, uint64_t* response) // "tx_samp_freq?" command
{
    uint32_t sampling_freq_hz;

    ad9361_get_tx_sampling_freq(phy, &sampling_freq_hz);

    *error = CMD_OK;
    memcpy(response, &sampling_freq_hz, sizeof(sampling_freq_hz));

    printf("tx_samp_freq=%lu Hz\n\r", sampling_freq_hz);
}

/**************************************************************************//***
 * @brief Sets the sampling frequency [Hz].
 *
 * @return None.
*******************************************************************************/
void ADSDR_impl::set_tx_samp_freq(uint64_t *param, char param_no, char* error, uint64_t* response) // "tx_samp_freq=" command
{
    uint32_t sampling_freq_hz;

    if(param_no >= 1)
    {
        memcpy(&sampling_freq_hz, param, sizeof(sampling_freq_hz));
        ad9361_set_tx_sampling_freq(phy, sampling_freq_hz);
        ad9361_get_tx_sampling_freq(phy, &sampling_freq_hz);

        *error = CMD_OK;
        memcpy(response, &sampling_freq_hz, sizeof(sampling_freq_hz));

        printf("tx_samp_freq=%lu Hz\n\r", sampling_freq_hz);
    }
    else
    {
        *error = CMD_INVALID_PARAM;
        printf("ERROR: set_tx_samp_freq: invalid parameter!\n\r");
    }
}

/**************************************************************************//***
 * @brief Gets current TX RF bandwidth [Hz].
 *
 * @return None.
*******************************************************************************/
void ADSDR_impl::get_tx_rf_bandwidth(uint64_t *param, char param_no, char* error, uint64_t* response) // "tx_rf_bandwidth?" command
{
    uint32_t bandwidth_hz;

    ad9361_get_tx_rf_bandwidth(phy, &bandwidth_hz);

    *error = CMD_OK;
    memcpy(response, &bandwidth_hz, sizeof(bandwidth_hz));

    printf("tx_rf_bandwidth=%lu Hz\n\r", bandwidth_hz);
}

/**************************************************************************//***
 * @brief Sets the TX RF bandwidth [Hz].
 *
 * @return None.
*******************************************************************************/
void ADSDR_impl::set_tx_rf_bandwidth(uint64_t *param, char param_no, char* error, uint64_t* response) // "tx_rf_bandwidth=" command
{
    uint32_t bandwidth_hz;

    if(param_no >= 1)
    {
        memcpy(&bandwidth_hz, param, sizeof(bandwidth_hz));
        ad9361_set_tx_rf_bandwidth(phy, bandwidth_hz);
        ad9361_get_tx_rf_bandwidth(phy, &bandwidth_hz);

        *error = CMD_OK;
        memcpy(response, &bandwidth_hz, sizeof(bandwidth_hz));

        printf("tx_rf_bandwidth=%lu Hz\n\r", bandwidth_hz);
    }
    else
    {
        *error = CMD_INVALID_PARAM;
        printf("ERROR: set_tx_rf_bandwidth: invalid parameter!\n\r");
    }
}

/**************************************************************************//***
 * @brief Gets current TX attenuation [mdB].
 *
 * @return None.
*******************************************************************************/
void ADSDR_impl::get_tx_attenuation(uint64_t *param, char param_no, char* error, uint64_t* response) // "tx1_attenuation?" command
{
    uint32_t attenuation_mdb;

    ad9361_get_tx_attenuation(phy, 0, &attenuation_mdb);

    *error = CMD_OK;
    memcpy(response, &attenuation_mdb, sizeof(attenuation_mdb));

    printf("tx_attenuation=%lu mdB\n\r", attenuation_mdb);
}

/**************************************************************************//***
 * @brief Sets the TX attenuation [mdB].
 *
 * @return None.
*******************************************************************************/
void ADSDR_impl::set_tx_attenuation(uint64_t *param, char param_no, char* error, uint64_t* response) // "tx1_attenuation=" command
{
    uint32_t attenuation_mdb;

    if(param_no >= 1)
    {
        attenuation_mdb = param[0];
        memcpy(&attenuation_mdb, param, sizeof(attenuation_mdb));
        ad9361_set_tx_attenuation(phy, 0, attenuation_mdb);
        ad9361_get_tx_attenuation(phy, 0, &attenuation_mdb);

        *error = CMD_OK;
        memcpy(response, &attenuation_mdb, sizeof(attenuation_mdb));

        printf("tx_attenuation=%lu mdB\n\r", attenuation_mdb);
    }
    else
    {
        *error = CMD_INVALID_PARAM;
        printf("ERROR: set_tx_attenuation: invalid parameter!\n\r");
    }
}

/**************************************************************************//***
 * @brief Gets current TX FIR state.
 *
 * @return None.
*******************************************************************************/
void ADSDR_impl::get_tx_fir_en(uint64_t *param, char param_no, char* error, uint64_t* response) // "tx_fir_en?" command
{
    uint8_t en_dis;

    ad9361_get_tx_fir_en_dis(phy, &en_dis);

    *error = CMD_OK;
    memcpy(response, &en_dis, sizeof(en_dis));

    printf("tx_fir_en=%d\n\r", en_dis);
}

/**************************************************************************//***
 * @brief Sets the TX FIR state.
 *
 * @return None.
*******************************************************************************/
void ADSDR_impl::set_tx_fir_en(uint64_t *param, char param_no, char* error, uint64_t* response) // "tx_fir_en=" command
{
    uint8_t en_dis;

    if(param_no >= 1)
    {
        en_dis = param[0];
        memcpy(&en_dis, param, sizeof(en_dis));
        ad9361_set_tx_fir_en_dis(phy, en_dis);
        ad9361_get_tx_fir_en_dis(phy, &en_dis);

        *error = CMD_OK;
        memcpy(response, &en_dis, sizeof(en_dis));

        printf("tx_fir_en=%d\n\r", en_dis);
    }
    else
    {
        *error = CMD_INVALID_PARAM;
        printf("ERROR: set_tx_fir_en: invalid parameter!\n\r");
    }
}

/**************************************************************************//***
 * @brief Gets current RX LO frequency [Hz].
 *
 * @return None.
*******************************************************************************/
void ADSDR_impl::get_rx_lo_freq(uint64_t *param, char param_no, char* error, uint64_t* response) // "rx_lo_freq?" command
{
    uint64_t lo_freq_hz;

    ad9361_get_rx_lo_freq(phy, &lo_freq_hz);

    *error = CMD_OK;
    memcpy(response, &lo_freq_hz, sizeof(lo_freq_hz));

    printf("rx_lo_freq=%llu Hz\n\r", lo_freq_hz);
}

/**************************************************************************//***
 * @brief Sets the RX LO frequency [Hz].
 *
 * @return None.
*******************************************************************************/
void ADSDR_impl::set_rx_lo_freq(uint64_t *param, char param_no, char* error, uint64_t* response) // "rx_lo_freq=" command
{
    uint64_t lo_freq_hz;
    uint32_t port = A_BALANCED;

    if(param_no >= 1)
    {
        memcpy(&lo_freq_hz, param, sizeof(lo_freq_hz));

        // Select appropriate signal port/path
        if(lo_freq_hz >= 3000000000ULL) // 3000-6000 MHz: Port A
        {
            port = A_BALANCED;
            printf("INFO: using RX port A\n\r");
        }
        else if(lo_freq_hz >= 1600000000ULL) // 1600-3000 MHz: Port B
        {
            port = B_BALANCED;
            printf("INFO: using RX port B\n\r");
        }
        else // 70-1800 MHz: Port C
        {
            port = C_BALANCED;
            printf("INFO: using RX port C\n\r");
        }
        ad9361_set_rx_rf_port_input(phy, port);
//        rx_band_select(port);

        // Set LO frequency
        ad9361_set_rx_lo_freq(phy, lo_freq_hz);
        ad9361_get_rx_lo_freq(phy, &lo_freq_hz);

        *error = CMD_OK;
        memcpy(response, &lo_freq_hz, sizeof(lo_freq_hz));

        printf("rx_lo_freq=%llu Hz\n\r", lo_freq_hz);
    }
    else
    {
        printf("ERROR: set_rx_lo_freq: invalid parameter!\n\r");
    }
}

/**************************************************************************//***
 * @brief Gets current RX sampling frequency [Hz].
 *
 * @return None.
*******************************************************************************/
void ADSDR_impl::get_rx_samp_freq(uint64_t *param, char param_no, char* error, uint64_t* response) // "rx_samp_freq?" command
{
    uint32_t sampling_freq_hz;

    ad9361_get_rx_sampling_freq(phy, &sampling_freq_hz);

    *error = CMD_OK;
    memcpy(response, &sampling_freq_hz, sizeof(sampling_freq_hz));

    printf("rx_samp_freq=%lu Hz\n\r", sampling_freq_hz);
}

/**************************************************************************//***
 * @brief Sets the RX sampling frequency [Hz].
 *
 * @return None.
*******************************************************************************/
void ADSDR_impl::set_rx_samp_freq(uint64_t *param, char param_no, char* error, uint64_t* response) // "rx_samp_freq=" command
{
    uint32_t sampling_freq_hz;

    if(param_no >= 1)
    {
        sampling_freq_hz = (uint32_t)param[0];
        memcpy(&sampling_freq_hz, param, sizeof(sampling_freq_hz));
        ad9361_set_rx_sampling_freq(phy, sampling_freq_hz);
        ad9361_get_rx_sampling_freq(phy, &sampling_freq_hz);

        *error = CMD_OK;
        memcpy(response, &sampling_freq_hz, sizeof(sampling_freq_hz));

        printf("rx_samp_freq=%lu Hz\n\r", sampling_freq_hz);
    }
    else
    {
        *error = CMD_INVALID_PARAM;
        printf("ERROR: rx_samp_freq: invalid parameter!\n\r");
    }
}

/**************************************************************************//***
 * @brief Gets current RX RF bandwidth [Hz].
 *
 * @return None.
*******************************************************************************/
void ADSDR_impl::get_rx_rf_bandwidth(uint64_t *param, char param_no, char* error, uint64_t* response) // "rx_rf_bandwidth?" command
{
    uint32_t bandwidth_hz;

    ad9361_get_rx_rf_bandwidth(phy, &bandwidth_hz);

    *error = CMD_OK;
    memcpy(response, &bandwidth_hz, sizeof(bandwidth_hz));

    printf("rx_rf_bandwidth=%lu Hz\n\r", bandwidth_hz);
}

/**************************************************************************//***
 * @brief Sets the RX RF bandwidth [Hz].
 *
 * @return None.
*******************************************************************************/
void ADSDR_impl::set_rx_rf_bandwidth(uint64_t *param, char param_no, char* error, uint64_t* response) // "rx_rf_bandwidth=" command
{
    uint32_t bandwidth_hz;

    if(param_no >= 1)
    {
        memcpy(&bandwidth_hz, param, sizeof(bandwidth_hz));
        ad9361_set_rx_rf_bandwidth(phy, bandwidth_hz);
        ad9361_get_rx_rf_bandwidth(phy, &bandwidth_hz);

        *error = CMD_OK;
        memcpy(response, &bandwidth_hz, sizeof(bandwidth_hz));

        printf("rx_rf_bandwidth=%lu Hz\n\r", bandwidth_hz);
    }
    else
    {
        *error = CMD_INVALID_PARAM;
        printf("ERROR: set_rx_rf_bandwidth: invalid parameter!\n\r");
    }
}

/**************************************************************************//***
 * @brief Gets current RX GC mode.
 *
 * @return None.
*******************************************************************************/
void ADSDR_impl::get_rx_gc_mode(uint64_t *param, char param_no, char* error, uint64_t* response) // "rx1_gc_mode?" command
{
    uint8_t gc_mode;

    ad9361_get_rx_gain_control_mode(phy, 0, &gc_mode);

    *error = CMD_OK;
    memcpy(response, &gc_mode, sizeof(gc_mode));

    printf("rx_gc_mode=%d\n\r", gc_mode);
}

/**************************************************************************//***
 * @brief Sets the RX GC mode.
 *
 * @return None.
*******************************************************************************/
void ADSDR_impl::set_rx_gc_mode(uint64_t *param, char param_no, char* error, uint64_t* response) // "rx1_gc_mode=" command
{
    uint8_t gc_mode;

    if(param_no >= 1)
    {
        memcpy(&gc_mode, param, sizeof(gc_mode));
        ad9361_set_rx_gain_control_mode(phy, 0, gc_mode);
        ad9361_get_rx_gain_control_mode(phy, 0, &gc_mode);

        *error = CMD_OK;
        memcpy(response, &gc_mode, sizeof(gc_mode));

        printf("rx_gc_mode=%d\n\r", gc_mode);
    }
    else
    {
        *error = CMD_INVALID_PARAM;
        printf("ERROR: set_rx_gc_mode: invalid parameter!\n\r");
    }
}

/**************************************************************************//***
 * @brief Gets current RX RF gain.
 *
 * @return None.
*******************************************************************************/
void ADSDR_impl::get_rx_rf_gain(uint64_t *param, char param_no, char* error, uint64_t* response) // "rx1_rf_gain?" command
{
    int32_t gain_db;

    ad9361_get_rx_rf_gain(phy, 0, &gain_db);

    *error = CMD_OK;
    memcpy(response, &gain_db, sizeof(gain_db));

    printf("rx_rf_gain=%ld dB\n\r", gain_db);
}

/**************************************************************************//***
 * @brief Sets the RX RF gain. [dB]
 *
 * @return None.
*******************************************************************************/
void ADSDR_impl::set_rx_rf_gain(uint64_t *param, char param_no, char* error, uint64_t* response) // "rx1_rf_gain=" command
{
    int32_t gain_db;

    if(param_no >= 1)
    {
        memcpy(&gain_db, param, sizeof(gain_db));
        ad9361_set_rx_rf_gain(phy, 0, gain_db);
        ad9361_get_rx_rf_gain(phy, 0, &gain_db);

        *error = CMD_OK;
        memcpy(response, &gain_db, sizeof(gain_db));

        printf("rx_rf_gain=%ld dB\n\r", gain_db);
    }
    else
    {
        *error = CMD_INVALID_PARAM;
        printf("ERROR: set_rx_rf_gain: invalid parameter!\n\r");
    }
}

/**************************************************************************//***
 * @brief Gets current RX FIR state.
 *
 * @return None.
*******************************************************************************/
void ADSDR_impl::get_rx_fir_en(uint64_t *param, char param_no, char* error, uint64_t* response) // "rx_fir_en?" command
{
    uint8_t en_dis;

    ad9361_get_rx_fir_en_dis(phy, &en_dis);

    *error = CMD_OK;
    memcpy(response, &en_dis, sizeof(en_dis));

    printf("rx_fir_en=%d\n\r", en_dis);
}

/**************************************************************************//***
 * @brief Sets the RX FIR state.
 *
 * @return None.
*******************************************************************************/
void ADSDR_impl::set_rx_fir_en(uint64_t *param, char param_no, char* error, uint64_t* response) // "rx_fir_en=" command
{
    uint8_t en_dis;

    if(param_no >= 1)
    {
        memcpy(&en_dis, param, sizeof(en_dis));
        ad9361_set_rx_fir_en_dis(phy, en_dis);
        ad9361_get_rx_fir_en_dis(phy, &en_dis);

        *error = CMD_OK;
        memcpy(response, &en_dis, sizeof(en_dis));

        printf("rx_fir_en=%d\n\r", en_dis);
    }
    else
    {
        *error = CMD_INVALID_PARAM;
        printf("ERROR: set_rx_fir_en: invalid parameter!\n\r");
    }
}

void ADSDR_impl::set_datapath_en(uint64_t *param, char param_no, char* error, uint64_t* response)
{
    uint8_t en_dis;
    int ret = 0;

    if(param_no >= 1)
    {
        memcpy(&en_dis, param, sizeof(en_dis));
        *error = CMD_OK;
        memcpy(response, &en_dis, sizeof(en_dis));

        if(en_dis == 1)
        {
            // Enable FDD
            ret = ad9361_set_en_state_machine_mode(phy, ENSM_MODE_FDD);
            print_ensm_state(phy);
        }
        else
        {
            ret = ad9361_set_en_state_machine_mode(phy, ENSM_MODE_WAIT);
            print_ensm_state(phy);
        }

        if(ret != 0)
        {
            *error = CMD_ENSM_ERR;
        }

        printf("datapath_en=%d\n\r", en_dis);
    }
    else
    {
        *error = CMD_INVALID_PARAM;
        printf("ERROR: set_datapath_en: invalid parameter!\n\r");
    }
}

void ADSDR_impl::get_version(uint64_t *param, char param_no, char* error, uint64_t* response)
{
    uint8_t version[8] = {
            0,
            3,
            0,
            0,
            0,
            0,
            0,
            0
    };

    *error = CMD_OK;
    memcpy(response, &version, sizeof(uint64_t));
}

void ADSDR_impl::set_loopback_en(uint64_t* param, char param_no, char* error, uint64_t* response)
{
    uint8_t en_dis;
    int ret = -1;

    if(param_no >= 1)
    {
        memcpy(&en_dis, param, sizeof(en_dis));
        *error = CMD_OK;
        memcpy(response, &en_dis, sizeof(en_dis));

        if(en_dis == 1)
        {
            // Enable loopback
            ret = ad9361_bist_loopback(phy, 1);
        }
        else
        {
            // Disable
            ret = ad9361_bist_loopback(phy, 0);
        }

        if(ret != 0)
        {
            *error = CMD_ENSM_ERR;
        }

        printf("loopback_en=%d\n\r", en_dis);
    }
    else
    {
        *error = CMD_INVALID_PARAM;
        printf("ERROR: set_loopback_en: invalid parameter!\n\r");
    }
}

void ADSDR_impl::print_ensm_state(struct ad9361_rf_phy *phy)
{
    static uint32_t mode;

    ad9361_get_en_state_machine_mode(phy, &mode);

    switch(mode)
    {
        case ENSM_MODE_TX:
            printf("INFO: AD9364 in TX mode\n\r");
            break;
        case ENSM_MODE_RX:
            printf("INFO: AD9364 in RX mode\n\r");
            break;
        case ENSM_MODE_ALERT:
            printf("INFO: AD9364 in ALERT mode\n\r");
            break;
        case ENSM_MODE_FDD:
            printf("INFO: AD9364 in FDD mode\n\r");
            break;
        case ENSM_MODE_WAIT:
            printf("INFO: AD9364 in WAIT mode\n\r");
            break;
        case ENSM_MODE_SLEEP:
            printf("INFO: AD9364 in SLEEP mode\n\r");
            break;
        case ENSM_MODE_PINCTRL:
            printf("INFO: AD9364 in PINCTRL mode\n\r");
            break;
        case ENSM_MODE_PINCTRL_FDD_INDEP:
            printf("INFO: AD9364 in PINCTRL_FDD_INDEP mode\n\r");
            break;
    }
}

