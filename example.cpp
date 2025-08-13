/**
 * @file ad7124_console_app.cpp
 * @brief AD7124 Console Application
 */

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <array>

/******************************************************************************/
/********************* Macros and Constants Definitions ***********************/
/******************************************************************************/

/* Maximum number of ADC samples for the single/continuous measurement type */
constexpr uint16_t MAX_ADC_SAMPLES = 100;

/* Maximum number of ADC samples for the average measurement type */
constexpr uint16_t MAX_AVG_ADC_SAMPLES = 8;

/* The max number of sensors connected to any AD7124 device */
constexpr uint8_t MAX_RTD_SENSORS = 5;

/******************************************************************************/
/********************* Variables and User Defined Data Types *****************/
/******************************************************************************/

/* Sensor configuration names */
static const std::array<const char*, 4> sensor_configs = {
    "2-Wire RTD",
    "3-Wire RTD", 
    "4-Wire RTD",
    "Thermocouple",
    "Thermistor"
};

/* Maximum number of sensors connected to different AD7124 devices */
static const std::array<uint8_t, 5> max_supported_sensors = {
    MAX_RTD_SENSORS,  // 2-wire RTD
    MAX_RTD_SENSORS,  // 3-wire RTD  
    MAX_RTD_SENSORS,  // 4-wire RTD
    MAX_THERMOCOUPLE_SENSORS,  // Thermocouple
    MAX_THERMISTOR_SENSORS     // Thermistor
};

/* Current sensor configuration */
static const char* current_sensor_config;
static enum sensor_config_ids current_sensor_config_id;

/* ADC raw data for n samples */
static std::array<std::array<int32_t, MAX_ADC_SAMPLES>, NUM_OF_SENSOR_CHANNELS> n_sample_data;

/* CJC sensor ADC raw data for n samples */
static std::array<std::array<int32_t, MAX_ADC_SAMPLES>, MAX_THERMOCOUPLE_SENSORS> n_cjc_sample_data;

/* User input for console menu */
static std::array<char, 2> user_input = {'Y'};

/* For storing decimal value(s) in character form */
static std::array<char, 50> decimal_eqv_str = {};
static std::array<char, 50 * NUM_OF_SENSOR_CHANNELS> decimal_eqv_str_arr = {};

/* Sensor enable status */
static std::array<bool, NUM_OF_SENSOR_CHANNELS> sensor_enable_status = {
    false, false, false, false, false, false, false, false
};

/* ADC calibration types */
enum adc_calibration_type {
    INTERNAL_CALIBRATION,
    SYSTEM_CALIBRATION
};

/* ADC calibration configs */
struct adc_calibration_configs {
    int32_t power_mode;
    std::array<int32_t, NUM_OF_SENSOR_CHANNELS> gain_before_calib;
    std::array<int32_t, NUM_OF_SENSOR_CHANNELS> gain_after_calib;
    std::array<int32_t, NUM_OF_SENSOR_CHANNELS> offset_after_calib;
    std::array<int32_t, NUM_OF_SENSOR_CHANNELS> offset_before_calib;
    bool adc_calibration_done;
};

static adc_calibration_configs adc_calibration_config;

/* 3-wire RTD calibration types */
enum rtd_3wire_calibration_type {
    MEASURING_EXCITATION_CURRENT,
    CHOPPING_EXCITATION_CURRENT
};

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/**
 * @brief Initialize the AD7124 device and associated low level peripherals
 * @param config_id[in] - Configuration ID
 * @return SUCCESS/FAILURE
 */
int32_t ad7124_app_initialize(uint8_t config_id) {
    /* Get the current sensor configuration */
    current_sensor_config = sensor_configs[config_id];
    current_sensor_config_id = static_cast<sensor_config_ids>(config_id);

    /* Don't apply calibration coefficients when new config is selected */
    adc_calibration_config.adc_calibration_done = false;

    ad7124_init_params.regs = ad7124_register_map;

    /* Initialize the AD7124 device */
    if (ad7124_setup(&p_ad7124_dev, ad7124_init_params) != SUCCESS) {
        printf("Error setting up AD7124 device" EOL);
        return FAILURE;
    }

    return SUCCESS;
}

/**
 * @brief Perform the ADC conversion
 * @param chn[in] - ADC channel to perform conversion on
 * @param data[out] - Pointer to array to read data into
 * @param measurement_type[in] - Temperature measurement and display type
 * @return SUCCESS/FAILURE
 */
int32_t perform_adc_conversion(uint8_t chn, 
                               int32_t (*data)[MAX_ADC_SAMPLES],
                               sensor_measurement_type measurement_type) {
    int32_t sample_data;
    int64_t avg_sample_data = 0;
    uint16_t samples_cnt;

    /* Enable the current channel */
    ad7124_register_map[AD7124_Channel_0 + chn].value |= AD7124_CH_MAP_REG_CH_EN;
    if (ad7124_write_register(p_ad7124_dev, ad7124_register_map[AD7124_Channel_0 + chn]) != SUCCESS) {
        return FAILURE;
    }

    samples_cnt = (measurement_type == AVERAGED_MEASUREMENT) ? MAX_AVG_ADC_SAMPLES : MAX_ADC_SAMPLES;

    /* Enter into continuous conversion mode */
    ad7124_register_map[AD7124_ADC_Control].value &= (~AD7124_ADC_CTRL_REG_MSK);
    ad7124_register_map[AD7124_ADC_Control].value |= AD7124_ADC_CTRL_REG_MODE(CONTINUOUS_CONVERSION_MODE);

    if (ad7124_write_register(p_ad7124_dev, ad7124_register_map[AD7124_ADC_Control]) != SUCCESS) {
        return FAILURE;
    }

    /* Let the channel settle */
    mdelay(100);

    /* Read adc samples */
    for (uint16_t sample = 0; sample < samples_cnt; sample++) {
        /*
         * This polls the status register READY/ bit to determine when conversion is done
         * This also ensures the STATUS register value is up to date and contains the
         * channel that was sampled as well.
         * Then we immediately read the ADC result register.
         */
        if (ad7124_wait_for_conv_ready(p_ad7124_dev, p_ad7124_dev->spi_rdy_poll_cnt) != SUCCESS) {
            return FAILURE;
        }

        if (ad7124_read_data(p_ad7124_dev, &sample_data) != SUCCESS) {
            return FAILURE;
        }

        (*data)[sample] = sample_data;
        avg_sample_data += sample_data;
    }

    /* Disable the current channel */
    ad7124_register_map[AD7124_Channel_0 + chn].value &= (~AD7124_CH_MAP_REG_CH_EN);
    if (ad7124_write_register(p_ad7124_dev, ad7124_register_map[AD7124_Channel_0 + chn]) != SUCCESS) {
        return FAILURE;
    }

    if (measurement_type == AVERAGED_MEASUREMENT) {
        /* Calculate the averaged adc raw value */
        (*data)[0] = (avg_sample_data / samples_cnt);
    }

    return SUCCESS;
}

/**
 * @brief Select (enable/disable) excitation sources for RTD measurement
 * @param enable_status[in] - Iout enable status
 * @param rtd_config_id[in] - RTD type (2/3/4-wire)
 * @param chn[in] - ADC channel assigned to given RTD sensor
 * @param multiple_3wire_rtd_enabled[in] - Multiple RTD enable status
 * @return SUCCESS/FAILURE
 */
int32_t select_rtd_excitation_sources(bool enable_status,
                                      uint32_t rtd_config_id,
                                      uint8_t chn, 
                                      bool multiple_3wire_rtd_enabled) {
    int32_t iout0_exc, iout1_exc;
    uint8_t arr_indx = 0;

    const std::array<std::array<uint8_t, MAX_RTD_SENSORS>, 3> rtd_iout0_source = {{
        {RTD1_2WIRE_IOUT0, RTD2_2WIRE_IOUT0, RTD3_2WIRE_IOUT0, RTD4_2WIRE_IOUT0, RTD5_2WIRE_IOUT0},
        {RTD1_3WIRE_IOUT0, RTD2_3WIRE_IOUT0, RTD3_3WIRE_IOUT0, RTD4_3WIRE_IOUT0, RTD5_3WIRE_IOUT0},
        {RTD1_4WIRE_IOUT0, RTD2_4WIRE_IOUT0, RTD3_4WIRE_IOUT0, RTD4_4WIRE_IOUT0, RTD5_4WIRE_IOUT0}
    }};

    const std::array<uint8_t, MAX_RTD_SENSORS> rtd_3wire_iout1_source = {
        RTD1_3WIRE_IOUT1, RTD2_3WIRE_IOUT1, RTD3_3WIRE_IOUT1, RTD4_3WIRE_IOUT1
    };

    /* Get the index mapped to RTD config ID */
    switch (rtd_config_id) {
    case AD7124_CONFIG_2WIRE_RTD:
        arr_indx = 0;
        break;
    case AD7124_CONFIG_3WIRE_RTD:
        arr_indx = 1;
        break;
    case AD7124_CONFIG_4WIRE_RTD:
        arr_indx = 2;
        break;
    default:
        break;
    }

    /* Select excitation source based on RTD configuration */
    if (multiple_3wire_rtd_enabled) {
        iout0_exc = RTD_IOUT0_250UA_EXC;
        iout1_exc = RTD_IOUT1_250UA_EXC;
    } else {
        iout0_exc = RTD_IOUT0_500UA_EXC;
        iout1_exc = RTD_IOUT1_500UA_EXC;
    }

    if (enable_status) {
        /* Enable and direct IOUT0 excitation current source for current RTD sensor measurement */
        ad7124_register_map[AD7124_IOCon1].value |= (AD7124_IO_CTRL1_REG_IOUT_CH0(
                    rtd_iout0_source[arr_indx][chn]) | AD7124_IO_CTRL1_REG_IOUT0(iout0_exc));

        if (rtd_config_id == AD7124_CONFIG_3WIRE_RTD) {
            /* Enable and direct IOUT1 excitation current source for 3-wire RTD measurement */
            ad7124_register_map[AD7124_IOCon1].value |= (AD7124_IO_CTRL1_REG_IOUT_CH1(
                        rtd_3wire_iout1_source[chn]) | AD7124_IO_CTRL1_REG_IOUT1(iout1_exc));
        }
    } else {
        /* Turn off the excitation currents */
        ad7124_register_map[AD7124_IOCon1].value &= ((~AD7124_IO_CTRL1_REG_IOUT0_MSK)
                & (~AD7124_IO_CTRL1_REG_IOUT_CH0_MSK));

        if (rtd_config_id == AD7124_CONFIG_3WIRE_RTD) {
            ad7124_register_map[AD7124_IOCon1].value &= ((~AD7124_IO_CTRL1_REG_IOUT1_MSK)
                    & (~AD7124_IO_CTRL1_REG_IOUT_CH1_MSK));
        }
    }

    if (ad7124_write_register(p_ad7124_dev, ad7124_register_map[AD7124_IOCon1]) != SUCCESS) {
        return FAILURE;
    }

    return SUCCESS;
}

/**
 * @brief Perform the ADC sampling for selected RTD sensor channel
 * @param rtd_config_id[in] - RTD type (2/3/4-wire)
 * @param chn[in] - ADC channel assigned to given RTD sensor
 * @param adc_raw[out] - ADC raw result
 * @param measurement_type[in] - Temperature measurement and display type
 * @param multiple_3wire_rtd_enabled[in] - Multiple RTD enable status
 * @return RTD sensor ADC sampling status
 */
bool do_rtd_sensor_adc_sampling(uint32_t rtd_config_id, 
                                uint8_t chn,
                                int32_t (*adc_raw)[MAX_ADC_SAMPLES], 
                                sensor_measurement_type measurement_type,
                                bool multiple_3wire_rtd_enabled) {
    bool adc_sampling_status = true;
    uint8_t setup = ad7124_get_channel_setup(p_ad7124_dev, chn);

    do {
        /* Apply previous calibration coefficients while performing new measurement */
        if (adc_calibration_config.adc_calibration_done) {
            ad7124_register_map[AD7124_Gain_0 + setup].value =
                adc_calibration_config.gain_after_calib[chn];
            if (ad7124_write_register(p_ad7124_dev,
                                      ad7124_register_map[AD7124_Gain_0 + setup]) != SUCCESS) {
                adc_sampling_status = false;
                break;
            }

            ad7124_register_map[AD7124_Offset_0 + setup].value =
                adc_calibration_config.offset_after_calib[chn];
            if (ad7124_write_register(p_ad7124_dev,
                                      ad7124_register_map[AD7124_Offset_0 + setup]) != SUCCESS) {
                adc_sampling_status = false;
                break;
            }
        }

        select_rtd_excitation_sources(true, rtd_config_id, chn, multiple_3wire_rtd_enabled);

        if (ad7124_write_register(p_ad7124_dev, ad7124_register_map[AD7124_IOCon1]) != SUCCESS) {
            adc_sampling_status = false;
            break;
        }

        if (perform_adc_conversion(chn, adc_raw, measurement_type) != SUCCESS) {
            adc_sampling_status = false;
            break;
        }

        select_rtd_excitation_sources(false, rtd_config_id, chn, multiple_3wire_rtd_enabled);
    } while (0);

    return adc_sampling_status;
}

/**
 * @brief Perform RTD temperature measurement and display
 * @param rtd_config_id[in] - RTD type (2/3/4-wire)
 * @param measurement_type[in] - Temperature measurement and display type
 * @return none
 */
void perform_rtd_temperature_measurement(uint32_t rtd_config_id,
                                         sensor_measurement_type measurement_type) {
    bool adc_error = false;
    uint8_t rtd_gain;
    uint16_t sample_cnt;
    bool continue_measurement = false;
    float temperature;

    if (measurement_type == CONTINUOUS_MEASUREMENT) {
        printf(EOL "Press ESC key once to stop measurement..." EOL);
        continue_measurement = true;
    }

    /* Print display header */
    printf(EOL EOL);
    for (uint8_t chn = SENSOR_CHANNEL0; chn < max_supported_sensors[rtd_config_id]; chn++) {
        if (sensor_enable_status[chn]) {
            printf("\tRTD%d   ", chn + 1);
        }
    }
    printf(EOL "\t-----------------------------------------------" EOL EOL);

    /* Perform additional configs for 3-wire RTD measurement */
    if (rtd_config_id == AD7124_CONFIG_3WIRE_RTD) {
        rtd_gain = MULTI_3WIRE_RTD_GAIN;
    } else {
        rtd_gain = RTD_4WIRE_GAIN_VALUE;
    }

    do {
        /* Sample and Read all enabled RTD channels in sequence */
        for (uint8_t chn = SENSOR_CHANNEL0; chn < max_supported_sensors[rtd_config_id]; chn++) {
            if (sensor_enable_status[chn]) {
                if (!do_rtd_sensor_adc_sampling(rtd_config_id, chn, &n_sample_data[chn], 
                                                 measurement_type, false)) {
                    adc_error = true;
                    break;
                }
            }
        }

        if (adc_error) {
            printf(EOL EOL "\tError Performing Measurement" EOL);
            break;
        } else {
            /* Calculate temperature and display result */
            if (measurement_type == AVERAGED_MEASUREMENT) {
                for (uint8_t chn = SENSOR_CHANNEL0; chn < max_supported_sensors[rtd_config_id]; chn++) {
                    if (sensor_enable_status[chn]) {
                        temperature = get_rtd_temperature(n_sample_data[chn][0], rtd_gain);
                        sprintf(decimal_eqv_str.data(), "%.4f  ", temperature);
                        strcat(decimal_eqv_str_arr.data(), decimal_eqv_str.data());
                    }
                }
                printf("\t%s" EOL EOL, decimal_eqv_str_arr.data());
                decimal_eqv_str_arr[0] = '\0';
            } else {
                for (sample_cnt = 0; sample_cnt < MAX_ADC_SAMPLES; sample_cnt++) {
                    for (uint8_t chn = SENSOR_CHANNEL0; chn < max_supported_sensors[rtd_config_id]; chn++) {
                        if (sensor_enable_status[chn]) {
                            temperature = get_rtd_temperature(n_sample_data[chn][sample_cnt], rtd_gain);
                            sprintf(decimal_eqv_str.data(), "%.4f  ", temperature);
                            strcat(decimal_eqv_str_arr.data(), decimal_eqv_str.data());
                        }
                    }
                    printf("\t%s" EOL EOL, decimal_eqv_str_arr.data());
                    decimal_eqv_str_arr[0] = '\0';
                }
            }
        }
    } while (continue_measurement && !was_escape_key_pressed());

    adi_press_any_key_to_continue();
    adi_clear_console();
}

/**
 * @brief Perform the device configurations required for ADC calibration
 * @return ADC calibration configuration status
 */
int32_t do_adc_calibration_configs() {
    int32_t adc_config_status = SUCCESS;

    do {
        /* Put ADC into standby mode */
        ad7124_register_map[AD7124_ADC_Control].value &= (~AD7124_ADC_CTRL_REG_MSK);
        ad7124_register_map[AD7124_ADC_Control].value |= AD7124_ADC_CTRL_REG_MODE(STANDBY_MODE);

        /* Get ADC power mode status for previous config */
        adc_calibration_config.power_mode = AD7124_ADC_CTRL_REG_POWER_MODE_RD(
                ad7124_register_map[AD7124_ADC_Control].value);

        /* Select low power ADC mode for ADC calibration */
        ad7124_register_map[AD7124_ADC_Control].value &= (~AD7124_ADC_CTRL_REG_POWER_MODE_MSK);
        ad7124_register_map[AD7124_ADC_Control].value |= AD7124_ADC_CTRL_REG_POWER_MODE(LOW_POWER_MODE);

        if (ad7124_write_register(p_ad7124_dev, ad7124_register_map[AD7124_ADC_Control]) != SUCCESS) {
            adc_config_status = FAILURE;
            break;
        }
    } while (0);

    return adc_config_status;
}

/**
 * @brief Reset the ADC configuration to previous demo mode configuration
 * @return none
 */
void reset_adc_calibration_configs() {
    /* Put ADC into standby mode */
    ad7124_register_map[AD7124_ADC_Control].value &= (~AD7124_ADC_CTRL_REG_MSK);
    ad7124_register_map[AD7124_ADC_Control].value |= AD7124_ADC_CTRL_REG_MODE(STANDBY_MODE);

    /* Reset ADC power mode */
    ad7124_register_map[AD7124_ADC_Control].value &= (~AD7124_ADC_CTRL_REG_POWER_MODE_MSK);
    ad7124_register_map[AD7124_ADC_Control].value |= AD7124_ADC_CTRL_REG_POWER_MODE(
                adc_calibration_config.power_mode);

    ad7124_write_register(p_ad7124_dev, ad7124_register_map[AD7124_ADC_Control]);
}

/**
 * @brief Perform the ADC calibration for given channel
 * @param calibration_mode[in] - Calibration mode
 * @param chn[in] - ADC channel
 * @param setup[in] - Setup mapped to selected ADC channel
 * @param pos_analog_input[in] - Positive analog input mapped to selected ADC channel
 * @param neg_analog_input[in] - Negative analog input mapped to selected ADC channel
 * @return ADC calibration status
 */
int32_t do_adc_calibration(uint32_t calibration_mode, 
                           uint8_t chn,
                           uint8_t setup,
                           uint8_t pos_analog_input, 
                           uint8_t neg_analog_input) {
    int32_t calibration_status = SUCCESS;
    uint8_t pga = AD7124_PGA_GAIN(ad7124_get_channel_pga(p_ad7124_dev, chn));

    do {
        if ((calibration_mode == INTERNAL_FULL_SCALE_CALIBRATE_MODE) ||
            (calibration_mode == INTERNAL_ZERO_SCALE_CALIBRATE_MODE)) {
            
            if (calibration_mode == INTERNAL_FULL_SCALE_CALIBRATE_MODE) {
                /* Write default offset register value before starting full-scale internal calibration */
                ad7124_register_map[AD7124_Offset_0 + setup].value = AD7124_DEFAULT_OFFSET;
                if (ad7124_write_register(p_ad7124_dev,
                                          ad7124_register_map[AD7124_Offset_0 + setup]) != SUCCESS) {
                    calibration_status = FAILURE;
                    break;
                }

                /* Don't continue further internal full-scale calibration at gain of 1 */
                if (pga == 1) {
                    printf("\tDevice does not support internal full-scale calibration at Gain of 1!!" EOL);
                    break;
                } else {
                    printf("\tRunning internal full-scale (gain) calibration..." EOL);
                }
            } else {
                printf("\tRunning internal zero-scale (offset) calibration..." EOL);
            }
        } else {
            if (calibration_mode == SYSTEM_FULL_SCALE_CALIBRATE_MODE) {
                printf(EOL "\tApply full-scale input voltage and press any key to continue..." EOL);
            } else {
                printf(EOL "\tApply zero-scale input voltage and press any key to continue..." EOL);
            }

            /* Wait for user input */
            getchar();
        }

        /* Get setup/configuration mapped to corresponding channel */
        setup = AD7124_CH_MAP_REG_SETUP_RD(ad7124_register_map[AD7124_Channel_0 + chn].value);

        if ((calibration_mode == INTERNAL_FULL_SCALE_CALIBRATE_MODE) ||
            (calibration_mode == SYSTEM_FULL_SCALE_CALIBRATE_MODE)) {
            /* Read the gain coefficient value */
            if (ad7124_read_register(p_ad7124_dev,
                                     &ad7124_register_map[AD7124_Gain_0 + setup]) != SUCCESS) {
                calibration_status = FAILURE;
                break;
            }
            adc_calibration_config.gain_before_calib[chn] =
                ad7124_register_map[AD7124_Gain_0 + setup].value;
        }

        if ((calibration_mode == INTERNAL_ZERO_SCALE_CALIBRATE_MODE) ||
            (calibration_mode == SYSTEM_ZERO_SCALE_CALIBRATE_MODE)) {
            /* Read the offset coefficient value */
            if (ad7124_read_register(p_ad7124_dev,
                                     &ad7124_register_map[AD7124_Offset_0 + setup]) != SUCCESS) {
                calibration_status = FAILURE;
                break;
            }
            adc_calibration_config.offset_before_calib[chn] =
                ad7124_register_map[AD7124_Offset_0 + setup].value;
        }

        ad7124_register_map[AD7124_ADC_Control].value =
            ((ad7124_register_map[AD7124_ADC_Control].value & ~AD7124_ADC_CTRL_REG_MSK) |
             AD7124_ADC_CTRL_REG_MODE(calibration_mode));

        if (ad7124_write_register(p_ad7124_dev, ad7124_register_map[AD7124_ADC_Control]) != SUCCESS) {
            calibration_status = FAILURE;
            break;
        }

        /* Let the channel settle */
        mdelay(100);

        /* Wait for calibration (conversion) to finish */
        if (ad7124_wait_for_conv_ready(p_ad7124_dev, p_ad7124_dev->spi_rdy_poll_cnt) != SUCCESS) {
            calibration_status = FAILURE;
            break;
        }
    } while (0);

    return calibration_status;
}

/**
 * @brief Perform the ADC internal/system calibration
 * @param calibration_type[in] - Type of calibration (internal/system)
 * @return SUCCESS/FAILURE
 */
int32_t perform_adc_calibration(uint32_t calibration_type) {
    bool adc_error = false;
    uint8_t chn_cnt;
    uint8_t pos_analog_input, neg_analog_input;
    uint8_t setup;
    uint8_t pga;

    /* Load ADC configurations and perform the calibration */
    if (do_adc_calibration_configs() == SUCCESS) {
        /* Calibrate all the user enabled ADC channels sequentially */
        for (chn_cnt = 0; chn_cnt < NUM_OF_SENSOR_CHANNELS; chn_cnt++) {
            if (sensor_enable_status[chn_cnt]) {
                /* Get the positive and negative analog inputs mapped to channel */
                pos_analog_input = AD7124_CH_MAP_REG_AINP_RD(
                        ad7124_register_map[AD7124_Channel_0 + chn_cnt].value);
                neg_analog_input = AD7124_CH_MAP_REG_AINM_RD(
                        ad7124_register_map[AD7124_Channel_0 + chn_cnt].value);

                /* Get setup mapped to corresponding channel */
                setup = AD7124_CH_MAP_REG_SETUP_RD(
                        ad7124_register_map[AD7124_Channel_0 + chn_cnt].value);

                /* Get the programmable gain mapped to corresponding channels setup */
                pga = AD7124_PGA_GAIN(ad7124_get_channel_pga(p_ad7124_dev, chn_cnt));

                printf(EOL "Calibrating Channel %d => " EOL, chn_cnt);

                /* Enable channel for calibration */
                ad7124_register_map[AD7124_Channel_0 + chn_cnt].value |= AD7124_CH_MAP_REG_CH_EN;
                if (ad7124_write_register(p_ad7124_dev,
                                          ad7124_register_map[AD7124_Channel_0 + chn_cnt]) != SUCCESS) {
                    adc_error = true;
                    break;
                }

                // Handle different sensor configurations for calibration
                if ((current_sensor_config_id == AD7124_CONFIG_2WIRE_RTD) ||
                    (current_sensor_config_id == AD7124_CONFIG_3WIRE_RTD) ||
                    (current_sensor_config_id == AD7124_CONFIG_4WIRE_RTD)) {
                    /* Enable the Iout source on channel */
                    select_rtd_excitation_sources(true, current_sensor_config_id, chn_cnt, true);
                } else if (current_sensor_config_id == AD7124_CONFIG_THERMOCOUPLE) {
                    if ((chn_cnt == CJC_RTD_CHN) || (chn_cnt == CJC_THERMISTOR_CHN)) {
                        do_cjc_configs(chn_cnt);
                    }
                }

                if (calibration_type == INTERNAL_CALIBRATION) {
                    /* Perform the internal full-scale (gain) calibration */
                    if (do_adc_calibration(INTERNAL_FULL_SCALE_CALIBRATE_MODE,
                                           chn_cnt, setup, pos_analog_input, neg_analog_input) != SUCCESS) {
                        adc_error = true;
                        break;
                    }

                    /* Read the gain coefficient value (post calibrated) */
                    ad7124_read_register(p_ad7124_dev, &ad7124_register_map[AD7124_Gain_0 + setup]);
                    adc_calibration_config.gain_after_calib[chn_cnt] =
                        ad7124_register_map[AD7124_Gain_0 + setup].value;

                    /* Perform the internal zero-scale (offset) calibration */
                    if (do_adc_calibration(INTERNAL_ZERO_SCALE_CALIBRATE_MODE,
                                           chn_cnt, setup, pos_analog_input, neg_analog_input) != SUCCESS) {
                        adc_error = true;
                        break;
                    }

                    /* Read the offset coefficient value (post calibrated) */
                    ad7124_read_register(p_ad7124_dev, &ad7124_register_map[AD7124_Offset_0 + setup]);
                    adc_calibration_config.offset_after_calib[chn_cnt] =
                        ad7124_register_map[AD7124_Offset_0 + setup].value;

                    /* Compare the pre and post adc calibration gain coefficients to check calibration status */
                    if (pga > 1) {
                        if (adc_calibration_config.gain_after_calib[chn_cnt] !=
                            adc_calibration_config.gain_before_calib[chn_cnt]) {
                            printf("\tGain %d: 0x%lx" EOL, setup,
                                   adc_calibration_config.gain_after_calib[chn_cnt]);
                        } else {
                            printf(EOL "\tError in internal full-scale (gain) calibration!!" EOL);
                            adc_calibration_config.gain_after_calib[chn_cnt] =
                                adc_calibration_config.gain_before_calib[chn_cnt];
                        }
                    }

                    /* Compare the pre and post adc calibration offset coefficients to check calibration status */
                    if (adc_calibration_config.offset_after_calib[chn_cnt] !=
                        adc_calibration_config.offset_before_calib[chn_cnt]) {
                        printf("\tOffset %d: 0x%lx" EOL, setup,
                               adc_calibration_config.offset_after_calib[chn_cnt]);
                    } else {
                        printf(EOL "\tError in internal zero-scale (offset) calibration!!" EOL);
                        adc_calibration_config.offset_after_calib[chn_cnt] =
                            adc_calibration_config.offset_before_calib[chn_cnt];
                    }
                } else {
                    // System calibration logic here...
                }

                // Disable excitation sources after calibration
                if ((current_sensor_config_id == AD7124_CONFIG_2WIRE_RTD) ||
                    (current_sensor_config_id == AD7124_CONFIG_3WIRE_RTD) ||
                    (current_sensor_config_id == AD7124_CONFIG_4WIRE_RTD)) {
                    /* Disable the Iout source on RTD channel */
                    select_rtd_excitation_sources(false, current_sensor_config_id, chn_cnt, true);
                } else {
                    /* Turn off the Iout0 excitation current */
                    ad7124_register_map[AD7124_IOCon1].value &= ((~AD7124_IO_CTRL1_REG_IOUT0_MSK)
                            & (~AD7124_IO_CTRL1_REG_IOUT_CH0_MSK));
                    ad7124_write_register(p_ad7124_dev, ad7124_register_map[AD7124_IOCon1]);
                }

                /* Disable current channel */
                ad7124_register_map[AD7124_Channel_0 + chn_cnt].value &= (~AD7124_CH_MAP_REG_CH_EN);
                if (ad7124_write_register(p_ad7124_dev,
                                          ad7124_register_map[AD7124_Channel_0 + chn_cnt]) != SUCCESS) {
                    adc_error = true;
                    break;
                }

                if (!adc_error) {
                    printf(EOL "\tCalibration done..." EOL);
                }
            }
        }

        adc_calibration_config.adc_calibration_done = !adc_error;
    } else {
        printf(EOL "\tError in calibration!!" EOL);
        adc_calibration_config.adc_calibration_done = false;
    }

    /* Reset the ADC configs to previously enabled config to apply calibration
     * offset and gain coefficients */
    reset_adc_calibration_configs();

    adi_press_any_key_to_continue();
    adi_clear_console();

    return SUCCESS;
}

/**
 * @brief Display the sensor enable/disable status
 * @return none
 */
void display_sensor_enable_disable_status() {
    const std::array<const char*, 2> status_info = {"Disabled", "Enabled"};

    printf(EOL "\tSensor Enable/Disable Status" EOL);
    printf("\t----------------------------------------------------" EOL);
    printf("\tCh0: %s \tCh1: %s \tCh2: %s \tCh3: %s" EOL,
           status_info[sensor_enable_status[SENSOR_CHANNEL0]],
           status_info[sensor_enable_status[SENSOR_CHANNEL1]],
           status_info[sensor_enable_status[SENSOR_CHANNEL2]],
           status_info[sensor_enable_status[SENSOR_CHANNEL3]]);
    printf("\tCh4: %s \tCh5: %s \tCh6: %s \tCh7: %s" EOL,
           status_info[sensor_enable_status[SENSOR_CHANNEL4]],
           status_info[sensor_enable_status[SENSOR_CHANNEL5]],
           status_info[sensor_enable_status[SENSOR_CHANNEL6]],
           status_info[sensor_enable_status[SENSOR_CHANNEL7]]);
    printf("\t----------------------------------------------------" EOL);
    printf("\t*Note: The AD7124 is factory calibrated at a gain of 1, and the resulting gain coefficient" EOL
           "\t       is the default gain coefficient on the device. The device does not support further" EOL
           "\t       internal full-scale calibrations at a gain of 1" EOL);
}