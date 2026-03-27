/*
 *
 * Copyright (c) [2020] by InvenSense, Inc.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include "imu/inv_imu_edmp.h"
#include "imu/inv_imu_edmp_gaf.h"
#include "imu/inv_imu_edmp_defs.h"
#include "imu/inv_imu_edmp_ram_gaf_defs.h"

/* GAF specific memory map */
#define EDMP_GAF_STATE_CONFIG         (0x1674)
#define EDMP_GAF_STATE_CONFIG_SIZE    (0x44)
#define EDMP_GAF_STATE_INIT           (0x18C6)
#define EDMP_GAF_STATE_INIT_SIZE      (0x1)
#define EDMP_GAF_MOUNTING_MATRIX      (0x1A30)
#define EDMP_GAF_MOUNTING_MATRIX_SIZE (0x12)
#define EDMP_GAF_STATE_GYRO_BIAS      (0x1800)
#define EDMP_GAF_STATE_GYRO_BIAS_SIZE (0xC)
#define EDMP_GAF_STATE_ACC_BIAS       (0x18C8)
#define EDMP_GAF_STATE_ACC_BIAS_SIZE  (0xC)

#define GYRO_2000DPS_Q30_TO_1DPS_Q16 131072000LL /* 2000dps * (1<<16) */

/* Fits in eDMP memory at EDMP_GAF_STATE_CONFIG address */
typedef struct {
	int32_t acc_fsr;
	int32_t gyr_fsr;

	union {
		uint32_t acc_odr_us;
		uint32_t gyr_odr_us;
	};

	int32_t temp_sensitivity;
	int32_t temp_offset;

	int8_t acc_accuracy;
	int8_t gyr_accuracy;

	int8_t clock_variation;

	union {
		uint32_t acc_pdr_us;
		uint32_t gyr_pdr_us;
	};

	int32_t stationary_angle_enable;
	int32_t stationary_angle_duration_us;
	int32_t stationary_angle_threshold_deg_q16;

	int32_t gyr_cal_stationary_duration_us;
	int32_t gyr_cal_threshold_metric1;
	int32_t gyr_cal_threshold_metric2;

	int32_t fus_high_speed_drift;
	int32_t fus_low_speed_drift_roll_pitch;
	int32_t fus_measurement_covariance_acc;
	int32_t fus_acceleration_rejection;
} internal_gaf_parameters_t;

int inv_imu_edmp_gaf_init(inv_imu_device_t *s)
{
	int            status = INV_IMU_OK;
	uint8_t        value;
	static uint8_t ram_img[] = {
#include "imu/edmp_prgm_gaf.h"
	};

	status |= inv_imu_edmp_init_apex(s);

	/* Clear SRAM data */
	value = 0;
	for (int i = 0; i < RAM_GAF_DATA_SIZE; i++)
		status |= inv_imu_write_sram(s, (uint32_t)RAM_GAF_DATA_BASE + i, 1, &value);

	status |= inv_imu_write_sram(s, (uint32_t)RAM_GAF_PRGM_BASE, sizeof(ram_img), ram_img);

	return status;
}

int inv_imu_edmp_gaf_init_parameters(inv_imu_device_t *s, inv_imu_edmp_gaf_parameters_t *gaf_params)
{
	int status = INV_IMU_OK;

	status |= inv_imu_read_reg(s, SW_PLL1_TRIM, 1, (uint8_t *)&gaf_params->clock_variation);

	gaf_params->pdr_us                             = 10000;
	gaf_params->stationary_angle_enable            = 0;
	gaf_params->stationary_angle_duration_us       = 500000;
	gaf_params->stationary_angle_threshold_deg_q16 = 65536;
	gaf_params->gyr_cal_stationary_duration_us     = 500000;
	gaf_params->gyr_cal_threshold_metric1          = 300;
	gaf_params->gyr_cal_threshold_metric2          = 400;
	gaf_params->fus_high_speed_drift               = 262144;
	gaf_params->fus_low_speed_drift_roll_pitch     = 20;
	gaf_params->fus_measurement_covariance_acc     = 32768;
	gaf_params->fus_acceleration_rejection         = 1073741824;
	/* Identity matrix in Q14 */
	gaf_params->mmatrix[0] = 1 << 14;
	gaf_params->mmatrix[1] = 0;
	gaf_params->mmatrix[2] = 0;
	gaf_params->mmatrix[3] = 0;
	gaf_params->mmatrix[4] = 1 << 14;
	gaf_params->mmatrix[5] = 0;
	gaf_params->mmatrix[6] = 0;
	gaf_params->mmatrix[7] = 0;
	gaf_params->mmatrix[8] = 1 << 14;

	return status;
}

int inv_imu_edmp_gaf_get_parameters(inv_imu_device_t *s, inv_imu_edmp_gaf_parameters_t *gaf_params)
{
	int                       status = INV_IMU_OK;
	internal_gaf_parameters_t params;

	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_GAF_STATE_CONFIG, (uint8_t *)&params);

	gaf_params->clock_variation                    = params.clock_variation;
	gaf_params->pdr_us                             = params.acc_pdr_us;
	gaf_params->stationary_angle_enable            = params.stationary_angle_enable;
	gaf_params->stationary_angle_duration_us       = params.stationary_angle_duration_us;
	gaf_params->stationary_angle_threshold_deg_q16 = params.stationary_angle_threshold_deg_q16;
	gaf_params->gyr_cal_stationary_duration_us     = params.gyr_cal_stationary_duration_us;
	gaf_params->gyr_cal_threshold_metric1          = params.gyr_cal_threshold_metric1;
	gaf_params->gyr_cal_threshold_metric2          = params.gyr_cal_threshold_metric2;
	gaf_params->fus_high_speed_drift               = params.fus_high_speed_drift;
	gaf_params->fus_low_speed_drift_roll_pitch     = params.fus_low_speed_drift_roll_pitch;
	gaf_params->fus_measurement_covariance_acc     = params.fus_measurement_covariance_acc;
	gaf_params->fus_acceleration_rejection         = params.fus_acceleration_rejection;

	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_GAF_MOUNTING_MATRIX, (uint8_t *)gaf_params->mmatrix);

	return status;
}

int inv_imu_edmp_gaf_set_parameters(inv_imu_device_t *                   s,
                                    const inv_imu_edmp_gaf_parameters_t *gaf_params)
{
	int                       status = INV_IMU_OK;
	internal_gaf_parameters_t params = {
		.acc_fsr                            = 32,
		.gyr_fsr                            = 4000,
		.temp_sensitivity                   = 8388608,
		.temp_offset                        = 1638400,
		.acc_accuracy                       = 0,
		.gyr_accuracy                       = gaf_params->gyr_bias_accuracy,
		.clock_variation                    = gaf_params->clock_variation,
		.acc_pdr_us                         = gaf_params->pdr_us,
		.stationary_angle_enable            = gaf_params->stationary_angle_enable,
		.stationary_angle_duration_us       = gaf_params->stationary_angle_duration_us,
		.stationary_angle_threshold_deg_q16 = gaf_params->stationary_angle_threshold_deg_q16,
		.gyr_cal_stationary_duration_us     = gaf_params->gyr_cal_stationary_duration_us,
		.gyr_cal_threshold_metric1          = gaf_params->gyr_cal_threshold_metric1,
		.gyr_cal_threshold_metric2          = gaf_params->gyr_cal_threshold_metric2,
		.fus_high_speed_drift               = gaf_params->fus_high_speed_drift,
		.fus_low_speed_drift_roll_pitch     = gaf_params->fus_low_speed_drift_roll_pitch,
		.fus_measurement_covariance_acc     = gaf_params->fus_measurement_covariance_acc,
		.fus_acceleration_rejection         = gaf_params->fus_acceleration_rejection
	};

	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_STATE_CONFIG, (uint8_t *)&params);

	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_MOUNTING_MATRIX, (uint8_t *)gaf_params->mmatrix);

	return status;
}

int inv_imu_edmp_gaf_enable(inv_imu_device_t *s)
{
	int             status   = INV_IMU_OK;
	uint8_t         gaf_init = 0;
	fifo_config0_t  fifo_config0;
	edmp_apex_en1_t edmp_apex_en1;

	/*
	 * Force reinitilization of algorithm because user might have changed GAF parameters between
	 * call	to `inv_imu_init_gaf()` and call to `inv_imu_edmp_gaf_enable()`.
	 * If this is not done, new user parameters won't be applied.
	 */
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_STATE_INIT, (uint8_t *)&gaf_init);

	/* Only 1.2k FIFO is supported if GAF RAM image is loaded */
	status |= inv_imu_read_reg(s, FIFO_CONFIG0, 1, (uint8_t *)&fifo_config0);
	if (fifo_config0.fifo_depth > FIFO_CONFIG0_FIFO_DEPTH_GAF)
		return INV_IMU_ERROR;

	/*
	 * Enable RAM feature in EDMP_APEX_EN1 so that ROM executes GAF RAM image.
	 */
	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	edmp_apex_en1.feature3_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);

	return status;
}

int inv_imu_edmp_gaf_disable(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en1_t edmp_apex_en1;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	edmp_apex_en1.feature3_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);

	return status;
}

int inv_imu_edmp_gaf_build_outputs(inv_imu_device_t *s, const uint8_t frame_es0[9],
                                   inv_imu_edmp_gaf_outputs_t *gaf_outputs)
{
	/* Frame ID is in last byte of ES FIFO frame and allows to decode frame content */
	switch (frame_es0[8]) {
	case RAM_GAF_FIFO_FRAME_ID_QUAT_START:
		gaf_outputs->grv_quat_q30[0] = ((int)frame_es0[3] << 24) + ((int)frame_es0[2] << 16) +
		                               ((int)frame_es0[1] << 8) + (int)frame_es0[0];
		gaf_outputs->grv_quat_q30[1] = ((int)frame_es0[7] << 24) + ((int)frame_es0[6] << 16) +
		                               ((int)frame_es0[5] << 8) + (int)frame_es0[4];
		return 0;
	case RAM_GAF_FIFO_FRAME_ID_QUAT_END:
		gaf_outputs->grv_quat_q30[2] = ((int)frame_es0[3] << 24) + ((int)frame_es0[2] << 16) +
		                               ((int)frame_es0[1] << 8) + (int)frame_es0[0];
		gaf_outputs->grv_quat_q30[3] = ((int)frame_es0[7] << 24) + ((int)frame_es0[6] << 16) +
		                               ((int)frame_es0[5] << 8) + (int)frame_es0[4];
		return 0;
	case RAM_GAF_FIFO_FRAME_ID_BIAS_START:
		gaf_outputs->gyr_bias_q16[0] = ((int)frame_es0[3] << 24) + ((int)frame_es0[2] << 16) +
		                               ((int)frame_es0[1] << 8) + (int)frame_es0[0];
		gaf_outputs->gyr_bias_q16[1] = ((int)frame_es0[7] << 24) + ((int)frame_es0[6] << 16) +
		                               ((int)frame_es0[5] << 8) + (int)frame_es0[4];
		return 0;
	case RAM_GAF_FIFO_FRAME_ID_BIAS_END:
		gaf_outputs->gyr_bias_q16[2] = ((int)frame_es0[3] << 24) + ((int)frame_es0[2] << 16) +
		                               ((int)frame_es0[1] << 8) + (int)frame_es0[0];
		gaf_outputs->gyr_accuracy_flag = frame_es0[4];
		gaf_outputs->stationary_flag   = frame_es0[5];
		/* This is the last FIFO frame expected, gaf_outputs is now fully filled in */
		return 1;
	default:
		return INV_IMU_ERROR;
	}
}

int inv_imu_edmp_gaf_set_biases(inv_imu_device_t *s, const int32_t acc_bias[3],
                                const int32_t gyr_bias[3])
{
	/* Wait for GAF to fully initialise */
	int      status   = INV_IMU_OK;
	uint8_t  gaf_init = 0;
	uint32_t timeout  = 50000; /*10ms is one fast ODR, more than enough*/
	/* Input bias is given in 1dps q16 format and
	 * needs to be converted to 2000dps q30 format
	 * The logic to convert a value, is as follows:
	 *     val_q30 = (val_q16 << 30) / (2000dps << 16)
	 * <=> val_q30 = val_q16 * (2^14/2000)
	 * <=> val_q30 = val_q16 * 8.192
	 */
	const int32_t gyr_bias_edmp[3] = {
		((int64_t)gyr_bias[0] << 30LL) / GYRO_2000DPS_Q30_TO_1DPS_Q16,
		((int64_t)gyr_bias[1] << 30LL) / GYRO_2000DPS_Q30_TO_1DPS_Q16,
		((int64_t)gyr_bias[2] << 30LL) / GYRO_2000DPS_Q30_TO_1DPS_Q16
	};
	edmp_apex_en1_t edmp_apex_en1;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	if (!edmp_apex_en1.edmp_enable) {
		/* if EDMP isn't enabled, GAF init can't happen and this function shall exit early */
		timeout = 0;
	}

	while ((0 == gaf_init) && (0 != timeout)) {
		status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_GAF_STATE_INIT, (uint8_t *)&gaf_init);
		/* Only sleep and consider timeout decrement if initialization did not complete */
		if (0 == gaf_init) {
			inv_imu_sleep_us(s, 200);
			timeout -= 200;
		}
	}

	if ((0 == gaf_init) && (0 == timeout)) {
		status |= INV_IMU_ERROR_TIMEOUT;
	} else {
		if (gyr_bias) {
			status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_STATE_GYRO_BIAS,
			                                  (const uint8_t *)gyr_bias_edmp);
		}
		if (acc_bias) {
			status |=
			    INV_IMU_WRITE_EDMP_SRAM(s, EDMP_GAF_STATE_ACC_BIAS, (const uint8_t *)acc_bias);
		}
	}
	return status;
}
