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

/** @defgroup GAF EDMP GAF
 *  @brief High-level functions to drive eDMP Gyro-Assisted Fusion feature
 *  @{
 */

/** @file inv_imu_edmp_gaf.h */

#ifndef _INV_IMU_GAF_H_
#define _INV_IMU_GAF_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "imu/inv_imu_driver.h"

/** @brief IMU GAF parameters definition
 */
typedef struct {
	/** Error on the clock in same format as reg SW_PLL1_TRIM.
	 *  Calculated as (actual_clk - target_clk) / target_clk * (2^7 - 1) / 5 * 100.
	 */
	int8_t clock_variation;

	/** Processing data rate in us (corresponds to GAF output data rate). */
	uint32_t pdr_us;

	/** Enable/disable stop integration of stationary angle */
	int32_t stationary_angle_enable;

	/** Duration of stationary detection */
	int32_t stationary_angle_duration_us;

	/** Threshold on angular deviation for stationary detection */
	int32_t stationary_angle_threshold_deg_q16;

	/** Duration for no motion gyro bias calibration */
	int32_t gyr_cal_stationary_duration_us;

	/** Stationary detection threshold of 1st metric */
	int32_t gyr_cal_threshold_metric1;

	/** Stationary detection threshold of 2nd metric */
	int32_t gyr_cal_threshold_metric2;

	/** Percentage of error on gyr integration (1% = 32767).
	 *  This error covers gyro sensitivity, timestamp, and quantization.
	 */
	int32_t fus_high_speed_drift;

	/** Gyr integration error related to bias precision. 
	 *  Higher value increase accel roll/pitch correction in steady state
	 */
	int32_t fus_low_speed_drift_roll_pitch;

	/** Accel measurement covariance in q15 (1G^2 = 32767) */
	int32_t fus_measurement_covariance_acc;

	/** Linear acceleration rejection.
	 *  Max is 100% = 1073741824L. No rejection is 0% = 0L.
	 */
	int32_t fus_acceleration_rejection;

	/** Mounting matrix 3x3 in Q14 to be applied to accel and gyro data before feeding eDMP GAF algorithm */
	int16_t mmatrix[9];

	/** Accuracy of gyroscope biases when explicitly given by user */
	int8_t gyr_bias_accuracy;
} inv_imu_edmp_gaf_parameters_t;

/** @brief IMU GAF outputs definition
 */
typedef struct {
	/** 6-axis (accel and gyro fusion) {w, x, y, z} quaternion */
	int32_t grv_quat_q30[4];

	/** Gyro bias {x, y, z} (1 dps = 1<<16)*/
	int32_t gyr_bias_q16[3];

	/** Gyro accuracy, from 0 (non calibrated) to 3 (well calibrated) */
	int8_t gyr_accuracy_flag;

	/** Stationary detection based on gyro data */
	int8_t stationary_flag;
} inv_imu_edmp_gaf_outputs_t;

/** @brief  Initialize GAF algorithm. 
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_gaf_init(inv_imu_device_t *s);

/** @brief  Initialize GAF configuration with default settings.
 *  @param[in]  s           Pointer to device.
 *  @param[out] gaf_params  GAF parameters.
 *  @return                 0 on success, negative value on error.
 */
int inv_imu_edmp_gaf_init_parameters(inv_imu_device_t *             s,
                                     inv_imu_edmp_gaf_parameters_t *gaf_params);

/** @brief  Get current GAF configuration settings.
 *  @param[in]  s           Pointer to device.
 *  @param[out] gaf_params  GAF parameters.
 *  @return                 0 on success, negative value on error.
 */
int inv_imu_edmp_gaf_get_parameters(inv_imu_device_t *s, inv_imu_edmp_gaf_parameters_t *gaf_params);

/** @brief  Set current GAF configuration settings.
 *  @param[in]  s           Pointer to device.
 *  @param[in]  gaf_params  GAF parameters.
 *  @return                 0 on success, negative value on error.
 */
int inv_imu_edmp_gaf_set_parameters(inv_imu_device_t *                   s,
                                    const inv_imu_edmp_gaf_parameters_t *gaf_params);

/** @brief  Enable APEX algorithm GAF.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_gaf_enable(inv_imu_device_t *s);

/** @brief  Disable APEX algorithm GAF.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_gaf_disable(inv_imu_device_t *s);

/** @brief  Build GAF output value based on several FIFO frames. 
 *  @param[in]  s            Pointer to device.
 *  @param[in]  frame_es0    FIFO frame field of ES data, typically es0 field of @sa inv_imu_sensor_event_t.
 *  @param[out] gaf_outputs  Pointer to GAF output structure, which will hold GAF latest values computed.
 *  @return                  1 on success and all FIFO frames were received to rebuild gaf_outputs,
 *                           0 on success but gaf_outputs is not complete yet,
 *                           negative value on error.
 */
int inv_imu_edmp_gaf_build_outputs(inv_imu_device_t *s, const uint8_t frame_es0[9],
                                   inv_imu_edmp_gaf_outputs_t *gaf_outputs);

/** @brief  Set accelerometer/gyroscope bias for GAF running on EDMP
 *  @param[in]  s           Pointer to device.
 *  @param[in]  acc_bias    Accelerometer bias to be set in EDMP, provided in s32q16 (i.e. 1g = 1<<16), as read from EDMP, if NULL it will not be set
 *  @param[in]  gyr_bias    Gyroscope bias to be set in EDMP, provided in s32q16 (i.e. 1dps = 1<<16), as read from EDMP, if NULL it will not be set
 *  @return                 0 on success, negative value on error.
 *  @warning
 *  This function shall be called just after `inv_imu_edmp_enable` without any further delay.
 */
int inv_imu_edmp_gaf_set_biases(inv_imu_device_t *s, const int32_t acc_bias[3],
                                const int32_t gyr_bias[3]);

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_GAF_H_ */

/** @} */
