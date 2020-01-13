/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


#include "PX4Accelerometer.hpp"

#include <lib/drivers/device/Device.hpp>

using namespace time_literals;
using matrix::Vector3f;

PX4Accelerometer::PX4Accelerometer(uint32_t device_id, uint8_t priority, enum Rotation rotation) :
	ModuleParams(nullptr),
	_sensor_pub{ORB_ID(sensor_accel), priority},
	_sensor_fifo_pub{ORB_ID(sensor_accel_fifo), priority},
	_sensor_status_pub{ORB_ID(sensor_accel_status), priority},
	_device_id{device_id},
	_rotation{rotation}
{
	// set software low pass filter for controllers
	updateParams();
	UpdateCalibration();

	ConfigureFilter(_param_imu_accel_cutoff.get());
}

void PX4Accelerometer::set_device_type(uint8_t devtype)
{
	// current DeviceStructure
	union device::Device::DeviceId device_id;
	device_id.devid = _device_id;

	// update to new device type
	device_id.devid_s.devtype = devtype;

	// copy back
	_device_id = device_id.devid;

	UpdateCalibration();
}

void PX4Accelerometer::set_sample_rate(uint16_t rate)
{
	_sample_rate = rate;

	ConfigureFilter(_filter.get_cutoff_freq());
}

void PX4Accelerometer::set_update_rate(uint16_t rate)
{
	const uint32_t update_interval = 1000000 / rate;

	_integrator_reset_samples = 4000 / update_interval;
}

void PX4Accelerometer::UpdateCalibration()
{
	for (unsigned i = 0; i < 3; ++i) {
		char str[30] {};
		sprintf(str, "CAL_ACC%u_ID", i);
		int32_t device_id = 0;

		if (param_get(param_find(str), &device_id) != OK) {
			PX4_ERR("Could not access param %s", str);
			continue;
		}

		if (device_id == 0) {
			continue;
		}

		if ((uint32_t)device_id == _device_id) {
			int32_t enabled = 1;
			sprintf(str, "CAL_ACC%u_EN", i);
			param_get(param_find(str), &enabled);
			_enabled = (enabled == 1);

			// scale factors (x, y, z)
			float scale[3] {};

			sprintf(str, "CAL_ACC%u_XSCALE", i);
			param_get(param_find(str), &scale[0]);

			sprintf(str, "CAL_ACC%u_YSCALE", i);
			param_get(param_find(str), &scale[1]);

			sprintf(str, "CAL_ACC%u_ZSCALE", i);
			param_get(param_find(str), &scale[2]);

			_calibration_scale = matrix::Vector3f{scale};


			// offsets factors (x, y, z)
			float offset[3] {};

			sprintf(str, "CAL_ACC%u_XOFF", i);
			param_get(param_find(str), &offset[0]);

			sprintf(str, "CAL_ACC%u_YOFF", i);
			param_get(param_find(str), &offset[1]);

			sprintf(str, "CAL_ACC%u_ZOFF", i);
			param_get(param_find(str), &offset[2]);

			_calibration_offset = matrix::Vector3f{offset};

			_calibrated = true;

			return;
		}
	}

	// reset if no calibration data found
	_calibration_scale = matrix::Vector3f{1.0f, 1.0f, 1.0f};
	_calibration_offset.zero();
	_calibrated = false;
	_enabled = true;
}

void PX4Accelerometer::update(hrt_abstime timestamp, float x, float y, float z)
{
	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		ModuleParams::updateParams();
		UpdateCalibration();

		ConfigureFilter(_param_imu_accel_cutoff.get());
	}

	// Apply rotation (before scaling)
	rotate_3f(_rotation, x, y, z);

	const Vector3f raw{x, y, z};

	// Clipping
	sensor_accel_status_s &status = _sensor_status_pub.get();
	const float clip_limit = (_range / _scale) * 0.95f;

	for (int i = 0; i < 3; i++) {
		if (fabsf(raw(i)) > clip_limit) {
			status.clipping[i]++;
			_integrator_clipping++;
		}
	}

	// Apply range scale and the calibrating offset/scale
	const Vector3f val_calibrated{(((raw * _scale) - _calibration_offset).emult(_calibration_scale))};

	// Filtered values
	const Vector3f val_filtered{_filter.apply(val_calibrated)};

	// Integrated values
	Vector3f integrated_value;
	uint32_t integral_dt = 0;

	_integrator_samples++;

	if (_integrator.put(timestamp, val_calibrated, integrated_value, integral_dt)) {

		sensor_accel_s report{};
		report.timestamp = timestamp;
		report.device_id = _device_id;
		report.temperature = _temperature;
		report.scaling = _scale;
		report.error_count = _error_count;

		// Raw values (ADC units 0 - 65535)
		report.x_raw = x;
		report.y_raw = y;
		report.z_raw = z;

		report.x = val_filtered(0);
		report.y = val_filtered(1);
		report.z = val_filtered(2);

		report.integral_dt = integral_dt;
		report.integral_samples = _integrator_samples;
		report.x_integral = integrated_value(0);
		report.y_integral = integrated_value(1);
		report.z_integral = integrated_value(2);
		report.integral_clip_count = _integrator_clipping;

		_sensor_pub.publish(report);

		// reset integrator
		ResetIntegrator();

		// update vibration metrics
		const Vector3f delta_velocity = integrated_value * (integral_dt * 1.e-6f);
		UpdateVibrationMetrics(delta_velocity);
	}

	// publish status
	status.device_id = _device_id;
	status.error_count = _error_count;
	status.full_scale_range = _range;
	status.rotation = _rotation;
	status.measure_rate = _update_rate;
	status.sample_rate = _sample_rate;
	status.temperature = _temperature;
	status.vibration_metric = _vibration_metric;
	status.calibrated = _calibrated;
	status.enabled = _enabled;
	status.timestamp = hrt_absolute_time();
	_sensor_status_pub.publish(status);
}

void PX4Accelerometer::updateFIFO(const FIFOSample &sample)
{
	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		ModuleParams::updateParams();
		UpdateCalibration();

		ConfigureFilter(_param_imu_accel_cutoff.get());
	}

	// filtered data (control)
	float x_filtered = _filterArrayX.apply(sample.x, sample.samples);
	float y_filtered = _filterArrayY.apply(sample.y, sample.samples);
	float z_filtered = _filterArrayZ.apply(sample.z, sample.samples);

	// Apply rotation (before scaling)
	rotate_3f(_rotation, x_filtered, y_filtered, z_filtered);

	const Vector3f raw{x_filtered, y_filtered, z_filtered};

	// Apply range scale and the calibrating offset/scale
	const Vector3f val_calibrated{(((raw * _scale) - _calibration_offset).emult(_calibration_scale))};


	// status
	{
		sensor_accel_status_s &status = _sensor_status_pub.get();

		const int16_t clip_limit = (_range / _scale) * 0.95f;

		// x clipping
		for (int n = 0; n < sample.samples; n++) {
			if (abs(sample.x[n]) > clip_limit) {
				status.clipping[0]++;
				_integrator_clipping++;
			}
		}

		// y clipping
		for (int n = 0; n < sample.samples; n++) {
			if (abs(sample.y[n]) > clip_limit) {
				status.clipping[1]++;
				_integrator_clipping++;
			}
		}

		// z clipping
		for (int n = 0; n < sample.samples; n++) {
			if (abs(sample.z[n]) > clip_limit) {
				status.clipping[2]++;
				_integrator_clipping++;
			}
		}

		status.device_id = _device_id;
		status.error_count = _error_count;
		status.full_scale_range = _range;
		status.rotation = _rotation;
		status.measure_rate = _update_rate;
		status.sample_rate = _sample_rate;
		status.temperature = _temperature;
		status.calibrated = _calibrated;
		status.enabled = _enabled;
		status.timestamp = hrt_absolute_time();
		_sensor_status_pub.publish(status);
	}


	// integrated data (INS)
	{
		// reset integrator if previous sample was too long ago
		if ((sample.timestamp_sample > _timestamp_sample_prev)
		    && ((sample.timestamp_sample - _timestamp_sample_prev) > (sample.samples * sample.dt * 2))) {

			ResetIntegrator();
		}

		if (_integrator_samples == 0) {
			_integrator_timestamp_sample = sample.timestamp_sample;
		}

		// integrate
		_integrator_samples += 1;
		_integrator_fifo_samples += sample.samples;

		for (int n = 0; n < sample.samples; n++) {
			_integrator_accum[0] += sample.x[n];
		}

		for (int n = 0; n < sample.samples; n++) {
			_integrator_accum[1] += sample.y[n];
		}

		for (int n = 0; n < sample.samples; n++) {
			_integrator_accum[2] += sample.z[n];
		}

		if (_integrator_fifo_samples > 0 && (_integrator_samples >= _integrator_reset_samples)) {

			const uint32_t integrator_dt_us = _integrator_fifo_samples * sample.dt; // time span in microseconds

			// average integrated values to apply calibration
			float x_int_avg = _integrator_accum[0] / _integrator_fifo_samples;
			float y_int_avg = _integrator_accum[1] / _integrator_fifo_samples;
			float z_int_avg = _integrator_accum[2] / _integrator_fifo_samples;

			// Apply rotation (before scaling)
			rotate_3f(_rotation, x_int_avg, y_int_avg, z_int_avg);

			const Vector3f raw_int{x_int_avg, y_int_avg, z_int_avg};

			// Apply range scale and the calibrating offset/scale
			Vector3f val_int_calibrated{(((raw_int * _scale) - _calibration_offset).emult(_calibration_scale))};
			val_int_calibrated *= (_integrator_fifo_samples * sample.dt * 1e-6f);	// restore

			// publish
			sensor_accel_s report{};
			report.device_id = _device_id;
			report.temperature = _temperature;
			report.scaling = _scale;
			report.error_count = _error_count;

			// Raw values (ADC units 0 - 65535)
			report.x_raw = sample.x[0];
			report.y_raw = sample.y[0];
			report.z_raw = sample.z[0];

			report.x = val_calibrated(0);
			report.y = val_calibrated(1);
			report.z = val_calibrated(2);

			report.integral_dt = integrator_dt_us;
			report.integral_samples = _integrator_fifo_samples;
			report.x_integral = val_int_calibrated(0);
			report.y_integral = val_int_calibrated(1);
			report.z_integral = val_int_calibrated(2);
			report.integral_clip_count = _integrator_clipping;

			report.timestamp = _integrator_timestamp_sample;
			_sensor_pub.publish(report);

			// update vibration metrics
			const Vector3f delta_velocity = val_int_calibrated * (integrator_dt_us * 1.e-6f);
			UpdateVibrationMetrics(delta_velocity);

			// reset integrator
			ResetIntegrator();
		}

		_timestamp_sample_prev = sample.timestamp_sample;
	}

	sensor_accel_fifo_s fifo{};

	fifo.device_id = _device_id;
	fifo.timestamp_sample = sample.timestamp_sample;
	fifo.dt = sample.dt;
	fifo.scale = _scale;
	fifo.samples = sample.samples;

	memcpy(fifo.x, sample.x, sizeof(sample.x[0]) * sample.samples);
	memcpy(fifo.y, sample.y, sizeof(sample.y[0]) * sample.samples);
	memcpy(fifo.z, sample.z, sizeof(sample.z[0]) * sample.samples);

	fifo.timestamp = hrt_absolute_time();
	_sensor_fifo_pub.publish(fifo);
}

void PX4Accelerometer::ResetIntegrator()
{
	_integrator_samples = 0;
	_integrator_fifo_samples = 0;
	_integrator_accum[0] = 0;
	_integrator_accum[1] = 0;
	_integrator_accum[2] = 0;
	_integrator_clipping = 0;

	_integrator_timestamp_sample = 0;
	_timestamp_sample_prev = 0;
}

void PX4Accelerometer::ConfigureFilter(float cutoff_freq)
{
	_filter.set_cutoff_frequency(_sample_rate, cutoff_freq);

	_filterArrayX.set_cutoff_frequency(_sample_rate, cutoff_freq);
	_filterArrayY.set_cutoff_frequency(_sample_rate, cutoff_freq);
	_filterArrayZ.set_cutoff_frequency(_sample_rate, cutoff_freq);
}

void PX4Accelerometer::UpdateVibrationMetrics(const Vector3f &delta_velocity)
{
	// Accel high frequency vibe = filtered length of (delta_velocity - prev_delta_velocity)
	const Vector3f delta_velocity_diff = delta_velocity - _delta_velocity_prev;
	_vibration_metric = 0.99f * _vibration_metric + 0.01f * delta_velocity_diff.norm();

	_delta_velocity_prev = delta_velocity;
}

void PX4Accelerometer::print_status()
{
	char device_id_buffer[80] {};
	device::Device::device_id_print_buffer(device_id_buffer, sizeof(device_id_buffer), _device_id);
	PX4_INFO("device id: %d (%s)", _device_id, device_id_buffer);
	PX4_INFO("rotation: %d", _rotation);
	PX4_INFO("sample rate: %d Hz", _sample_rate);
	PX4_INFO("filter cutoff: %.3f Hz", (double)_filter.get_cutoff_freq());
	PX4_INFO("calibrated: %d", _calibrated);
	PX4_INFO("enabled: %d", _enabled);

	PX4_INFO("calibration scale: %.5f %.5f %.5f", (double)_calibration_scale(0), (double)_calibration_scale(1),
		 (double)_calibration_scale(2));
	PX4_INFO("calibration offset: %.5f %.5f %.5f", (double)_calibration_offset(0), (double)_calibration_offset(1),
		 (double)_calibration_offset(2));

}
