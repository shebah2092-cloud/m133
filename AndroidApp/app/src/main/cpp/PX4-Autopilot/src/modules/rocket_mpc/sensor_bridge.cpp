#include "sensor_bridge.h"
#include <px4_platform_common/defines.h>   // PX4_ISFINITE

void SensorBridge::update_gps(const sensor_gps_s &gps)
{
	if (gps.fix_type < sensor_gps_s::FIX_TYPE_3D) { return; }

	// Reject non-finite values (NaN / Inf) before they poison the flat-earth
	// conversion, the origin latch, or the MHE measurement vector. A NaN on
	// the first fix would permanently corrupt _ref_lat_deg via set_gps_origin.
	if (!PX4_ISFINITE(gps.latitude_deg)
	    || !PX4_ISFINITE(gps.longitude_deg)
	    || !PX4_ISFINITE(gps.altitude_msl_m)) {
		return;
	}

	// Auto-set origin on first valid 3D fix if not yet set
	if (!_gps_origin_set) {
		set_gps_origin(gps.latitude_deg, gps.longitude_deg, gps.altitude_msl_m);
	}

	// Flat-earth conversion: GPS WGS84 → local NED relative to arm origin
	const double dlat = gps.latitude_deg  - _ref_lat_deg;
	const double dlon = gps.longitude_deg - _ref_lon_deg;

	_gps_north   = dlat * METERS_PER_DEG_LAT;
	_gps_east    = dlon * METERS_PER_DEG_LON_EQ * _ref_cos_lat;
	_gps_alt_msl = gps.altitude_msl_m;

	if (gps.vel_ned_valid
	    && PX4_ISFINITE(gps.vel_n_m_s)
	    && PX4_ISFINITE(gps.vel_e_m_s)
	    && PX4_ISFINITE(gps.vel_d_m_s)) {
		_gps_vn = (double)gps.vel_n_m_s;
		_gps_ve = (double)gps.vel_e_m_s;
		_gps_vd = (double)gps.vel_d_m_s;
	}

	_gps_valid = true;
	_last_gps_update_us = hrt_absolute_time();
}

void SensorBridge::update_from_lpos(const vehicle_local_position_s &lpos)
{
	// SITL fallback: treat EKF2 local position as GPS measurements.
	// Position is NED relative to EKF2 origin (≈ arm origin).

	// Guard 1: reject until GPS origin is set — otherwise _ref_alt_msl=0
	// and _gps_alt_msl would be a relative value mistaken for MSL,
	// causing MHE to diverge trying to fit physics at wrong altitude.
	if (!_gps_origin_set) { return; }

	// Guard 2: reject if EKF2 has not yet produced valid position estimates.
	if (!lpos.xy_valid || !lpos.z_valid) { return; }

	// Guard 3: reject non-finite values (NaN / Inf) that would poison MHE.
	if (!PX4_ISFINITE(lpos.x) || !PX4_ISFINITE(lpos.y) || !PX4_ISFINITE(lpos.z)) { return; }

	_gps_north   = (double)lpos.x;
	_gps_east    = (double)lpos.y;
	// Convert NED-down to altitude MSL for MHE measurement model
	_gps_alt_msl = _ref_alt_msl + (double)(-lpos.z);

	if (lpos.v_xy_valid && lpos.v_z_valid
	    && PX4_ISFINITE(lpos.vx) && PX4_ISFINITE(lpos.vy) && PX4_ISFINITE(lpos.vz)) {
		_gps_vn = (double)lpos.vx;
		_gps_ve = (double)lpos.vy;
		_gps_vd = (double)lpos.vz;
	}

	_gps_valid = true;
	_last_gps_update_us = hrt_absolute_time();
}

SensorMeasurement SensorBridge::build_measurement(
	const sensor_combined_s &sc,
	const vehicle_air_data_s &air)
{
	SensorMeasurement m;

	// Drop GPS validity if no update arrived within the staleness window.
	// Prevents pushing frozen position/velocity into the MHE when the GPS
	// feed dies mid-flight.
	const hrt_abstime now = hrt_absolute_time();
	if (_gps_valid && _last_gps_update_us > 0
	    && (now - _last_gps_update_us) > GPS_STALE_TIMEOUT_US) {
		_gps_valid = false;
	}

	m.valid = (sc.timestamp > 0) && _gps_valid;

	// Gyroscope (body frame, rad/s) — raw from IMU
	m.y[0] = (double)sc.gyro_rad[0];
	m.y[1] = (double)sc.gyro_rad[1];
	m.y[2] = (double)sc.gyro_rad[2];

	// Accelerometer (body frame, m/s²) — raw from IMU
	m.y[3] = (double)sc.accelerometer_m_s2[0];
	m.y[4] = (double)sc.accelerometer_m_s2[1];
	m.y[5] = (double)sc.accelerometer_m_s2[2];

	// Barometric altitude (m) — raw from baro
	m.y[6] = (double)air.baro_alt_meter;

	// GPS position relative to arm origin (local NED, meters)
	m.y[7]  = _gps_north;
	m.y[8]  = _gps_east;
	m.y[9]  = _gps_alt_msl;

	// GPS velocity (NED frame, m/s) — raw from GPS
	m.y[10] = _gps_vn;
	m.y[11] = _gps_ve;
	m.y[12] = _gps_vd;

	return m;
}
