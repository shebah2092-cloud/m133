#pragma once

#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_local_position.h>
#include <drivers/drv_hrt.h>

#include <cmath>

static constexpr int SENSOR_NMEAS = 13;

// GPS staleness timeout: if no GPS update within this window, treat
// the cached position/velocity as invalid so stale values are not
// pushed into the MHE measurement stream.
static constexpr hrt_abstime GPS_STALE_TIMEOUT_US = 500'000; // 500 ms

struct SensorMeasurement {
	double y[SENSOR_NMEAS];
	// y[0..2]  = gyro_rad[0..2]          (raw IMU)
	// y[3..5]  = accelerometer_m_s2[0..2] (raw IMU)
	// y[6]     = baro_alt_meter           (raw baro)
	// y[7..8]  = GPS position north, east (m, relative to arm origin)
	// y[9]     = GPS altitude MSL (m)
	// y[10..12] = GPS velocity N, E, D (m/s)
	bool valid;
};

class SensorBridge {
public:
	SensorBridge() = default;

	void set_launch_alt(float launch_alt_m) {
		_launch_alt_m = launch_alt_m;
	}

	/** Set GPS reference origin in WGS84 (call at arm / pre-launch) */
	void set_gps_origin(double lat_deg, double lon_deg, double alt_msl_m) {
		_ref_lat_deg = lat_deg;
		_ref_lon_deg = lon_deg;
		_ref_alt_msl = alt_msl_m;
		_ref_cos_lat = cos(lat_deg * M_PI / 180.0);
		_gps_origin_set = true;
	}

	/** Feed raw GPS reading — converts to local NED internally */
	void update_gps(const sensor_gps_s &gps);

	/** SITL fallback: use EKF2 local position as GPS substitute.
	 *  Only use when real sensor_gps is unavailable (SITL circular dep).
	 *  Requires set_gps_origin() to have been called first (for alt ref). */
	void update_from_lpos(const vehicle_local_position_s &lpos);

	/** Build MHE measurement vector from raw sensors (bypasses EKF2) */
	SensorMeasurement build_measurement(
		const sensor_combined_s &sc,
		const vehicle_air_data_s &air);

	/** Reference altitude MSL (set at arm). Used as MHE launch_alt parameter. */
	double ref_alt_msl() const { return _ref_alt_msl; }
	bool   gps_origin_set() const { return _gps_origin_set; }
	bool   gps_valid() const { return _gps_valid; }

	/** True when last GPS update is newer than GPS_STALE_TIMEOUT_US. */
	bool   gps_fresh(hrt_abstime now) const {
		return _gps_valid && _last_gps_update_us > 0
		       && (now - _last_gps_update_us) < GPS_STALE_TIMEOUT_US;
	}

	hrt_abstime last_gps_update_us() const { return _last_gps_update_us; }

private:
	// WGS84 flat-earth constants
	static constexpr double METERS_PER_DEG_LAT    = 111132.92;
	static constexpr double METERS_PER_DEG_LON_EQ = 111320.0;

	// GPS reference origin
	double _ref_lat_deg{0.0};
	double _ref_lon_deg{0.0};
	double _ref_alt_msl{0.0};
	double _ref_cos_lat{1.0};
	bool   _gps_origin_set{false};

	// Last GPS reading converted to local NED
	double _gps_north{0.0};
	double _gps_east{0.0};
	double _gps_alt_msl{0.0};
	double _gps_vn{0.0};
	double _gps_ve{0.0};
	double _gps_vd{0.0};
	bool   _gps_valid{false};
	hrt_abstime _last_gps_update_us{0};

	float _launch_alt_m{0.0f};
};
