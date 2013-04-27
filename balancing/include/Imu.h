/**
 * @file Imu.h
 * @author Can Erdogan
 * @date Jan 14, 2013
 * @brief The Imu definition.
 */

/// IMU interface
class Imu {
private:
	ach_channel_t imu_chan;
	double th;								  // pitch [rad]
	double dth;								 	// pitch (angular) velocity [rad/s]

public:
	double imu_offset;					// the offset to all readings (?)
	Imu(const char *channelName, double imu_offset);					///< The constructor
	~Imu();																///< The destructor
	void update();												///< Updates the imu readings
	inline double get_th()  {   return th;  }
	inline double get_dth() {   return dth; }
};
