/**
 * @file Motor.h
 * @author Can Erdogan
 * @date Jan 14, 2013
 * @brief The motor interface definitions.
 */

#include <somatic/motor.h>
#include <unistd.h>

/// The interface to the motors
struct Motor {
private:
	unsigned int n_data;						///< The number of motors on this set, i.e. arm = 7.

public:
	somatic_motor_t motor;
	std::string label;
	Motor(const char *, const char *, const std::string&, const unsigned int&);
	~Motor();
	void halt();
	void reset();
	void update();
	void set_current(double *);
	void set_velocity(double *);
	void set_position(double *);
	inline unsigned int get_ndata() { return n_data; }
	inline double get_pos(const short& num) {
		if (!n_data) exit(-1);
		return this->motor.pos[num];
	}
	inline double get_vel(const short& num) {
		if (!n_data) exit(-1);
		return this->motor.vel[num];
	}
	inline void set_pos_offset(const double* offset) {
		if (!n_data) { fprintf(stderr, "%s: n_data = %d\n", label.c_str(), n_data); exit(-1); }
		aa_fcpy( this->motor.pos_offset, offset, this->motor.n );
	}

	void generate_motorcmd( double *vals, Somatic__MotorParam type );
	
	void digital_out(size_t, bool);
};


