import yaw_controller
import pid
import lowpass
import rospy
GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MIN_SPEED = 0.1
LOWPASS_TAU = 0.5
LOWPASS_TS = 0.02 #sample time

class Controller(object):
    def __init__(self, vehicle_mass,fuel_capacity, brake_deadband, decel_limit,
                 accel_limit, wheel_radius,
                 wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        self.yaw_controller =YawController(wheel_base, steer_ratio, MIN_SPEED , max_lat_accel, max_steer_angle)

        #pid params
        kp = 0.3
        ki = 0.1
        kd = 0.
        min_throttle = 0.
        max_throttle = 0.2
        self.throttle_ctr = PID(kp, ki, kd, min_throttle, max_throttle)
        self.vel_lowpass = LowPassFilter(LOWPASS_TAU, LOWPASS_TS )

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.last_time = rospy.get_time()


    def control(self, dbw_enabled, current_vel, linear_vel, angular_vel):
        if not dbw_enabled:
            self.throttle_ctr.reset()
            return 0.,0., 0

        current_vel = self.vel_lowpass.filt(current_vel)
        #todo might need to tweak this to dampen large angular vel changes
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        vel_diff = linear_vel - current_vel
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        throttle = 0

        self.throttle_ctr.step(vel_diff, sample_time)
        brake = 0

        #stopped e.g at traffoc lights
        if linear_vel <= 0.1 and current_vel <= 0.1:
            throttle = 0.
            brake = 400

        elif throttle < 0.1 and vel_diff < 0:
            throttle = 0
            deceleration = max(vel_diff, self.decel_limit)
            brake = abs(deceleration) * self.vehicle_mass * self.wheel_radius
        return throttle, brake, steering
