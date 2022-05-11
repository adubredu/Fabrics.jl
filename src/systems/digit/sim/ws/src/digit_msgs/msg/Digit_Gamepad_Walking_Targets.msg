Header header
string ctrl_mode    # controller mode (standing, walking)
string fp_type      # foot placment method (1step or mpc)
float32 vel_x_des   # target sagittal velocity (body frame)
float32 vel_y_des   # target lateral velocity (body frame)
float32 turn_rps    # turn radians per sec. Multiplied by step time to decide target heading        