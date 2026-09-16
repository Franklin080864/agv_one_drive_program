# Direct dual-steer per-wheel control. Bypasses kinematics in agv2_control_node.
# Body frame (base_link): x forward, y left, z up; theta CCW about z.

float64 front_steer_angle   # rad, front steering wheel target angle
float64 rear_steer_angle    # rad, rear steering wheel target angle
float64 front_wheel_speed   # m/s, front wheel linear speed (+ = body forward)
float64 rear_wheel_speed    # m/s, rear wheel linear speed (+ = body forward)
