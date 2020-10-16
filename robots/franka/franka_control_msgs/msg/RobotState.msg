std_msgs/Header header

float64[6] cartesian_collision
float64[6] cartesian_contact
float64[6] O_dP_EE # EE vel computed as J*dq

# float64[7] q # joint position, velocity, and effort in joint_states topic
# float64[7] dq

float64[7] q_d
float64[7] dq_d
# float64[7] tau_J
float64[7] dtau_J # torque derivative
float64[7] tau_J_d # desired joint torque
# float64[6] K_F_ext_hat_K
# float64[2] elbow
# float64[2] elbow_d

float64[7] joint_collision
float64[7] joint_contact

# float64[6] O_F_ext_hat_K # in tip state

float64[7] tau_ext_hat_filtered # filtered external torque
float64[3] F_x_Cee # Configured center of mass of the end effector load with respect to flange frame.
float64[3] F_x_Cload # Configured center of mass of the external load with respect to flange frame.
float64[3] F_x_Ctotal # Combined center of mass of the end effector load and the external load with respect to flange frame.
float64[9] I_ee # Configured rotational inertia matrix of the end effector load with respect to center of mass.
float64[9] I_load
float64[9] I_total

float64 m_ee # configured mass of end-effector
float64 m_load # configured mass of external load
float64 m_total


float64[7] gravity
float64[7] coriolis
float64[49] mass_matrix # mass matrix of end-effector wrt base frame # Vectorized 7x7, column-major

float64[42] O_Jac_EE # zero jacobian of end-effector frame. Vectorized 6x7 Jacobian, column-major

# float64[16] O_T_EE # ----- in tip state # Vectorized 4x4, column-major
float64[16] O_T_EE_d # Last desired end effector pose of motion generation in base frame.  # Vectorized 4x4, column-major
float64[16] F_T_EE # End effector frame pose in flange frame # Vectorized 4x4, column-major
float64[16] EE_T_K # Stiffness frame pose in end effector frame # Vectorized 4x4, column-major
float64 time

uint8 ROBOT_MODE_OTHER=0
uint8 ROBOT_MODE_IDLE=1
uint8 ROBOT_MODE_MOVE=2
uint8 ROBOT_MODE_GUIDING=3
uint8 ROBOT_MODE_REFLEX=4
uint8 ROBOT_MODE_USER_STOPPED=5
uint8 ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY=6
uint8 robot_mode

franka_msgs/Errors current_errors
franka_msgs/Errors last_motion_errors
