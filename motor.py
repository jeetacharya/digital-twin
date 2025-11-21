#!/usr/bin/env python
# coding: utf-8

# In[1]:


'''
Digital Twin for a Medical Device Actuator (Software Simulation)
'''


# Importing libraries
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp


# Defining function to solve the differential equation using scipy.integrate.solve_ivp function
def angle_solver(t, y, setpoint, b, j, kt, kp, ki, kd, prev_t, prev_error, error_int, input_type):
    theta, dtheta = y                                    # extract state parameters
    if input_type == 'step':
        error = setpoint - theta
    elif input_type == 'ramp':
        error = (setpoint * t / 2) - theta
    else:
        error = np.sin(setpoint * t) - np.sin(theta)    # sinusoidal input error calculation
    error_int += error * (t - prev_t)
    if t != prev_t:
        error_differential = (error - prev_error)/(t - prev_t)
    else:
        error_differential = 0
    torque = (kp * error) + (ki * error_int) + (kd * error_differential)    # torque calculation using PID gains
    ddtheta = ((kt * torque) - (b * dtheta))/(j)
    dy_dt = [dtheta, ddtheta]                         # derivative of state parameters
    prev_error = error
    prev_t = t
    return dy_dt


# Initializing values
t_span = (0, 4)
time_array = np.arange(0,4,0.001)
y0 = [0.0, 0.0]                      # initial state is theta = 0 rad and dtheta = 0 rad/s
setpoint = np.pi/2


# Parameter values for true (real) system
b_true = 0.025         # damping [Nms/rad]
j_true = 0.0035        # inertia [kg-m^2]
kt_true = 0.05         # torque constant [Nm/A]


# Parameter values for twin system
b_twin = 0.022         # slightly off values
j_twin = 0.004
kt_twin = 0.055



# STEP input
# Tuning PID controller gains for true system
input_type = 'step'
kp_true_step = 0.65    # proportional gain for true system
ki_true_step = 0.5     # integral gain for true system
kd_true_step = 0.001   # derivative gain for true system


# Calling solve_ivp function with required tolerances and true system parameters
prev_error = 0.0       # reinitializing variables
error_int = 0.0
prev_t = t_span[0]

solver_true_step = solve_ivp(angle_solver, t_span, y0, rtol = 1e-2, atol = 1e-4, dense_output = True, 
                        args = (setpoint, b_true, j_true, kt_true, kp_true_step, ki_true_step, kd_true_step, prev_t, prev_error, error_int, input_type))


# Calling solve_ivp function with required tolerances and twin system parameters with PID gains of true system
prev_error = 0.0
error_int = 0.0
prev_t = t_span[0]

solver_twin_constant_step = solve_ivp(angle_solver, t_span, y0, rtol = 1e-2, atol = 1e-4, dense_output = True, 
                                 args = (setpoint, b_twin, j_twin, kt_twin, kp_true_step, ki_true_step, kd_true_step, prev_t, prev_error, error_int, input_type))


# Tuning PID controller gains for twin system
kp_twin_step = 0.5
ki_twin_step = 0.4
kd_twin_step = 0.0008


# Calling solve_ivp function with required tolerances and twin system parameters
prev_error = 0.0
error_int = 0.0
prev_t = t_span[0]

solver_twin_step = solve_ivp(angle_solver, t_span, y0, rtol = 1e-2, atol = 1e-4, dense_output = True, 
                        args = (setpoint, b_twin, j_twin, kt_twin, kp_twin_step, ki_twin_step, kd_twin_step, prev_t, prev_error, error_int, input_type))



# RAMP input
# Tuning PID controller gains for true system
input_type = 'ramp'
kp_true_ramp = 5
ki_true_ramp = 12
kd_true_ramp = 0.0015


# Calling solve_ivp function with required tolerances and true system parameters
prev_error = 0.0
error_int = 0.0
prev_t = t_span[0]

solver_true_ramp = solve_ivp(angle_solver, t_span, y0, rtol = 1e-2, atol = 1e-4, dense_output = True, 
                             args = (setpoint, b_true, j_true, kt_true, kp_true_ramp, ki_true_ramp, kd_true_ramp, prev_t, prev_error, error_int, input_type))


# Calling solve_ivp function with required tolerances and twin system parameters with PID gains of true system
prev_error = 0.0
error_int = 0.0
prev_t = t_span[0]

solver_twin_constant_ramp = solve_ivp(angle_solver, t_span, y0, rtol = 1e-2, atol = 1e-4, dense_output = True, 
                                 args = (setpoint, b_twin, j_twin, kt_twin, kp_true_ramp, ki_true_ramp, kd_true_ramp, prev_t, prev_error, error_int, input_type))


# Tuning PID controller gains for twin system
kp_twin_ramp = 2.5
ki_twin_ramp = 8
kd_twin_ramp = 0.006


# Calling solve_ivp function with required tolerances and twin system parameters
prev_error = 0.0
error_int = 0.0
prev_t = t_span[0]

solver_twin_ramp = solve_ivp(angle_solver, t_span, y0, rtol = 1e-2, atol = 1e-4, dense_output = True, 
                        args = (setpoint, b_twin, j_twin, kt_twin, kp_twin_ramp, ki_twin_ramp, kd_twin_ramp, prev_t, prev_error, error_int, input_type))



# SINUSOIDAL input
# Tuning PID controller gains for true system
input_type = 'sine'
kp_true_sine = 3
ki_true_sine = 11
kd_true_sine = 0.01


# Calling solve_ivp function with required tolerances and true system parameters
prev_error = 0.0
error_int = 0.0
prev_t = t_span[0]

solver_true_sine = solve_ivp(angle_solver, t_span, y0, rtol = 1e-3, atol = 1e-6, dense_output = True, 
                             args = (setpoint, b_true, j_true, kt_true, kp_true_sine, ki_true_sine, kd_true_sine, prev_t, prev_error, error_int, input_type))


# Calling solve_ivp function with required tolerances and twin system parameters with PID gains of true system
prev_error = 0.0
error_int = 0.0
prev_t = t_span[0]

solver_twin_constant_sine = solve_ivp(angle_solver, t_span, y0, rtol = 1e-2, atol = 1e-4, dense_output = True, 
                                 args = (setpoint, b_twin, j_twin, kt_twin, kp_true_sine, ki_true_sine, kd_true_sine, prev_t, prev_error, error_int, input_type))


# Tuning PID controller gains for twin system
kp_twin_sine = 2.7
ki_twin_sine = 10
kd_twin_sine = 0.008


# Calling solve_ivp function with required tolerances and twin system parameters
prev_error = 0.0
error_int = 0.0
prev_t = t_span[0]

solver_twin_sine = solve_ivp(angle_solver, t_span, y0, rtol = 1e-2, atol = 1e-4, dense_output = True, 
                        args = (setpoint, b_twin, j_twin, kt_twin, kp_twin_sine, ki_twin_sine, kd_twin_sine, prev_t, prev_error, error_int, input_type))



# Calculating overshoot, steady state error and settling time for STEP input
overshoot_twin_constant_step = round(((max(solver_twin_constant_step.y[0])) - (setpoint)) * 100 / (setpoint), 1)    # percentage overshoot

error_twin_constant_step = round(solver_twin_constant_step.y[0][-1] - setpoint, 4)    # steady state error

tolerance_percentage = 0.02                           # tolerance can be changed from current value of 2%
tolerance_band = tolerance_percentage * setpoint
settling_time_index = -1
for i in range(len(solver_twin_constant_step.y[0]) - 1, -1, -1):
    if abs(solver_twin_constant_step.y[0][i] - setpoint) > tolerance_band:
        settling_time_index = i + 1
        break
if settling_time_index != -1 and settling_time_index < len(solver_twin_constant_step.t):
    settling_twin_constant_step = solver_twin_constant_step.t[settling_time_index]    # settling time calculation
else:
    settling_twin_constant_step = 0                                                   # settling time does not exist if data does not stay within tolerance range
settling_twin_constant_step = round(settling_twin_constant_step, 1)


# Displaying outputs
print("Step input:-")
print("Response characteristics of twin system with PID gains of true system:- ")
print("Overshoot:                                     ", overshoot_twin_constant_step, "%")
print("Settling time:                                 ", settling_twin_constant_step, "s")
print("Steady state error (deg):                      ", error_twin_constant_step * 180/np.pi)
print("")
print("Steady state theta (deg) for true system:                               ", round(solver_true_step.y[0][-1] * 180/np.pi, 1))
print("Steady state theta (deg) for twin system with PID gains of true system: ", round(solver_twin_constant_step.y[0][-1] * 180/np.pi, 1))
print("Steady state theta (deg) for twin system:                               ", round(solver_twin_step.y[0][-1] * 180/np.pi, 1))
print("Setpoint theta (deg) for step input:                                    ", round(np.pi/2 * 180/np.pi, 1))
print("")
print("")

print("Ramp input:-")
print("Final theta (deg) for true system:                               ", round(solver_true_ramp.y[0][-1] * 180/np.pi, 1))
print("Final theta (deg) for twin system with PID gains of true system: ", round(solver_twin_constant_ramp.y[0][-1] * 180/np.pi, 1))
print("Final theta (deg) for twin system:                               ", round(solver_twin_ramp.y[0][-1] * 180/np.pi, 1))
print("Final theta (deg) for ramp input:                                ", round(np.pi * 180/np.pi, 1))
print("")
print("")

print("Sinusoidal input:-")
print("Final theta (deg) for true system:                               ", round(solver_true_sine.y[0][-1] * 180/np.pi, 1))
print("Final theta (deg) for twin system with PID gains of true system: ", round(solver_twin_constant_sine.y[0][-1] * 180/np.pi, 1))
print("Final theta (deg) for twin system:                               ", round(solver_twin_sine.y[0][-1] * 180/np.pi, 1))
print("Final theta (deg) for sinusoidal input:                          ", round(np.sin(2 * np.pi) * 180/np.pi, 1))
print("")
print("")


# Plotting and saving graphs
fig1, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize = (8, 18))
ax1.plot(solver_true_step.t, solver_true_step.y[0], label = 'True theta', color = 'blue')
ax1.plot(solver_twin_constant_step.t, solver_twin_constant_step.y[0], label = 'Twin constant theta', color = 'green')
ax1.plot(solver_twin_step.t, solver_twin_step.y[0], label = 'Twin theta', color = 'red')
ax1.plot(time_array, np.array(setpoint * np.ones_like(time_array)), label = 'Step input', color = 'grey', linestyle = '--')
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Theta (rad)")
ax1.set_title("Joint angle vs Time for Step Input")
ax1.legend()
ax2.plot(solver_true_ramp.t, solver_true_ramp.y[0], label = 'True theta', color = 'blue')
ax2.plot(solver_twin_constant_ramp.t, solver_twin_constant_ramp.y[0], label = 'Twin constant theta', color = 'green')
ax2.plot(solver_twin_ramp.t, solver_twin_ramp.y[0], label = 'Twin theta', color = 'red')
ax2.plot(time_array, np.array((setpoint / 2) * time_array), label = 'Ramp input', color = 'grey', linestyle = '--')
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Theta (rad)")
ax2.set_title("Joint angle vs Time for Ramp Input")
ax2.legend()
ax3.plot(solver_true_sine.t, solver_true_sine.y[0], label = 'True theta', color = 'blue')
ax3.plot(solver_twin_constant_sine.t, solver_twin_constant_sine.y[0], label = 'Twin constant theta', color = 'green')
ax3.plot(solver_twin_sine.t, solver_twin_sine.y[0], label = 'Twin theta', color = 'red')
ax3.plot(time_array, np.array(np.sin(setpoint * time_array)), label = 'Sinusoidal input', color = 'grey', linestyle = '--')
ax3.set_xlabel("Time (s)")
ax3.set_ylabel("Theta (rad)")
ax3.set_title("Joint angle vs Time for Sine Input")
ax3.legend()
fig1.savefig('plots.png', dpi = 300, bbox_inches = 'tight')
plt.show()


# In[281]:


get_ipython().system('jupyter nbconvert --to script motor.ipynb')


# In[ ]:




