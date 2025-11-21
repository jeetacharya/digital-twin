# digital-twin
Digital Twin for a Medical Device Actuator (Software Simulation)

## Goal:
1. The main goal of this project is to model a rotational actuator system so that it follows a desired joint angle trajectory (step, ramp and sinusoidal) using a PID controller.
2. Another goal of this project is to develop a “digital twin” model with slightly offset system parameter values that can replicate the motion of the real system with different values of the PID gains.
3. The goal is also to check the behaviour of a model with system parameters of the twin model and PID gain values of the real model when provided with step, ramp and sinusoidal desired joint angle trajectories.

## Approach:
1. A rotational actuator system is modelled as a 2nd order differential equation in the joint angle theta. The coefficients of the equation (constants) are defined as system parameters and have different values for the true (real) model and the twin model.
2. Since a 2nd order differential equation is involved, the state can be defined as [theta, dtheta] where theta is the joint angle and dtheta is the 1st order derivative of the joint angle theta.
3. The values of the joint angles theta can be found at different times using the function scipy.integrate.solve_ivp given the initial state values.
4. This solve_ivp function was used because it can numerically integrate a system of ordinary differential equations given the initial state. The initial state for all 3 models was defined as [0, 0].
5. The solve_ivp function calls the angle_solver function which is used to differentiate the state parameters wrt time (in this case) using the PID controller gains for calculating the control input.
6. The true model and the twin model with PID controller gains of the true model were run with the same PID gain values whereas there was a different set of PID gain values for the twin model.
7. The PID gains were tuned so that the desired joint angle trajectory can be reached with minimum possible overshoot, settling time and steady state error while adjusting the default tolerances within the solve_ivp function.
8. The response characteristics of the twin model with PID controller gains of the true model were calculated so as to compare them with the other models for step input.

## Results:
1. The parameters for the twin model are slightly off compared to the true (real) model. Hence, in order to achieve similar joint angle trajectories, different PID gain values had to be used for the true model and the twin model.
2. The twin constant model is another model which has the parameters of the twin model but the same controller gains as used in the true model. It is used to determine how closely the twin model behaves when being used with the PID gain values of the true model.
3. For step input, the steady state joint angle did not have any significant error after applying the respective controller gains on the models.
4. All the models had similar settling time with the overshoot being highest for the twin model with controller gains of the true model.
5. The overshoot for the twin model with controller gains of the true model was 2.6% and the settling time was 1.6 seconds.
6. For ramp input, there was up to 0.4 degrees of error in the final joint angle in the 3 models wrt the final joint angle of the ramp input.
7. For sinusoidal input, the range of error in the final joint angle for the 3 models wrt the sinusoidal input was -0.4 to +0.5 degrees with the twin model having no error.
8. The graphs of joint angle vs time for step input, ramp input and sinusoidal input are shown below:
