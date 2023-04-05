# Model Predictive Control on the pendulum

Model Predictive Control implemented in Python, using scipy.optimize.minimize, on the model of a pendulum.

Code created to support a Linkedin post. Link to the post: https://www.linkedin.com/posts/simone-bertoni-control-eng_model-predictive-control-activity-7047877760085565440-ajf7?utm_source=share&utm_medium=member_desktop

Follow me on Linkedin! https://www.linkedin.com/in/simone-bertoni-control-eng/

![image](https://user-images.githubusercontent.com/29520048/229251774-ea09b9de-792f-4613-a3cf-efad98db4361.png)

So much fun playing with Model Predictive Control!

And Python. Interested? 👇

My problem has always been that there are many libraries that show how MPC works and assist you in the test.

However, there's little freedom to implement the algorithm in a way that can be easily ported on a microcontroller.

MPC is based on the minimisation of a cost function, so I always wanted to implement it using a minimisation function, not a specific MPC library.

Finally, I had some time to play and got a working example.

Let's see! 👇

The picture below shows a recap of the pendulum dynamics and its differential equation:

The plant used is a pendulum, with the following dynamic equation:

ddtheta_ddt = (tau - m\*g\*l\*sin(theta) - k\*dtheta_dt) / (m\*l^2)

To formulate the MPC algorithm, we need the system in the discrete-time form:

✅ ddtheta_ddt = (tau - m\*g\*l\*sin(theta) - k\*dtheta_dt)/(m\*l^2)
✅ dtheta_dt = dtheta_dt + ddtheta_ddt\*dt
✅ theta = theta + dtheta_dt\*dt

where dt is our time step.

Now the cost function.

I included a term that considers the angular speed too, so:

J = sum[ Q11\*dtheta_dt(i)^2 + Q22\*(theta_ref(i) - theta(i))^2 + R\*tau(i)^2]

Also, I thought of two logical constraints for the torque control sequence:

✅ Max torque: -tau_max < tau(i) < tau_max
✅ Torque rate of change: -delta_tau_max < tau(i+1) - tau(i) < delta_tau_max

Have a look at the code (link to GitHub in the comments) to see how the second constraint is implemented, I think it's interesting!

This is how the control algorithm works:

✅ The function mpc_cost computes J for the next N (20) steps, starting from the current state
✅ The function solve_mpc sets the constraints and bounds and uses scipy.optimize.minimize to minimise the cost computed by mpc_cost and then returns the optimal control input sequence

The code implements a simulation of the loop for 10 seconds, where the setpoint is 180 deg for the first 5 seconds and 90 deg from 5s to 10s.

The prediction horizon of 20 steps corresponds to 2 seconds with the dt = 0.1s that I used.

I used comments to show the separation between the simulation code and the control code (which should be implemented on a microcontroller for real-world operation).

Have a look at the simulation result, I find it interesting how different the response is with respect to a linear controller.

It is clear that the control algorithm "knows" what's going to happen next!

This is a simple first example, I plan to use this approach in many others.

If you enjoyed this follow me for more tips on control and embedded software engineering.

Hit the 🔔 on my profile to get a notification for all my new posts.

What do you want to know? Ask in the comments!

#controlsystems #embeddedsystems #softwareengineering #embeddedsoftware #coding #controltheory #python #mpc
