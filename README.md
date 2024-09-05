# closed_loop_ankle

​		A mainstream 2-DOF humanoid ankle simulation in Webots. This implementation includes forward and inverse kinematics for a closed-loop ankle, enabling torque mapping between the two motors and the ankle joints.  Only for ankle like this:

![ankle]((https://github.com/HuNingHe/closed_loop_ankle/blob/main/pictures/ankle.gif))

# Test

- Ubuntu24.04
- WebotsR2023b

# Request

- Eigen
- [ros2_plot](https://github.com/HuNingHe/ros2_plot)
- Webots

# Build

```bash
git clone https://github.com/HuNingHe/closed_loop_ankle.git
cd closed_loop_ankle/controllers/my_controller && mkdir build
cd build && cmake ../
make -j4
```

# Usage

```bash
cd ros2_plot_ws && source install/setup.bash
ros2 run ros2_plot ros2_plot # run ros2_plot first!!!!
ros2 run plotjuggler plotjuggler
# Open webots and run ankle.wbt
```

​		If you are not using ros2, you can remove all shared memory related code and recompile, you can run still without data visualization.

​		set `test_kinematics` in `my_controller.cpp`(line 352) to false, then you can test torque mapping between motors and ankle joint.

​		**suggest to use plot.xml of ros2_plot in plotjuggler**.

# Mathematics

​		You can find the formula below in the code:
$$
\begin{bmatrix}
    \theta_{1} \\
    \theta_{2}
\end{bmatrix}
=
f_1(\theta_{pitch},\theta_{roll}, d, L_1,h_1,h_2)
$$

​		$f_1$ is `ankle_ik`
$$
\begin{bmatrix}
    \dot{\theta}_{1} \\
    \dot{\theta}_{2}
\end{bmatrix}
=
\begin{bmatrix}
    \frac{\delta f_{11}}{\delta \theta_{pitch}} & \frac{\delta f_{12}}{\delta \theta_{roll}} \\
    \frac{\delta f_{21}}{\delta \theta_{pitch}} & \frac{\delta f_{22}}{\delta \theta_{roll}}
\end{bmatrix}
\begin{bmatrix}
    \dot{\theta}_{pitch} \\
    \dot{\theta}_{roll}
\end{bmatrix}=J_2\begin{bmatrix}
    \dot{\theta}_{pitch} \\
    \dot{\theta}_{roll}
\end{bmatrix}
$$
​		$\frac{\delta f_{11}}{\delta \theta_{pitch}}$ is `computeTmLtY`, $\frac{\delta f_{12}}{\delta \theta_{roll}}$ is `computeTmLtX`, $\frac{\delta f_{21}}{\delta \theta_{pitch}}$ is `computeTmRtY`, $\frac{\delta f_{22}}{\delta \theta_{roll}}$ is `computeTmRtX`,or general case $J_2=J_1^{-1}$

$$
\begin{bmatrix}
    \theta_{pitch} \\
    \theta_{roll}
\end{bmatrix}
=
\begin{bmatrix}
0.5(\theta_{1}+\theta_{2}) \\
f_2(\theta_{1},\theta_{2}, d, L_1,h_1,h_2)
\end{bmatrix}
$$

​		$f_2$ is `calculateTx`
$$
\begin{bmatrix}
    \dot{\theta}_{pitch} \\
    \dot{\theta}_{roll}
\end{bmatrix}
=
\begin{bmatrix}
    0.5 & 0.5 \\
    \frac{\delta f_{2}}{\delta \theta_1} & \frac{\delta f_{2}}{\delta \theta_2}
\end{bmatrix}
\begin{bmatrix}
    \dot{\theta}_{1} \\
    \dot{\theta}_{2}
\end{bmatrix}=J_1\begin{bmatrix}
    \dot{\theta}_{1} \\
    \dot{\theta}_{2}
\end{bmatrix}
$$

​		$\frac{\delta f_{2}}{\delta \theta_{1}}$ is `calculateTxML`, $\frac{\delta f_{2}}{\delta \theta_{2}}$ is `calculateTxMR`

​		according to the principle of virtual work, we have:
$$
\begin{bmatrix}
    \tau_1 \\
    \tau_2
\end{bmatrix}=J_1^T
\begin{bmatrix}
    \tau_{pitch} \\
    \tau_{roll}
\end{bmatrix}
$$

$$
\begin{bmatrix}
    \tau_{pitch} \\
    \tau_{roll}
\end{bmatrix}=J_2^T\begin{bmatrix}
    \tau_1 \\
    \tau_2
\end{bmatrix}
$$



​		Both of 2 motors and 2 joints of the ankle follow the right-hand rule: rotation around the Y-axis in the counterclockwise direction is considered positive, with the X-axis pointing forward and the Z-axis pointing vertically.

​		Some notations:![ankle_d](/home/henning/Pictures/ankle_d.jpg)

![ankle_h1h2L1](/home/henning/Pictures/ankle_h1h2L1.png)
