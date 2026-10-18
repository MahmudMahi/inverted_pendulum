# Inverted Pendulum: System Modeling, Stability Analysis & LQR Control

A case study on modeling, linearizing, and stabilizing a non-linear Cart-Pole (Inverted Pendulum) dynamic system. Based on system dynamics from *"Feedback Systems: An Introduction for Scientists and Engineers"* by Åström and Murray.

## Overview

The inverted pendulum on a cart is a classic benchmark problem in control theory. This project demonstrates:

1. **First-Principles Modeling:** Formulating non-linear state equations using Euler-Lagrange / matrix dynamic representations.

2. **Open-Loop Dynamics:** Simulating the open-loop non-linear response using MATLAB's `ode45`.

3. **Linearization & Stability Analysis:** Computing Jacobian matrices at equilibrium points and analyzing open-loop stability via state eigenvalues.

4. **Controllability Check:** Verifying full controllability via the controllability matrix rank test.

5. **Optimal Control (LQR):** Tuning state ($Q$) and effort ($R$) weighting matrices using **Bryson's Rule** to stabilize the upright system under severe initial perturbations ($45^\circ$ initial tilt).

## System Dynamics & Equations

### Equations of Motion

The non-linear system dynamics are represented in matrix differential form:

$$
M(q)\ddot{q} + C(q, \dot{q})\dot{q} + K(q) = B(q)u
$$

Where:

* Generalized coordinates: $q = \begin{bmatrix} x_{\text{cart}} \\ \theta \end{bmatrix}$

* Input Matrix: $B(q)u = \begin{bmatrix} 1 \\ 0 \end{bmatrix} F$

#### Mass Matrix $M(q)$:

$$
M(q) = \begin{bmatrix} M + m & ml \cos\theta \\ ml \cos\theta & J_t \end{bmatrix}
$$

#### Coriolis & Damping Vector $C(q, \dot{q})$:

$$
C(q, \dot{q}) = \begin{bmatrix} b & -ml \sin\theta \cdot \dot{\theta} \\ 0 & \gamma \end{bmatrix}
$$

#### Potential Energy Force Vector $K(q)$:

$$
K(q) = \begin{bmatrix} 0 \\ -mgl \sin\theta \end{bmatrix}
$$

*(where* $J_t = J + ml^2$ *is the total moment of inertia about the pivot,* $b$ *is cart viscous friction, and* $\gamma$ *is pivot damping).*

### State-Space Representation

Converting the coupled 2nd-order ODEs into 4 first-order ODEs:

$$
\mathbf{z} = \begin{bmatrix} z_1 \\ z_2 \\ z_3 \\ z_4 \end{bmatrix} = \begin{bmatrix} x_{\text{cart}} & \text{(Cart position)} \\ \dot{x}_{\text{cart}} & \text{(Cart velocity)} \\ \theta & \text{(Pendulum angle)} \\ \dot{\theta} & \text{(Angular velocity)} \end{bmatrix}
$$

Solving for accelerations via matrix inverse:

$$
\begin{bmatrix} \ddot{x}_{\text{cart}} \\ \ddot{\theta} \end{bmatrix} = M(q)^{-1} \left( B(q)u - C(q, \dot{q})\dot{q} - K(q) \right)
$$

> **Why convert 2nd-order ODEs to 1st-order state-space?**
>
> 1. MATLAB solvers like `ode45` require standard $\dot{\mathbf{z}} = f(t, \mathbf{z}, u)$ vector format.
>
> 2. Control algorithms like LQR regulate state vectors. Without tracking velocity separately, the controller might attempt to zero out acceleration while the cart drifts at dangerous velocities (e.g., $40\text{ km/h}$).

## Physical Parameters

| Parameter | Symbol | Value | Units | Description | 
| ----- | ----- | ----- | ----- | ----- | 
| **Cart Mass** | $M$ | `0.5` | $\text{kg}$ | Mass of the cart base | 
| **Pendulum Mass** | $m$ | `0.2` | $\text{kg}$ | Mass of the pendulum link | 
| **Length to COM** | $l$ | `0.3` | $\text{m}$ | Distance from pivot to COM | 
| **Pendulum Inertia** | $J$ | `0.005` | $\text{kg}\cdot\text{m}^2$ | Inertia about center of mass | 
| **Gravity** | $g$ | `9.81` | $\text{m/s}^2$ | Acceleration due to gravity | 
| **Cart Friction** | $b$ | `0.1` | $\text{N}\cdot\text{s/m}$ | Viscous damping of cart | 
| **Pivot Friction** | $\gamma$ | `0.0` | $\text{N}\cdot\text{m}\cdot\text{s/rad}$ | Viscous damping of pivot | 

## Linearization & Open-Loop Analysis

Evaluating the symbolic Jacobians about the upright equilibrium point $\mathbf{z}_{\text{fixed}} = [0, 0, 0, 0]^T$ and $u_{\text{fixed}} = 0$:

$$
A = \left.\frac{\partial f}{\partial \mathbf{z}}\right\vert{}_{\mathbf{z}_0, u_0}, \quad B = \left.\frac{\partial f}{\partial u}\right\vert{}_{\mathbf{z}_0, u_0}
$$

### Open-Loop Eigenvalues

$$
\lambda_{\text{open}} = \begin{bmatrix} 0 \\ -0.1428 \\ -5.6041 \\ \mathbf{+5.5651} \end{bmatrix}
$$

**Stability Diagnosis:**
The existence of a positive real eigenvalue ($\lambda = +5.5651$) proves the continuous open-loop system is **unstable** (the state matrix exponential term $e^{\lambda t}$ grows exponentially over time).

## Controller Design

### 1. Controllability Verification

Before synthesizing state feedback gains, the system controllability matrix is evaluated:

$$
\mathcal{C} = \begin{bmatrix} B & AB & A^2B & A^3B \end{bmatrix}
$$

```
rank(ctrb(A, B)) % Output: 4

```

Since $\text{rank}(\mathcal{C}) = n = 4$, the system is fully state-controllable, meaning poles can be arbitrarily placed in the complex plane.

### 2. LQR Synthesis (Bryson's Rule)

The Linear Quadratic Regulator minimizes the performance index:

$$
J = \int_{0}^{\infty} (\mathbf{z}^T Q \mathbf{z} + u^T R u) \, dt
$$

State and input penalties are assigned using **Bryson's Rule** ($Q_{ii} = \frac{1}{\text{max}(z_i)^2}$):

* Max acceptable cart displacement ($z_1$): $\pm 5\text{ m} \implies Q_{11} = 1 / 5^2 = 0.04$

* Max acceptable cart velocity ($z_2$): $\pm 2\text{ m/s} \implies Q_{22} = 1 / 2^2 = 0.25$

* Max acceptable pendulum angle ($z_3$): $\pm 0.1\text{ rad} \ (\approx 5.7^\circ) \implies Q_{33} = 1 / 0.1^2 = 100$

* Max acceptable angular velocity ($z_4$): $\pm 1.5\text{ rad/s} \implies Q_{44} = 1 / 1.5^2 = 0.444$

* Max allowable force ($u$): $\pm 100\text{ N} \implies R = 1 / 100^2 = 0.0001$

```
Q = diag([1/5^2, 1/2^2, 1/0.1^2, 1/1.5^2]);
R = 1/100^2;
K = lqr(A, B, Q, R);

```

### 3. Closed-Loop Stability Verification

Computing eigenvalues of the closed-loop system matrix $(A - BK)$:

$$
\lambda_{\text{closed}} = \begin{bmatrix} -3.1614 \\ -0.1451 \\ -0.0040 + 0.0019i \\ -0.0040 - 0.0019i \end{bmatrix}
$$

All closed-loop poles lie strictly in the **left-half of the complex s-plane** ($\text{Re}(\lambda) < 0$), guaranteeing asymptotic closed-loop stability.

## Simulation Results

### Initial Conditions

Simulation is performed on the **full non-linear model** with a severe disturbance:

$$
\mathbf{z}_0 = \begin{bmatrix} 0\text{ m} & 0\text{ m/s} & \mathbf{45^\circ}\ (\text{0.785 rad}) & 0\text{ rad/s} \end{bmatrix}^T
$$

### Response Characteristics

* **Angle Stabilization:** Because pendulum tilt ($z_3$) carries the highest penalty in the $Q$ matrix ($Q_{33} = 100$), the controller rapidly corrects the $45^\circ$ error, bringing the angle to near zero in approximately $0.13\text{ seconds}$.

* **Control Effort:** Peak initial impulse control force reaches $\approx 849.6\text{ N}$ to arrest the falling pendulum.

* **Cart Position:** The cart initially moves sideways to balance the falling pole, then smoothly returns toward origin without instability.

## Code Structure

```
% Core Non-Linear Function Structure
function dz = cartpoleNL(z, u, p)
    % Extract states
    x = z(1); xdot = z(2); theta = z(3); omega = z(4);
    
    % Mass Matrix
    H = [p.M + p.m,          p.m * p.l * cos(theta);
         p.m * p.l * cos(theta), p.Jt];
         
    % Right Hand Side Vector
    rhs = [u - p.b * xdot + p.m * p.l * omega^2 * sin(theta);
           p.m * p.g * p.l * sin(theta) - p.c * omega];
           
    % Solve accelerations (H \ rhs)
    acc = H \ rhs;
    
    % Return state derivatives
    dz = [xdot; acc(1); omega; acc(2)];
end

```

## References

1. **Åström, K. J., & Murray, R. M.** *Feedback Systems: An Introduction for Scientists and Engineers*. Caltech.

2. **Brunton, S.** *Control Bootcamp Video Series*.

3. **Bryson's Rule Discussion:** [Reddit Robotics Community Thread](https://www.reddit.com/r/robotics/comments/19evuua/is_there_a_theoretical_foundation_for_brysons_rule/)
