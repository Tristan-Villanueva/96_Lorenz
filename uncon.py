from scipy.integrate import odeint
import matplotlib.pyplot as plt
import numpy as np

# These are our constants
N = 40  # Number of variables
F = 15  # Forcing


def L96(x, t):
    """Lorenz 96 model with constant forcing"""
    return (np.roll(x, -1) - np.roll(x, 2)) * np.roll(x, 1) - x + F 


x0 = F * np.ones(N)  # Initial state (equilibrium)
x0[0] += 0.01  # Add small perturbation to the first variable
t = np.arange(0.0, 1000.0, 0.01)

x = odeint(L96, x0, t)

# Plot the first three variables
fig = plt.figure()
ax = fig.add_subplot(projection="3d")
ax.plot(x[:, 0], x[:, 1], x[:, 2])
ax.set_xlabel("$x_1$")
ax.set_ylabel("$x_2$")
ax.set_zlabel("$x_3$")
plt.show()
# plot of x_1 vs x_2
plt.figure()
plt.plot(x[:, 0], x[:, 1])
plt.xlabel("$x_1$")
plt.ylabel("$x_2$")
plt.show()
# plot of x_1 vs x_3
plt.figure()
plt.plot(x[:, 0], x[:, 2])
plt.xlabel("$x_1$")
plt.ylabel("$x_3$")
plt.show()
# plot of x_2 vs x_3
plt.figure()
plt.plot(x[:, 1], x[:, 2])
plt.xlabel("$x_2$")
plt.ylabel("$x_3$")
plt.show()