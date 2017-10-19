import numpy as np
import nmpc
import matplotlib.pyplot as plt

mpciterations = 101
N = 8
T = 0.1
t0 = 0
x0 = np.zeros(2)
u0 = np.ones([1,N])

tmeasure = t0
xmeasure = x0

t = np.zeros([mpciterations, 1])
x = np.zeros([mpciterations, 2])
u = np.zeros([mpciterations, 1])
u_new = np.array([0])

mpciter = 0

while mpciter < mpciterations:

    #  get new initial value
    t0, x0 = nmpc.measureInitialValue(tmeasure, xmeasure)

    # solve optimal control problem
    u_new, V, exitflag, output = nmpc.solveOptimalControlProblem(nmpc.system, N, t0, x0, u0, T)

    # store closed loop data
    t[mpciter] = tmeasure
    x[mpciter, 0] = xmeasure[0]
    x[mpciter, 1] = xmeasure[1]
    u[mpciter] = u_new[0, 0]

    # apply control
    tmeasure, xmeasure = nmpc.applyControl(nmpc.system, T, t0, x0, u_new)

    # prepare restart
    u0 = nmpc.shiftHorizon(u_new)


    mpciter = mpciter + 1

# print(t)
# print(x)
# print("done!")

f, ax = plt.subplots(3, sharex = True)
ax[0].plot(t,x[:,[0]])
ax[1].plot(t,x[:,[1]])
ax[2].plot(t,u)

for k in range(3):
    ax[k].grid(True)

ax[0].set_ylabel('x(1)')
ax[1].set_ylabel('x(2)')
ax[2].set_ylabel('u')
ax[2].set_xlabel('t [sec]')
plt.show()

