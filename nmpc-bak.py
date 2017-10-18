import numpy as np
from scipy.optimize import minimize


def measureInitialValue(tmeasure, xmeasure):
    return tmeasure, xmeasure


def solveOptimalControlProblem(system, N, t0, x0, u0, T):
    output = 'info'

    #sol = minimize(cost)
    #y = runningCosts( np.array([1,2]), np.array([3]) )
    #x = x0[0]
    #u = u0[0]
    #y = runningCosts(x0, np.array([3]))
    # u_new = np.array([sol['x']])

    # closed-loop control
    # y = costFunction(runningCosts, system, N, T, t0, x0, u0)
    sol = minimize( costFunction, x0, args = (N, T, t0, x0, u0) )

    # a = 1
    # b = 2
    # c = 3
    # d = 4
    # sol = minimize(f,x0,args=(a,b,c,d))
    # print(sol['x'])


    # open-loop control
    u_new = np.array([[1,1,1,1,1,1,1,1]])


    V = 0
    exitflag = 0
    return u_new, V, exitflag, output


def f(x,a,b,c,d):
    return a*(x[0]-1)**2 + b*x[1]**2 + f2(x,c,d)

def f2(x,c,d):
    return c*x[0] + d*x[1]


def runningCosts(t, x, u):
    Q = np.array([[100]])
    R = np.array([[1]])
    r = np.array([1])
    e = r - x[0]
    cost = np.matmul(e.T,np.matmul(Q,e)) + np.matmul(u.T,np.matmul(R,u)) #(r - x(1))*Q*(r-x(1)) +
    return cost


def costFunction(runningcosts, system, N, T, t0, x0, u):
    # cost = 0;
    # x = computeOpenloopSolution(system, N, T, t0, x0, u);
    #
    # for k=1:N
    # costvec(k) = runningcosts(t0 + k * T, x(k,:), u(:, k));
    # cost = cost + costvec(k);
    # end
    cost = 0

    x = computeOpenloopSolution(system, N, T, t0, x0, u)

    costvec = np.zeros([N,1])

    for k in range(N):
        costvec[k] = runningcosts(t0 + k*T, x[k,:], u[:, k])
        cost = cost + costvec[k]

    return cost


def system(xk, uk, T):
    xkp1 = np.zeros(2)
    xkp1[0] = xk[0] + T*xk[1]
    xkp1[1] = xk[1] + T*uk[0]
    return xkp1


def applyControl(system, T, t0, x0, u):
    u0 = u[:,0]
    xapplied = system(x0, u0, T)
    tapplied = t0+T
    return tapplied, xapplied


def shiftHorizon(u):
    u0 = np.full_like(u,1)
    n = int(u0.size)
    a = np.array([u[0,0:n-1]])
    b = np.array([[u[0,n-1]]])
    u0 = np.concatenate([a,b],axis=1)
    return u0


def computeOpenloopSolution(system, N, T, t0, x0, u):
    x = np.zeros([N,np.size(x0)])
    x[0] = x0

    for k in range(N-1):
        x[k+1] = system(x[k], u[:, k], T)

    return x