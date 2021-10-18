from spline import BSpline, BSplineBasis
from spline_extra import definite_integral
import numpy as np
from casadi import MX, vertcat, nlpsol
import math
import matplotlib.pyplot as plt

class SplineVehicle():
    def __init__(self):
        print('Spline vehicle created')
        self.w, self.w_list, self.lbw, self.ubw = [], [], [], []
        self.g, self.g_list, self.lbg, self.ubg = [], [], [], []
        self.J = 0
        self.epsilon = 0.1
        self.safety_weight = 100
        self.knot_intervals = None

    def define_knots(self,degree=3, **kwargs):
        """This function defines the knots and creates the
        B-spline basis function with the prescribed degree.
        Input:
            degree: degree of the B-spline basis functions
            knot_intervals: number of knot intervals
            knots (optional): knot vector. If not given, calculated using the
            number of knot_intervals
        Returns:
            basis: array of B-spline basis functions
            knots: the knot vector
            knot_intervals
        """
        def f(x):
            return np.sqrt(x)
                
        
        if 'knot_intervals' in kwargs:
            knot_intervals = kwargs['knot_intervals']
            # knots = np.r_[np.zeros(degree),
            #               np.linspace(0, 1, knot_intervals+1),
            #               np.ones(degree)]
            knots = np.r_[np.zeros(degree),
                          f(np.linspace(0, 1, knot_intervals+1)),
                          np.ones(degree)]
        if 'knots' in kwargs:
            knot_intervals = len(knots) - 2*degree - 1
            knots = kwargs['knots']

        basis = BSplineBasis(knots, degree)

        return basis

    def define_splines(self, degree, knot_intervals, n_spl, lower_bound, upper_bound, name = ''):
        """This function defines a set of splines.
        Input:
            basis: the basis function to define the spline with. If not provided,
            self.basis will be used.
            n_spl: number of splines to define
        Returns:
            a B-spline class
        """
        basis = self.define_knots(degree = degree, knot_intervals = knot_intervals)

        splines = []
        for k in range(n_spl):
            coeffs = MX.sym(name[k], len(basis))
            self.w += [coeffs]
            self.w_list += [name[k] for i in range(len(basis))]
            splines += [BSpline(basis, coeffs)]
            
            
        for i in range(len(splines)):
            for j in range(splines[i].coeffs.shape[0]):
                self.lbw += [lower_bound[i]]
                self.ubw += [upper_bound[i]]

        return splines

    def define_constraint(self, constraint, lower_bound, upper_bound, constraint_type = 'overall', name = ''):
        """This function defines constraint on the b_spline coefficients
        Input:
            constraint (list): a spline constraint. We will set upper and lower bounds on its coefficients
            lower_bound (list): the lower bound of the coefficients
            upper_bound (list): the upper bound of the coefficients
            constraint_type: 'overall', 'initial', 'final'
        Returns:

        """
        if constraint_type == 'overall':
            for i in range(len(constraint)):
                for j in range(constraint[i].coeffs.shape[0]):
                    self.g += [constraint[i].coeffs[j]]
                    self.g_list += [name]
                    self.lbg += [lower_bound[i]]
                    self.ubg += [upper_bound[i]]
            return self

        elif constraint_type == 'initial':
            for i in range(len(constraint)):
                self.g += [constraint[i].coeffs[0]] # we restrict the first coefficient
                self.g_list += [name[i] + "_0"]
                self.lbg += [lower_bound[i]]
                self.ubg += [upper_bound[i]]
            return self
        elif constraint_type == 'final':
            for i in range(len(constraint)):
                self.g += [constraint[i].coeffs[-1]] # we restrict the last coefficient
                self.g_list += [name[i] + "_f"]
                self.lbg += [lower_bound[i]]
                self.ubg += [upper_bound[i]]
            return self

        else:
            raise NotImplementedError()
            
    def collision_avoidance_circular(self, splines, center, radious, name):
        """This function defines constraints on the splines to avoid the space arodund
        a certaint point with a given radious.
        Input:
            splines (list): a spline on which we want to set final constraint
            center: the point to avoid
            radious: the minimum distance from the center point
        Returns:

        """

        constraint = (splines[0] - center[0])**2 + (splines[1] - center[1])**2
        self.define_constraint([constraint], lower_bound = [radious**2], upper_bound = [math.inf, math.inf], name = name)

    def collision_avoidance_hyperplane(self, splines, center, radious, name):

        # a
        a = self.define_splines(degree = 1, knot_intervals = self.knot_intervals, n_spl = len(splines),
                                lower_bound = [-math.inf] * len(splines),
                                upper_bound = [math.inf] * len(splines),
                                name = ["a"+ str(i) for i in range(len(splines))])

        # b
        b = self.define_splines(degree = 1, knot_intervals = self.knot_intervals, n_spl = 1,
                                lower_bound = [-math.inf],
                                upper_bound = [math.inf],
                                name = ["b"])

        # d_tau
        d_tau = self.define_splines(degree = 1, knot_intervals = self.knot_intervals, n_spl = 1,
                                lower_bound = [0],
                                upper_bound = [math.inf],
                                name = ["d_tau"])


        const1 = -a[0]*splines[0] - a[1]*splines[1] + b[0] - d_tau[0]
        const2 = a[0] * center[0] + a[1] * center[1] - b[0]
        const3 = a[0] * a[0] + a[1] * a[1]

        self.define_constraint([const1], lower_bound = [radious], upper_bound = [math.inf], name = "eq1")
        self.define_constraint([const2], lower_bound = [0], upper_bound = [math.inf], name = "eq2")
        self.define_constraint([const3], lower_bound = [-math.inf], upper_bound = [1.0], name = "eq3")

        self.J += self.safety_weight * definite_integral((self.epsilon - d_tau[0])**2, 0, 1)




# sv --> spline_vehicle
sv = SplineVehicle()
sv.knot_intervals = 100
x = sv.define_splines(degree = 3, knot_intervals = sv.knot_intervals, n_spl = 2,
                      lower_bound = [-10, -10],
                      upper_bound = [10, 10], name = ["x", "y"] )


# Setting objective
# J = definite_integral(x[0]**2 + x[1]**2, 0, 1)
sv.J += definite_integral(x[0].derivative()**2 + x[1].derivative()**2, 0.9, 1) # + dot(x[0].coeffs, x[0].coeffs) + dot(x[1].coeffs, x[1].coeffs)
# for i in range(x[0].coeffs.shape[0]):
#     J += 0.1 * x[0].coeffs[i]

# Setting initial constraint
sv.define_constraint(x, [0, 0], [0, 0], 'initial', name = ["x", "y"] )
sv.define_constraint(x, [2, 2], [2, 2], 'final', name = ["x", "y"] )
radious = np.sqrt( 0.5**2 + 0.5**2)
corner = [1.5, 1.5]
corner = [1, 1]
# sv.collision_avoidance_circular(x, [1, 1], radious)
sv.collision_avoidance_hyperplane(x, corner, radious, name = ["x", "y"] )


prob = {'f': sv.J,
        'x': vertcat(*sv.w),
        'g': vertcat(*sv.g)
        }

solver = nlpsol('solver', 'ipopt', prob)

arg = { 'lbx': sv.lbw,
        'ubx': sv.ubw,
        'lbg': sv.lbg,
        'ubg': sv.ubg
        }


solution = solver.call(arg)

coeff_solutions = solution['x'].full()

x_sol = x


# x_sol[0].coeffs = coeff_solutions[:len(coeff_solutions)//2]
# x_sol[1].coeffs = coeff_solutions[len(coeff_solutions)//2:]
x_sol[0].coeffs = coeff_solutions[:x[0].coeffs.shape[0]]
x_sol[1].coeffs = coeff_solutions[x[0].coeffs.shape[0]:x[0].coeffs.shape[0]*2]


# spline_sol = [BSpline(x[0].basis, x_sol[0].coeffs), BSpline(x[1].basis, x_sol[1].coeffs)]
# J = definite_integral(x_sol[0]**2 + x_sol[1]**2, 0, 1)
J_sol = (x_sol[0]**2).integral() + (x_sol[1]**2).integral()
print(J_sol)
print(solution['f'].full())
t = np.linspace(0, 1, 1000)

x_traj = []
y_traj = []


for i in range(len(t)):
    x_traj.append(float(x_sol[0](t[i])[0]))
    y_traj.append(float(x_sol[1](t[i])[0]))

plt.close('all')
fig, ax = plt.subplots(1, 1)
ax.plot(x_sol[0].coeffs, x_sol[1].coeffs, 'ro')
ax.plot(x_traj, y_traj, '.')

circle1 = plt.Circle(corner, radious, color='k', alpha=0.5, zorder = 10)
ax.add_patch(circle1)
plt.show()

plt.figure(1)
fig, axs = plt.subplots(2, 1)
axs[0].plot(t, x_traj)
axs[0].plot(x_sol[0].basis.knots[2:-2], x_sol[0].coeffs, 'ro')
axs[1].plot(t, y_traj)
axs[1].plot(x_sol[0].basis.knots[2:-2], x_sol[1].coeffs, 'ro')
