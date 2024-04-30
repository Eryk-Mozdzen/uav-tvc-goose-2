import sympy
from sympy import Symbol, Quaternion, Matrix
import os

dt = Symbol('\Delta t')

qw, qx, qy, qz = sympy.symbols('q_w q_x q_y q_z')
thetad = sympy.Symbol('\\theta_d')
wx, wy, wz = sympy.symbols('w_x w_y w_z')

q = Quaternion(qw, qx, qy, qz, norm=1)

x = Matrix([
    q.to_Matrix(),
    [thetad],
    [wx],
    [wy],
    [wz],
])

f = Matrix([
    (q + 0.5*dt*q*Quaternion(0, wx, wy, wz)).to_Matrix(),
    [thetad],
    [wx],
    [wy],
    [wz],
])

h_accel = q.to_rotation_matrix().transpose()*Matrix([[0], [0], [-1]])
h_gyro = Matrix([[wx], [wy], [wz]])
h_mag = q.to_rotation_matrix().transpose()*Matrix([[0], [sympy.cos(thetad)], [sympy.sin(thetad)]])

with open('estimator.tex', 'w') as file:
    file.write('\\documentclass{article}\n')
    file.write('\\usepackage[margin=10mm]{geometry}\n')
    file.write('\\usepackage{amsmath}\n')
    file.write('\\begin{document}\n')
    file.write('\\[x_k = ' + sympy.latex(x) + ' = f(x_{k-1}) = ' + sympy.latex(f) + '\\]\n')
    file.write('\\[\\frac{\partial}{\partial x}f(x_{k-1}) = ' + sympy.latex(f.jacobian(x)) + '\\]\n')
    file.write('\\[h_{accel}(x_k) = ' + sympy.latex(h_accel) + '\\]\n')
    file.write('\\[\\frac{\partial}{\partial x}h_{accel}(x_k) = ' + sympy.latex(h_accel.jacobian(x)) + '\\]\n')
    file.write('\\[h_{gyro}(x_k) = ' + sympy.latex(h_gyro) + '\\]\n')
    file.write('\\[\\frac{\partial}{\partial x}h_{gyro}(x_k) = ' + sympy.latex(h_gyro.jacobian(x)) + '\\]\n')
    file.write('\\[h_{mag}(x_k) = ' + sympy.latex(h_mag) + '\\]\n')
    file.write('\\[\\frac{\partial}{\partial x}h_{mag}(x_k) = ' + sympy.latex(h_mag.jacobian(x)) + '\\]\n')
    file.write('\\end{document}\n')

#os.system('pdflatex estimator.tex > /dev/null')
os.system('pdflatex estimator.tex -interaction=nonstopmode')
