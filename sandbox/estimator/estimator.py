import sympy
from sympy import Symbol, Quaternion, Matrix
import os

dt = Symbol('\Delta t')
g = Symbol('g')

qw, qx, qy, qz = sympy.symbols('q_w q_x q_y q_z')
thetad = sympy.Symbol('\\theta_d')
wx, wy, wz = sympy.symbols('w_x w_y w_z')
ax, ay, az = sympy.symbols('\\ddot{p}_x \\ddot{p}_y \\ddot{p}_z')
vx, vy, vz = sympy.symbols('\\dot{p}_x \\dot{p}_y \\dot{p}_z')
px, py, pz = sympy.symbols('p_x p_y p_z')
p0 = sympy.symbols('p_0')

q = Quaternion(qw, qx, qy, qz, norm=1)
w = Matrix([[wx], [wy], [wz]])
a = Matrix([[ax], [ay], [az]])
v = Matrix([[vx], [vy], [vz]])
p = Matrix([[px], [py], [pz]])

x = Matrix([
    q.to_Matrix(),
    [thetad],
    w,
    p,
    v,
    a,
    p0,
])

f = Matrix([
    (q + 0.5*dt*q*Quaternion(0, wx, wy, wz)).to_Matrix(),
    [thetad],
    w,
    p + dt*v + 0.5*dt**2*a,
    v + dt*a,
    a,
    p0,
])

h_accel = q.to_rotation_matrix().transpose()*(Matrix([[0], [0], [-g]]) + a)
h_gyro = w
h_mag = q.to_rotation_matrix().transpose()*Matrix([[0], [sympy.cos(thetad)], [sympy.sin(thetad)]])
h_range = Matrix([pz])
h_press = Matrix([p0*sympy.Pow(1 - pz/44330, 5.255)])
h_gps = Matrix([[px], [py]])

with open('output.tex', 'w') as file:
    file.write('\\documentclass{article}\n')
    file.write('\\usepackage[paperwidth=40cm, paperheight=40cm, margin=10mm]{geometry}\n')
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
    file.write('\\[h_{range}(x_k) = ' + sympy.latex(h_range) + '\\]\n')
    file.write('\\[\\frac{\partial}{\partial x}h_{range}(x_k) = ' + sympy.latex(h_range.jacobian(x)) + '\\]\n')
    file.write('\\[h_{press}(x_k) = ' + sympy.latex(h_press) + '\\]\n')
    file.write('\\[\\frac{\partial}{\partial x}h_{press}(x_k) = ' + sympy.latex(h_press.jacobian(x)) + '\\]\n')
    file.write('\\[h_{gps}(x_k) = ' + sympy.latex(h_gps) + '\\]\n')
    file.write('\\[\\frac{\partial}{\partial x}h_{gps}(x_k) = ' + sympy.latex(h_gps.jacobian(x)) + '\\]\n')
    file.write('\\end{document}\n')

os.system('pdflatex output.tex -interaction=nonstopmode')
