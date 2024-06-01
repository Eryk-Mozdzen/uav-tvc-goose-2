import sympy
import os

px, py, k1, k2 = sympy.symbols('x y K_1 K_2')
x1, y1 = sympy.symbols('x_1 y_1')
x2, y2 = sympy.symbols('x_2 y_2')
x3, y3 = sympy.symbols('x_3 y_3')

x = sympy.Matrix([
    px,
    py,
    k1,
    k2
])

f = sympy.Matrix([
    px,
    py,
    k1,
    k2
])

h_rssi_1 = sympy.Matrix([[k1 + k2*sympy.log(sympy.sqrt((px-x1)**2 + (py-y1)**2), 10)]])

output_directory = os.path.dirname(__file__)

os.makedirs(output_directory, exist_ok=True)
os.makedirs(output_directory + '/docs', exist_ok=True)

with open(output_directory + '/docs/estimator.tex', 'w') as file:
    file.write('\\documentclass{article}\n')
    file.write('\\usepackage{amsmath}\n')
    file.write('\\usepackage[paperwidth=50cm, paperheight=50cm, margin=10mm]{geometry}\n')
    file.write('\n')
    file.write('\\begin{document}\n')
    file.write('\t\\[x_k = ' + sympy.latex(x) + ' = f(x_{k-1}, u_k) = ' + sympy.latex(f) + '\\]\n')
    file.write('\t\\[\\frac{\partial}{\partial x}f(x_{k-1}, u_k) = ' + sympy.latex(f.jacobian(x)) + '\\]\n')
    file.write('\t\\[h_{rssi1}(x_k) = ' + sympy.latex(h_rssi_1) + '\\]\n')
    file.write('\t\\[\\frac{\partial}{\partial x}h_{rssi1}(x_k) = ' + sympy.latex(h_rssi_1.jacobian(x)) + '\\]\n')
    file.write('\\end{document}\n')

os.system('pdflatex -interaction=nonstopmode -output-directory=' + output_directory + '/docs ' + output_directory + '/docs/estimator.tex')
