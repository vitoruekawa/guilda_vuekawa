syms x1 x2 a 
f = [x2 - (x1^3 - a * x1 * x2^2) * (x1^2 + 2*x2^2);
     -2 * x1 + (6* a * x1^2 * x2 - 8 * x2^3) * (x1^2 + 2 * x2^2)];

v_fun = x1^2 + x2^2;

v_dot = transpose(gradient(v_fun)) * f;

