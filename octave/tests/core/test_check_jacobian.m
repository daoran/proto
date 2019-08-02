addpath(genpath("proto"));

step_size = 1e-6;
x = 2;
y0 = x**2;
y1 = (x+step_size)**2;
jac = 2 * x;
fdiff = y1 - y0;

jac_name = "jac";
fdiff = (y1 - y0) / step_size;
threshold = 1e-5;
retval = check_jacobian(jac_name, fdiff, jac, threshold);
assert(retval == 0);
