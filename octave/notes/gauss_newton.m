f = @(x) [ x(1)-.4; x(2)-.8 ; x(1)^2+x(2)^2-1 ];
fp = @(x) [ 1,0 ; 0,1 ; 2*x(1),2*x(2) ];  % define Jacobian fp(x)

format long g        % show all digits

x = [0;0];           % initial guess
while 1
  b = f(x);          % evaluate f
  A = fp(x);         % evaluate Jacobian
  d = -A\b;          % solve linear least squares problem norm(A*d+b)=min
  x = x + d
  if norm(d)<=1e-15  % stop iteration if norm(d)<=StepTolerance
    break
  end
end

xs = x;              % xs is exact solution which we just found
x = [0;0];
enormold = NaN;
for i=1:10
  b = f(x);
  A = fp(x);
  d = -A\b;
  x = x + d;
  enorm = norm(x-xs);
  fprintf('enorm = %g, quotient = %g\n',enorm,enorm/enormold)
  enormold = enorm;
end
