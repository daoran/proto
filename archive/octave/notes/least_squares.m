% Settings
N = 100;
mu_x = 5.0;
sigma_x = 1.0;
mu_y = 5.0;
sigma_y = 0.1;

% Generate some test data
m_gnd = 2.0;
c_gnd = 1.0;
noise = normrnd(0, 2.0, N, 1);
x = normrnd(mu_x, sigma_x, N, 1);
y = (m_gnd * x + c_gnd) + noise;

% y = m x + c
A = [x, ones(N, 1)];

% (A' * A) * b = A' * y
b = (A' * A)^-1 * A' * y
m = b(1)
c = b(2)

% Visualize
figure(1);
plot(x, y, 'ro');
y_start = m * 0 + c;
y_end = m * 10 + c;
line([0, 10], [y_start, y_end]);
ginput();
