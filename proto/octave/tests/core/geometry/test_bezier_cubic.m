addpath(genpath("proto"));

P0 = [0; 0];
C0 = [3; -3];
C1 = [7; 7];
P1 = [10; 10];

s = [];
for t = 0:0.01:1.0
  s = [s, bezier_cubic(P0, C0, C1, P1, t)];
endfor
assert(any(s(1:2, 1) == P0) == 1);
assert(any(s(1:2, end) == P1) == 1);

debug = false;
if debug
  figure(1);
  hold on;
  plot(P0(1), P0(2), 'ro', 'linewidth', 3);
  plot(P1(1), P1(2), 'ro', 'linewidth', 3);
  plot(C0(1), C0(2), 'bo', 'linewidth', 3);
  plot(C1(1), C1(2), 'bo', 'linewidth', 3);
  plot(C1(1), C1(2), 'bo', 'linewidth', 3);
  plot(s(1, 1:end), s(2, 1:end), 'k-', 'linewidth', 3);
  ginput();
end
