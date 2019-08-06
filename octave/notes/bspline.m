function s = deboor(k, x, t, c, p)
	% Evaluates S(x).
	%
	% Args
	% ----
	% k: index of knot interval that contains x
	% x: position
	% t: array of knot positions, needs to be padded as described above
	% c: array of control points
	% p: degree of B-spline
	% """
	d = zeros(p + 1);
	for j = 1:p+1
		d(j) = c(j + k - p)
	endfor

	for r = 1:p+1
		for j = p:r-1:-1
			% alpha = (x - t[j+k-p]) / (t[j+1+k-r] - t[j+k-p])
			% d[j] = (1.0 - alpha) * d[j-1] + alpha * d[j]
		endfor
	endfor

	d(p)
endfunction

# Signal
f = 1.0;
t = linspace(0.0, 1.0, 15);
y = sin(2 * pi * f * t);

# Plot
figure();
plot(t, y, "r-", "linewidth", 2.0);
ginput();


% int main()
% {
%     vec x(11);
%     vec y(11);
%     for(int i = 0; i < x.size(); ++i)
%     {
%         x[i] = i;
%         y[i] = sin(i);
%     }
%
%     vector<SplineSet> cs = spline(x, y);
%     for(int i = 0; i < cs.size(); ++i)
%         cout << cs[i].d << "\t" << cs[i].c << "\t" << cs[i].b << "\t" << cs[i].a << endl;
% }
