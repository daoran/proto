H = csvread("/tmp/H.csv");
m = 6;
r = 35;

Hmm = H(1:m, 1:m);
Hmr = H(1:m, m+1:end);
Hrm = H(m+1:end, 1:m);
Hrr = H(m+1:end, m+1:end);

[V, lambda] = eig(Hmm);
V
lambda

cond(Hmm)
cond(Hmm + 1e-4 * I(m))
Hmm_inv = pinv(Hmm)
H_marg = Hrr - Hrm * Hmm_inv * Hmr;

imshow(H_marg)
ginput();
