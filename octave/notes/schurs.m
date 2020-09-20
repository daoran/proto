H = csvread("/tmp/H.csv");

% m = 12;
% r = rows(H) - m;
%
H = H + 1e-5 * diag(H);
%
% Hmm = H(1:m, 1:m);
% Hmr = H(1:m, m+1:end);
% Hrm = H(m+1:end, 1:m);
% Hrr = H(m+1:end, m+1:end);
%
% cond(Hmm)
%
% % [V, lambda] = eig(Hmm);
% Hmm_inv = inv(Hmm);
% H_marg = Hrr - Hrm * Hmm_inv * Hmr;

% cond(H)
% covar = inv(H);
% imagesc(covar)
% colorbar()
% ginput()

printf("rows(H): %d\n", rows(H));
printf("rank(H): %d\n", rank(H));
% printf("rows(H_marg): %d\n", rows(H_marg));
% printf("rank(H_marg): %d\n", rank(H_marg));
% printf("det(H_marg): %f\n", det(H_marg));
% printf("det(covar): %f\n", det(covar));

figure(1);
imagesc(H);
colorbar();

% figure(2);
% imagesc(H_marg);
% colorbar();

ginput();
