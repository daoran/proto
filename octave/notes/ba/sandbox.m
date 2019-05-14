% function costs = calc_individual_costs(residuals)
%   N = length(residuals) / 2.0;
%
%   costs = [];
%   for i = 1:N
%     rx = residuals(((i - 1) * 2 + 1));
%     ry = residuals(((i - 1) * 2 + 2));
%
%     r = [rx; ry];
%     costs = [costs; 0.5 * r' * r];
%   endfor
% endfunction

% costs = calc_individual_costs(r);
% N = length(costs)
% mu = (costs' * costs) / (N - 1)
% sigma_sq = ((costs - mu)' * (costs - mu)) / (N - 1)

% figure();
% hold on;
% histfit(r);
% ginput();

% chi2 = ((r - mu)' * (r - mu)) / sigma_sq
% dof = 90;
% reduced_chi2 = chi2 / dof

% printf("mean: %f\n", mu);
% printf("variance: %f\n", sigma_sq);

% figure();
% hold on;
% r = r.*r;
% [nn, xx] = hist(r, 40, 1.0);
% hist(r, 40, 1.0)
% % x = 0.0:1e-30:max(r);
% x = 0.0:0.1:max(r);
% % dof = N - P;
% dof = 2;
% plot(x, chi2pdf(x, dof));
% ginput();
