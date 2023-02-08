lam_min = sqrt(5*(samples+1))/(sqrt(3*samples)-sqrt(5));
pow_func = @(x) 4/9 * (sqrt(samples+1)+x).^2./(x.^2*samples+(sqrt(samples+1)+x).^2);
[pow_func_m, pow_func_c] = function_affine(0, 1e-2, 1e4, lam_min, pow_func, 1e-4, lam_min);

data = mvnrnd(G_mean, G_cov, samples)';
E_w = mean(data, 2);
Cov_w = zeros(size(G_mean,1));

for cov_i = 1:samples
    Cov_w = Cov_w + ((data(:,cov_i) - E_w) * (data(:,cov_i) - E_w)')./samples;
end

std_est = zeros(n_lin_state,1);
for n_i = 1:n_lin_state
    std_est(n_i) = sqrt( G(n_i, :) * Cd_concat * Cov_w * Cd_concat' * G(n_i, :)');
end

start_time = tic;
cvx_solver mosek;
cvx_begin quiet

    variable Ex_dep(size(Ad, 1)*time_horizon, 1);
    variable U(size(Bd, 2)*time_horizon); 
    variable lambda(n_lin_state, 1);
    variable lambda_prob(n_lin_state, 1);
    
    minimize (U'*U)
    subject to
    
        Ex_dep == Ad_concat * x_0 + Bd_concat * U;
        input_space_A * U <= input_space_b;
        G * (Ex_dep + Cd_concat * E_w) + lambda .* std_est <= h;
        lambda >= lam_min;
        lambda_prob >= 0;
        for n_i = 1:(n_lin_state)
                lambda_prob(n_i) >= pow_func_m .* lambda(n_i) + pow_func_c;
        end
        sum(lambda_prob) <= safety_target;
cvx_end