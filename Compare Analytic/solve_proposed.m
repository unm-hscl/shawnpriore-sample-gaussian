lam_min = sqrt(5*(samples+1))/(sqrt(3*samples)-sqrt(5));
pow_func = @(x) 4/9 * (sqrt(samples+1)+x).^2./(x.^2*samples+(sqrt(samples+1)+x).^2);
[pow_func_m, pow_func_c] = function_affine(0, 1e-3, 250, lam_min, pow_func, 1e-4, lam_min);

E_w = mean(data, 2);
Cov_w = zeros(size(G_mean,1));

for cov_i = 1:samples
    Cov_w = Cov_w + ((data(:,cov_i) - E_w) * (data(:,cov_i) - E_w)')./samples;
end

std_est = sqrt(diag(G * Cd_concat * Cov_w * Cd_concat' * G'));

start_time = tic;
cvx_begin quiet

    variable U(size(Bd, 2)*time_horizon); 
    variable lambda(n_lin_state, 1);
    variable lambda_prob(n_lin_state, 1);
    
    minimize (U'*U)
    subject to
    
        input_space_A * U <= input_space_b;
        G * (x_mean_no_input + Bd_concat * U + Cd_concat * E_w) + lambda .* std_est <= h;
        lambda >= lam_min;
        lambda_prob >= 0;
        for n_i = 1:(n_lin_state)
                lambda_prob(n_i) >= pow_func_m .* lambda(n_i) + pow_func_c;
        end
        sum(lambda_prob) <= safety_target;
cvx_end

time_proposed = toc(start_time);
Ex_dep = x_mean_no_input + Bd_concat * U;
p = verify(1e5, Ex_dep, Cd_concat, G, h, G_mean, G_cov);

fprintf('Proposed Method \n');
fprintf('%s ', cvx_status);
fprintf('with %i samples \n', samples);
fprintf('Optimal Value: %e \n', cvx_optval);
fprintf('Time to Solve: %f \n', time_proposed);
fprintf('Empirical Constraint Satisfaction: %f \n\n', p)
