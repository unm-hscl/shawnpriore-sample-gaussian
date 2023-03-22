lam_min = sqrt(5/3);
pow_func = @(x) 4./(9 .*(x.^2+1));
[pow_func_m, pow_func_c] = function_affine(0, 1e-3, 250, lam_min, pow_func, 1e-4, lam_min);

std_est = sqrt(diag(G * Cd_concat * G_cov * Cd_concat' * G'));

start_time = tic;
cvx_begin quiet

    variable U(size(Bd, 2)*time_horizon); 
    variable lambda(n_lin_state, 1);
    variable lambda_prob(n_lin_state, 1);
    
    minimize (U'*U)
    subject to
    
        input_space_A * U <= input_space_b;
        G * (x_mean_no_input + Bd_concat * U + Cd_concat * G_mean') + lambda .* std_est <= h;
        lambda >= lam_min;
        lambda_prob >= 0;
        for n_i = 1:(n_lin_state)
                lambda_prob(n_i) >= pow_func_m .* lambda(n_i) + pow_func_c;
        end
        sum(lambda_prob) <= safety_target;
cvx_end

time_vp = toc(start_time);
Ex_dep_vp = x_mean_no_input + Bd_concat * U;
p = verify(1e5, Ex_dep_vp, Cd_concat, G, h, G_mean, G_cov);

fprintf('OSVPI Method \n');
fprintf('%s ', cvx_status);
fprintf('Optimal Value: %e \n', cvx_optval);
fprintf('Time to Solve: %f \n', time_vp);
fprintf('Empirical Constraint Satisfaction: %f \n\n', p)
