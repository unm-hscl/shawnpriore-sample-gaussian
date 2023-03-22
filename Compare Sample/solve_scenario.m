start_time = tic;
cvx_begin quiet

    variable U(size(Bd, 2)*time_horizon); 
    
    minimize (U'*U)
    subject to
        for n_i = 1:samples
            G * (x_mean_no_input + Bd_concat * U + Cd_concat * data(:, n_i)) <= h;
        end
        input_space_A * U <= input_space_b;
        
cvx_end

time_scenario = toc(start_time);
Ex_dep_s = x_mean_no_input + Bd_concat * U;
p = verify(1e5, Ex_dep_s, Cd_concat, G, h, G_mean, G_cov);

fprintf('Scenario Method \n');
fprintf('%s ', cvx_status);
fprintf('with %i samples \n', samples);
fprintf('Optimal Value: %e \n', cvx_optval);
fprintf('Time to Solve: %f \n', time_scenario);
fprintf('Empirical Constraint Satisfaction: %f \n\n', p)