samples = ceil(2/safety_target*(log(10^16)+15));
rng(2);
data = mvnrnd(G_mean, G_cov, samples)';


start_time = tic;
cvx_solver mosek;
cvx_begin quiet

    variable Ex_dep_s(size(Ad, 1)*time_horizon, 1);
    variable Ex_dep_samps(size(Ad, 1)*time_horizon, samples);
    variable U(size(Bd, 2)*time_horizon); 
    
    minimize (U'*U)
    subject to
    
        Ex_dep_s == Ad_concat * x_0 + Bd_concat * U;
        for n_i = 1:samples
            Ex_dep_samps(:, n_i) == Ex_dep_s + Cd_concat * data(:, n_i);
            G * Ex_dep_samps(:, n_i) <= h;
        end
        input_space_A * U <= input_space_b;
        
cvx_end


if strcmpi(cvx_status, 'Failed') || strcmpi(cvx_status, 'Infeasible')
    return
end

fprintf('Scenario \t %i', samples);
fprintf('\t %f', cvx_optval);
fprintf('\t %f', toc(start_time));
fprintf('\t %s', cvx_status);

p = verify(1e5, Ex_dep, Cd_concat, G, h, G_mean, G_cov);
fprintf('\t %f \n', p)