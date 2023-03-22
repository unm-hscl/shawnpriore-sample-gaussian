function p = verify(N, Ex_dep, C_concat, G, h, G_mean, G_cov)
    tester = zeros(N,1);
    rng(1);
    for  sample = 1:N
        x = Ex_dep + C_concat * mvnrnd(G_mean, G_cov)';
        tester(sample) = all(G*x <= h);
    end
    p = sum(tester)/N;
end