%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dynamics system
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
time_horizon                 = 5;
sampling_period              = 60;                                              % sec
orbital_radius               = 42164 * 1000;                                    % m (geosyncronus)
gravitational_constant       = 6.673e-11;                                       % m^3 kg^-1 sec^-2
celestial_mass               = 5.9472e24;                                       % kg
gravitational_body           = gravitational_constant * celestial_mass;         % m^3 sec^-2
orbit_ang_vel                = sqrt(gravitational_body / orbital_radius^3);     % rad sec^-2

% Continuous-time LTI CWH unforced dynamics e^{At}
e_power_At = @(t) [ 
    4 - 3 * cos(orbit_ang_vel * t), 0, 0, (1/orbit_ang_vel) * sin(orbit_ang_vel * t), (2/orbit_ang_vel) * (1 - cos(orbit_ang_vel * t)), 0; 
    6 * (sin(orbit_ang_vel * t) - orbit_ang_vel * t), 1, 0, -(2/orbit_ang_vel) * (1 - cos(orbit_ang_vel * t)), (1/orbit_ang_vel) * (4*sin(orbit_ang_vel * t) - 3*orbit_ang_vel * t), 0; 
    0, 0, cos(orbit_ang_vel * t), 0, 0, (1/orbit_ang_vel) * sin(orbit_ang_vel * t); 
    3 * orbit_ang_vel * sin(orbit_ang_vel * t), 0, 0, cos(orbit_ang_vel * t), 2 * sin(orbit_ang_vel * t), 0; 
    -6 * orbit_ang_vel * (1 - cos(orbit_ang_vel * t)), 0, 0, -2 * sin(orbit_ang_vel * t), 4 * cos(orbit_ang_vel * t) - 3, 0;
    0, 0, -orbit_ang_vel * sin(orbit_ang_vel * t), 0, 0, cos(orbit_ang_vel * t);
    ];

% Discrete-time system is Phi(T_s) for sampling time T_s since the system is time-invariant
Ad = e_power_At(sampling_period);

% Impulse control
Bd = Ad*[zeros(3); eye(3)];

% Concat matrix maker
Ad_concat = zeros(size(Ad, 1)*time_horizon, size(Ad, 2));
Bd_concat = zeros(size(Bd, 1)*time_horizon, size(Bd, 2)*time_horizon);
Cd_concat = zeros(size(Bd, 1)*time_horizon, size(Bd, 2)*2*time_horizon);
for i = 0:(time_horizon-1)
    Ad_concat(size(Ad, 1)*i + [1:size(Ad, 1)], :) = Ad^(i+1);
end
for i = 0:(time_horizon-1)
    for j = 0:i
        Bd_concat(size(Bd, 1)*i + [1:size(Bd, 1)], size(Bd, 2)*j + [1:size(Bd, 2)]) = Ad^(i-j) * Bd;
        Cd_concat(size(Bd, 1)*i + [1:size(Bd, 1)], (2*size(Bd, 2))*j + [1:(2*size(Bd, 2))]) = Ad^(i-j) * blkdiag(Bd(1:3,:), Bd(4:6,:));
    end
end

% problem set up
% initial state
% format: x, y, z,  x., y., z.
x_0 = [11;  -4;  6; 0; 0; 0] ; % satellite A

% target set
% format: x, y, z, x., y., z.
G_k = [-1,  0,  2, 0, 0, 0;
       -1,  2,  0, 0, 0, 0;
       -1,  0, -2, 0, 0, 0;
       -1, -2,  0, 0, 0, 0;
        1,  0,  0, 0, 0, 0];
h_k  = [0;0;0;0;10];

G_N = kron(eye(6), [1;-1]);
h_N = [2; 0; ones(4,1); 0.1 * ones(6,1)];

G = blkdiag( kron(eye(time_horizon-1), G_k), G_N);
h = [kron(ones(time_horizon-1,1),h_k); h_N];
                                          
% get number of half space constraints                      
n_lin_state = size(G,1);

% Input space
u_max = 0.1;
input_space_A = kron(eye(time_horizon), [eye(3); -eye(3)]);
input_space_b = u_max * ones(time_horizon*6,1);

% safety threshold
safety_target         = 0.15;  % in target set

% disturbance param
G_mean = zeros(1,size(Cd_concat, 2));
G_cov = kron(eye(time_horizon), [1e-6 * eye(3), zeros(3); zeros(3), 5e-8 * eye(3)]);
