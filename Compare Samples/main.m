% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clean env
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all;
clear;
clc;
cvx_clear;


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% system setup
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
system_setup;

rng(1);

fh = figure();
fh.WindowState = 'maximized';

colors = [224,   0,   0; % red
           30, 144  255; % dark blue
            0, 170,  85; % green
          118,   0, 168; % purple
           46,  52,  59; % grey
          236, 176,  31;  % yellow
           76, 189, 237; % light blue
          161,  19,  46  % dark red
           ] ./ 255;
       
 shapes = ['o', 's', 'd', 'v'];

       
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% system solve
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%

samples_set = [750, 1500, 3000];
for nsamps = 1:3
    samples = samples_set(nsamps);
    solve_acs;
    
    fprintf('%i', samples);
    fprintf('\t %f', cvx_optval);
    fprintf('\t %f', toc(start_time));
    fprintf('\t %s', cvx_status);
    if strcmpi(cvx_status, 'Failed') || strcmpi(cvx_status, 'Infeasible')
        return
    end
    p = verify(1e5, Ex_dep, Cd_concat, G, h, G_mean, G_cov);
    fprintf('\t %f \n', p)

    
    subplot(7,4,[5,6,9,10,13,14,17,18,21,22,25,26]);
    hold on
        plot([x_0(1);Ex_dep(1:6:end)], [x_0(2); Ex_dep(2:6:end)], '-','Color', colors(nsamps,:), 'Marker', shapes(nsamps), 'MarkerSize',15);
    hold off
    
    subplot(7,4,[7,8,11,12,15,16,19,20,23,24.27,28]);
    hold on
        plot([x_0(1); Ex_dep(1:6:end)], [x_0(3); Ex_dep(3:6:end)], '-o','Color', colors(nsamps,:), 'Marker', shapes(nsamps), 'MarkerSize',15);
    hold off
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% plot
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%




subplot(7,4,[5,6,9,10,13,14,17,18,21,22,25,26]);
hold on
patch([0; 10; 10], [0; -5; 5], 'b', 'FaceAlpha', 0.1);
patch([0; 2; 2; 0], [-1; -1; 1; 1], 'g', 'FaceAlpha', 0.1);
xlabel('x');
ylabel('y');
axis([0 12 -6 6]);
hold off

subplot(7,4,[7,8,11,12,15,16,19,20,23,24.27,28]);
hold on
patch([0; 10; 10], [0; -5; 5], 'b', 'FaceAlpha', 0.1);
patch([0; 2; 2; 0], [-1; -1; 1; 1], 'g', 'FaceAlpha', 0.1);
xlabel('x');
ylabel('z');
axis([0 12 -6 6]);
hold off
%%
subplot(7,4,[1:4]);
hold on
p0 = plot(nan, nan, '-','Color', colors(1,:), 'Marker', shapes(1), 'MarkerSize',15);
p1 = plot(nan, nan, '-','Color', colors(2,:), 'Marker', shapes(2), 'MarkerSize',15);
p2 = plot(nan, nan, '-','Color', colors(3,:), 'Marker', shapes(3), 'MarkerSize',15);
p4=patch(nan, nan, 'b', 'FaceAlpha', 0.1);
p5=patch(nan, nan, 'g', 'FaceAlpha', 0.1);
legend([p0,p1,p2,p4,p5], ["$N_s = 750$", "$N_s = 1500$", "$N_s = 3000$", 'Line of Sight', 'Target Set'], ...
    'Orientation','horizontal', ...
    'Location', 'south', ...
    'NumColumns', 6, ...
    'Interpreter', 'Latex');
axis([0 0.1 0 0.1]);
axis off
hold off
