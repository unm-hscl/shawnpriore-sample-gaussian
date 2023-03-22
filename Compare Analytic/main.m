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


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% disturbance samples
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
samples = 5000;
rng(3);
data = mvnrnd(G_mean, G_cov, samples)';

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% system solve
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
cvx_solver Gurobi_2;
%cvx_solver_settings('TimeLimit', 1800);
cvx_precision high;

solve_proposed;
solve_vp;

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% plot
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%

fh = figure();
fh.WindowState = 'maximized';

colors = [  0,   0,   0; % red
           30, 144  255; % dark blue
            0, 170,  85; % green
          118,   0, 168; % purple
          224,   0,   0; % grey
           30, 144  255; % dark blue
           77, 219, 255; % light blue
          236, 176,  31;  % yellow
          161,  19,  46  % dark red
           ] ./ 255;
       
shapes = ['^', '*', 'o'];
 
 
subplot(7,4,[1:4]);
hold on
p0 = plot(nan, nan, '-','Color', colors(1,:), 'Marker', shapes(1), 'MarkerSize',15);
p1 = plot(nan, nan, '--','Color', colors(5,:), 'Marker', shapes(2), 'MarkerSize',15);
p3 = patch(nan, nan, 'b', 'FaceAlpha', 0.05);
p4 = patch(nan, nan, 'g', 'FaceAlpha', 0.05);
p5 = plot(nan, nan, 'ks','MarkerFaceColor','k', 'MarkerSize',15);
legend([p0,p1,p5, p3,p4], ["Proposed Method", "MPC with OSVPI",  'Initial Condition','Line of Sight Cone', 'Target Set'], ...
    'Orientation','horizontal', ...
    'Location', 'south', ...
    'NumColumns', 3, ...
    'Interpreter', 'Latex');
axis([0 0.1 0 0.1]);
axis off
hold off


subplot(7,4,[5,6,9,10,13,14,17,18,21,22,25,26]);
hold on
plot([x_0(1); Ex_dep(1:6:end)], [x_0(2); Ex_dep(2:6:end)], '-','Color', colors(1,:));
plot([x_0(1); Ex_dep_vp(1:6:end)], [x_0(2); Ex_dep_vp(2:6:end)], '--','Color', colors(5,:));
plot(Ex_dep(1:6:end), Ex_dep(2:6:end),'Color', colors(1,:), 'Marker', shapes(1), 'MarkerSize', 15, 'LineStyle', 'none');
plot(Ex_dep_vp(1:6:end), Ex_dep_vp(2:6:end), 'Color', colors(5,:), 'Marker', shapes(2), 'MarkerSize', 15, 'LineStyle', 'none');
plot(x_0(1), x_0(2), 'ks','MarkerFaceColor','k', 'MarkerSize',15);
patch([0; 10; 10], [0; -5; 5], 'b', 'FaceAlpha', 0.05);
patch([0; 2; 2; 0], [-1; -1; 1; 1], 'g', 'FaceAlpha', 0.05);
xlabel('x');
ylabel('y');
axis([0 12 -6 6]);
hold off

subplot(7,4,[7,8,11,12,15,16,19,20,23,24.27,28]);
hold on
plot([x_0(1); Ex_dep(1:6:end)], [x_0(3); Ex_dep(3:6:end)], '-','Color', colors(1,:));
plot([x_0(1); Ex_dep_vp(1:6:end)], [x_0(3); Ex_dep_vp(3:6:end)], '--','Color', colors(5,:));
plot(Ex_dep(1:6:end), Ex_dep(3:6:end),'Color', colors(1,:), 'Marker', shapes(1), 'MarkerSize', 15, 'LineStyle', 'none');
plot(Ex_dep_vp(1:6:end), Ex_dep_vp(3:6:end), 'Color', colors(5,:), 'Marker', shapes(2), 'MarkerSize', 15, 'LineStyle', 'none');
plot(x_0(1), x_0(3), 'ks','MarkerFaceColor','k', 'MarkerSize',15);

patch([0; 10; 10], [0; -5; 5], 'b', 'FaceAlpha', 0.05);
patch([0; 2; 2; 0], [-1; -1; 1; 1], 'g', 'FaceAlpha', 0.05);
xlabel('x');
ylabel('z');
axis([0 12 -6 6]);
hold off


