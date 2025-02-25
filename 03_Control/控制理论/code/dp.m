%% Dynamic Programming
clc; clear; close all;

% Define IC
h_init = 0;     % h: height
v_init = 0;     % v: velocity

% Final state
h_final = 0;
v_final = 0;

% Boundary condition
h_min = 0;
h_max = 10;
N_h   = 5;      % 高度的离散节点数
v_min = 0;
v_max = 3;
N_v   = 3;      % 速度的离散节点数

% Create state array
Hd = h_min : (h_max - h_min)/N_h : h_max;
Vd = v_min : (v_max - v_min)/N_v : v_max;

% Input constraint, input is the system acceleration
u_min = -3; u_max = 2;

% Define cost to go matrix
J_costtogo = zeros(N_h+1, N_v+1);

% Define input acceleration matrix
Input_acc = zeros(N_h+1, N_v+1);

%%%%%%%%%%%%%%%% From 10m to 8m %%%%%%%%%%%%%%%%%%%%%%
v_avg = 0.5 * (v_final + Vd);   % Calculate average speed       
T_delta = (h_max - h_min)./(N_h * v_avg);   % Calculate travel time (Cost)
acc = (0 - Vd)./T_delta;    % Calculate acceleration
J_temp = T_delta;   % Assign delta T to cost to go
[acc_x, acc_y] = find(acc < u_min | acc > u_max);   % Find which acc is over the limit
Ind_lin_acc = sub2ind(size(acc), acc_x, acc_y); % Find linear index
J_temp(Ind_lin_acc) = inf;  % Let certain elements to infinity
J_costtogo(2, :) = J_temp;  % Save to cost to go matrix
Input_acc(2, :) = acc;  % Save to acceleration matrix

%%%%%%%%%%%%%%%% From 8m to 2m %%%%%%%%%%%%%%%%%%%%%%
for k = 3 : 1 : N_h
    
    [Vd_x, Vd_y] = meshgrid(Vd, Vd);    % Prepare the matrix
    v_avg = 0.5 * (Vd_x + Vd_y);    % Calculate average time
    T_delta = (h_max - h_min)./(N_h * v_avg);   % Calculate travel time (Cost)
    acc = (0 - Vd)./T_delta;    % Calculate acceleration
    J_temp = T_delta;   % Assign delta T to cost to go
    [acc_x, acc_y] = find(acc < u_min | acc > u_max);   % Find which acc is over the limit
    Ind_lin_acc = sub2ind(size(acc), acc_x, acc_y); % Find linear index
    J_temp(Ind_lin_acc) = inf;  % Let certain elements to infinity
    
    J_temp = J_temp + meshgrid(J_costtogo(k-1,:))';   % Add last cost to go
    [J_costtogo(k, :), l] = min(J_temp); % Save to cost to go matrix
    Ind_lin_acc = sub2ind(size(J_temp), l, 1:length(l)); % Find linear index
    Input_acc(k, :) = acc(Ind_lin_acc); % Save to acceleration matrix
    
end

%%%%%%%%%%%%%%%% From 2m to 0m %%%%%%%%%%%%%%%%%%%%%%







