% Task 23 in assignment 2

clear;
% State space model
A = [
    -0.3220     0.0640      0.0364      -0.9917     0.0003      0.0008  ;
    0           0           1           -0.0037     0           0       ;
    -30.6492    0           -3.6784     0.6646      -0.7333     0.1315  ;
    8.5396      0           -0.0254     -0.4764     -0.0319     -0.0620 ;
    0           0           0           0           -20.2       0       ;
    0           0           0           0           0           -20.2 
    ];

B = [
    0           0           ;
    0           0           ;
    0           0           ;
    0           0           ;
    20.2        0           ;
    0           20.2        
    ];

C = [
    0           0           0           57.2958     0           0       ;
    0           0           57.2958     0           0           0       ;
    57.2958     0           0           0           0           0       ;
    0           57.2958     0           0           0           0
    ];

D = zeros(4, 2);


% controller tuning
k_pp = -3;
k_ip = 0.0;
k_dp = 2.1562;


k_iX = 0.3438; % intergrator for chi controller
k_pX = 4.173; % proportional for chi controller

% constants
a_p1 = 3.6784;
a_p2 = -0.7333;
g = 9.81;
V_g = 552/3.6;

T = 300; % simulation time
step = 0.1; %timeseries resolution

simin.time = [0:step:T]';
simin.signals.values = ones((T/step)+1, 1);
simin.signals.dimensions = 1;
set_param('auto_pilot', 'StopTime', int2str(T));
sim('auto_pilot');

figure()
hold on;
plot(simout);
hold on;
plot(simin.time, simin.signals.values);
legend('simulation', 'reference');

simin.time = [0:step:T]';
simin.signals.values = [0 ; ones(1000, 1) ; zeros(1000, 1) ; -2 .* ones(1000, 1) ];
simin.signals.dimensions = 1;
set_param('auto_pilot', 'StopTime', int2str(T));
sim('auto_pilot');

figure();
hold on;
plot(simout);
hold on;
plot(simin.time, simin.signals.values);
legend('simulation', 'reference');
