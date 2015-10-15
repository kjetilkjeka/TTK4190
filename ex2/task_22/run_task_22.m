s = tf('s'); % everything multiplied with s will become a t.f.

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

% dynamics expressed as transfer functions
h_pitch = a_p2 / (s + a_p1); % open loop dynamics between aileron and pitch
h_pitch_damp = feedback(h_pitch, k_dp); %damping added to pitch dyn

h_p_controller = k_pp + k_ip/s; % open loop controller w/o damp dyn
h_phi_ol = h_p_controller * h_pitch * (1/s); % open loop dyn phi/aileron with pitch
h_phi_cl = feedback(h_phi_ol, 1); % closed loop phi controller

h_chi_controller = k_pX + k_iX/s; % open loop controller for chi
h_chi_ol = h_chi_controller * h_phi_cl * ((g/V_g)/s); % closed inner loop t.f. from phi_c to chi
% close the outer loop including chi controller
h_system = feedback(h_chi_ol, 1);

h_open = h_pitch * (1/s) * ((g/V_g) / s); % this is the open loop (excluding controller dynamics)

bode(h_system);
hold on;
bode(h_open);
legend('Closed loop system', 'Open loop system');