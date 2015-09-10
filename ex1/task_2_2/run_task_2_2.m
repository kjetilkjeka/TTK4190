% Run file for task 2.2

% satelite constants
m = 80;
r = 1.2;
Ig = m*r^2*eye(3);

% satelite dynamics
Trans2 = @(phi, theta, psi) [  1 sin(phi)*tan(theta) cos(phi)*tan(theta);
                               0 cos(phi) -sin(phi);
                               0 sin(phi)/cos(theta) cos(phi)/cos(theta) ];
Trans = @(Theta) Trans2(Theta(1), Theta(2), Theta(3));

            
theta_dot = @(theta, omega) Trans(theta) * omega;
omega_dot = @(omega, tau) inv(Ig)*(tau + cross( (Ig * omega), omega));

% controller constants
theta_d = [2; 0.5; 1];
Kp = [10 0 0; 0 10 0; 0 0 10];
Kd = [80 0 0; 0 80 0; 0 0 80];

% controller dynamics
u = @(theta, theta_d, omega) transpose(Trans(theta))*Kp*(theta_d - theta) - Kd*omega - cross( (Ig*omega), omega); 

% simulation, euler method
h = 0.005;
T = 60;
t = [0:h:T];
N = T/h;
theta(1:3, 1) = [0; 0; 0];
omega(1:3, 1) = [0; 0; 0];
tau(1:3, 1) = [0; 0; 0];


for i = 1:N
    theta(1:3, i+1) = euler2( theta_dot( theta(1:3, i), omega(1:3, i) ), theta(1:3, i), h);
    omega(1:3, i+1) = euler2(omega_dot(omega(1:3, i), tau(1:3, i)), omega(1:3, i), h);
    controller_output = u(theta(1:3, i), theta_d, omega(1:3, i) );
    noise = (rand(3, 1) - 0.5)*0.2;
    tau(1:3, i+1) = controller_output + noise;
end

% do plotting
figure(1);
hold on;
title('Euler angles - noise');
plot(t, theta(1, 1:N+1));
plot(t, theta(2, 1:N+1));
plot(t, theta(3, 1:N+1));
legend('phi', 'theta', 'psi');