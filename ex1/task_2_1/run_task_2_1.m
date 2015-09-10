% Run file for task 2.1


% satelite constants
m = 80;
r = 1.2;
Ig = m*r^2*eye(3);

% satelite dynamics
omega_dot = @(omega, tau) inv(Ig)*(tau + cross( (Ig * omega), omega));

% controller constants
omega_d = [2; 0; 1];
d = 2;
Kp = [100 0 0; 0 100 0; 0 0 100];
%Kd = @(omega)[0 0 0; 0 0 0; 0 0 0];
Kd = @(omega) - Ig + (Kp - Smtrx(Ig * omega) * (1/d));

% controller dynamics
u = @(omega_d, omega, omega_dot) Kp*(omega_d - omega) - Kd(omega)*omega_dot; 

% simulation, euler method
h = 0.01;
T = 50;
t = [0:h:T];
N = T/h;
omega(1:3, 1) = [0; 0; 0];
tau(1:3, 1) = [0; 0; 0];


for i = 1:N
    omega(1:3, i+1) = euler2(omega_dot(omega(1:3, i), tau(1:3, i)), omega(1:3, i), h);
    tau(1:3, i+1) = u(omega_d, omega(1:3, i), omega_dot(omega(1:3, i), tau(1:3, i)));
end

% do plotting
hold on;
plot(t, omega(1, 1:N+1));
figure();
plot(t, omega(2, 1:N+1));
figure();
plot(t, omega(3, 1:N+1));