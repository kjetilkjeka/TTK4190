% run file for task 3.1

% general helper stuff
Tq = @(q)   [0 0 0 1 0 0 0;
             0 0 0 0 1 0 0;
             0 0 0 0 0 1 0;
             0 0 0 0 0 0 1]* quatern(q) *   [0 0 0;
                                             0 0 0;
                                             0 0 0;
                                             1 0 0;
                                             0 1 0;
                                             0 0 1];


% satelite constants
m = 80;
r = 1.2;
Ig = m*r^2*eye(3);

% quaternion normalization factor
gamma = 100;

% satelite dynamics + normalization of quaternions
            
q_dot = @(q, omega) Tq(q) * omega + gamma/2*(1-q'*q)*q;
omega_dot = @(omega, tau) inv(Ig)*(tau + cross( (Ig * omega), omega));


% controller constants
rad = @(degree) degree/180*pi;

q_d = euler2q(rad(10), rad(20), rad(-10));
Kp = [1000 0 0; 0 1000 0; 0 0 1000];
Kd = [310 0 0; 0 310 0; 0 0 310];


% controller dynamics

u = @(q_d, q, omega) Kp*Tq(q)'*(q_d - q) - Kd*omega;

% simulation, euler method
h = 0.005;
T = 60;
t = [0:h:T];
N = T/h;
theta(1:3, 1) = [0; 0; 0];
q(1:4, 1) = euler2q(0, 0, 0);
omega(1:3, 1) = [0; 0; 0];
tau(1:3, 1) = [0; 0; 0];


for i = 1:N
    q(1:4, i+1) = euler2( q_dot( q(1:4, i), omega(1:3, i) ), q(1:4, i), h);
    omega(1:3, i+1) = euler2(omega_dot(omega(1:3, i), tau(1:3, i)), omega(1:3, i), h);
    tau(1:3, i+1) = u(q_d, q(1:4, i), omega(1:3, i) );
    
    theta(1:3, i+1) = q2euler(q(1:4, i+1));
end



% ok now the tracking error of 3.1 and 2.1 should be plotted together
for i = 1:(N+1)
    error_q(1:3, i) = q2euler(q_d) - theta(1:3, i);
end
run('../task_2_1/run_task_2_1.m');
delete(findall(0,'Type','figure')); % kill figures plotted fomr 2.1
for i = 1:(N+1)
    error_e(1:3, i) = theta_d - theta(1:3, i);
end


% do plotting
hold on;
figure(1)
plot(t, theta(1, 1:N+1));
figure(2);
plot(t, theta(2, 1:N+1));
figure(3);
plot(t, theta(3, 1:N+1));

figure(4);
plot(t, error_q(1, 1:N+1), t, error_e(1, 1:N+1));
figure(5);
plot(t, error_q(2, 1:N+1), t, error_e(2, 1:N+1));
figure(6);
plot(t, error_q(3, 1:N+1), t, error_e(3, 1:N+1));

