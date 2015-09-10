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

% do plotting
hold on;
plot(t, theta(1, 1:N+1));
figure();
plot(t, theta(2, 1:N+1));
figure();
plot(t, theta(3, 1:N+1));