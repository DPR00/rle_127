%% Variables globales
global A B C Q R K P T;

%% Generación de ZPM
% time (According to Fig. 4.40)
T = linspace(0,7,1400);
dt = T(2)-T(1); % Sample time
% ZMP en x (Based on Fig 4.40) 
Zmpx = 0*(T<2.5) + 0.3*(2.5<=T & T<3.25) + ...
        0.6*(3.25<=T & T<4.2) + 0.85*(4.2<=T);
% ZMP en y (Based on Fig 4.40) 
Zmpy = 0*(T<1.8) + 0.1*(1.8<=T & T<2.7)- 0.1*(2.7<=T & T<3.3)...
         + 0.1*(3.3<=T & T<4.2) - 0.1*(4.2<=T & T<5)+ 0*(5<=T);

figure(1),
plot(T,Zmpx,'b-', 'LineWidth', 1.5), xlabel("t(s)"), ylabel("x[m]"), title('ZMP en x')
figure(2),
plot(T,Zmpy,'b-', 'LineWidth', 1.5), xlabel("t(s)"), ylabel("y[m]"), title('ZMP en y')

%%  Preview Control System
zc = 0.3; % Length of a Solo's leg
g = 9.81; % gravity

% Kajita's Book: Introduction to Humanoid Robotics
A = [1, dt, (dt^2)/2;
     0,  1,       dt;
     0,  0,        1];
B = [(dt^3)/6; (dt^2)/2; dt];
C = [1 0 -zc/g]; 

% QR parameters
% dlqr assumed C = 1, but c'Qc must have a size of 3x3 (Eq. 4.76)
Q = 1; 
Q_ = C'*Q*C; % Then, dqlr is modified.
R = 1e-6; 

% Solution of eq 4.73. We obtain K and P for future calculations
[K, P, CLP] = dlqr(A,B,Q_,R);

%% Solution en X
[pk, xk] = solution(Zmpx, 200); % N = 200

figure(3),
plot(T, Zmpx, 'b:'), hold on
plot(T, pk, 'r-'), hold on
plot(T, xk(1,:), 'g-', 'LineWidth', 2)
legend('zmp ref', 'zmp', 'x')

%% Solution en y
[pk, yk] = solution(Zmpy, 200); % N = 200

figure(4),
plot(T, Zmpy, 'b:'), hold on
plot(T, pk, 'r-'), hold on
plot(T, yk(1,:), 'g-', 'LineWidth', 2)
legend('zmp ref', 'zmp', 'y')