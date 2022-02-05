close all;
clear all;
clc;

%% Forward Kinematics


syms alpha0 a0 d1 t1 alpha1 a1 d2 t2 alpha2 a2 d3 t3 alpha3 a3 d4 t4 alpha4 a4 d5 t5 x y z

DH = [0         0       0.4      0; % alpha, a, d, theta - Symbolic
      0         0.325      0       pi/2;
      0         0.225      0       0;
      0         0       -0.29      0];

T_01 = transformationMatrix(DH(1,:));
T_12 = transformationMatrix(DH(2,:));
T_23 = transformationMatrix(DH(3,:));
T_34 = transformationMatrix(DH(4,:));

T_04 = T_01*T_12*T_23*T_34;
%% List all points for different trajectories



% Trajectory 1 - From the reference configuration of the arm to the feeder
% T_04 to T_0A 

T_0A = [    0           -1          0           0.325;
            1           0           0           0.225;
            0           0           1           0.100;
            0           0           0           1];

T_0A1 =[    0           -1          0           0.325;
            1           0           0           0.225;
            0           0           1           0.110;
            0           0           0           1];

% Trajectory 2 - From the feeder to the first corner 
% T_0A to T_0A1 to T_0B1 to T_0B
% Trajectory 3 - From the first corner to the feeder

T_0B = [    -1          0           0           0;
            0           -1          0           0.18;
            0           0           1           0.1;
            0           0           0           1];

T_0B1 = [   -1          0           0           0;
            0           -1          0           0.18;
            0           0           1           0.11;
            0           0           0           1];

% Trajectory 4 - From the feeder to the second corner 
% Trajectory 5 - From the second corner to the feeder  

T_0C =  [   0           -1          0           -0.09;
            1           0           0           0.18;
            0           0           1           0.100;
            0           0           0           1];

T_0C1 =  [  0           -1          0           -0.09;
            1           0           0           0.18;
            0           0           1           0.110;
            0           0           0           1];

T_0D = [    1           0           0           -0.09;
            0           1           0           0.27;
            0           0           1           0.1;
            0           0           0           1];

T_0D1 = [   1           0           0           -0.09;
            0           1           0           0.27;
            0           0           1           0.11;
            0           0           0           1];

T_0E =  [   0           1           0           0;
            -1          0           0           0.27;
            0           0           1           0.100;
            0           0           0           1];

T_0E1 =  [  0           1           0           0;
            -1          0           0           0.27;
            0           0           1           0.110;
            0           0           0           1];
%%
% Maximum velocity for each joint

vmax = [(420/180)*pi; (720/180)*pi; (3000/180)*pi; 1.1  ];

%% Define robot


L(1) = Link('revolute','d',0.4,'a',0,'alpha',0,'modified');
L(2) = Link('revolute','d',0,'a',0.325,'alpha',0,'modified');
L(3) = Link('revolute','d',0,'a',0.225,'alpha',0,'modified');
L(4) = Link('prismatic','a',0,'alpha',pi,'modified');
% tool = transl(0, 0, -0.01);
L(4).qlim = [ 0.17  0.32];
% SCARA = SerialLink([L(1),L(2),L(3),L(4)], 'tool', tool);
SCARA = SerialLink([L(1),L(2),L(3),L(4)]);
SCARA.name = 'SCARA';
figure
SCARA.plot([0 pi/2 0 0.01])

%% Trajectory 1

thetaI = inverseKinematicsScara(T_04, 0.4, 0.325, 0.225);
thetaA = inverseKinematicsScara(T_0A, 0.4, 0.325, 0.225);

[t, theta] = totalTrajectory(thetaI , thetaI, thetaA, thetaA, vmax);

figure
plot(t, theta(1:3, :))
title('Trajectory 1')
xlabel('time(s)')
ylabel('radians')
legend('\theta_1', '\theta_2', '\theta_3')

figure
plot(t, theta(4, :))
title('Trajectory 1')
xlabel('time(s)')
ylabel('d_4 (m)')



%% Trajectory 2



thetaI = inverseKinematicsScara(T_04, 0.4, 0.325, 0.225);
thetaA = inverseKinematicsScara(T_0A, 0.4, 0.325, 0.225);
thetaA1 = inverseKinematicsScara(T_0A1, 0.4, 0.325, 0.225);
thetaB1 = inverseKinematicsScara(T_0B1, 0.4, 0.325, 0.225);
thetaB = inverseKinematicsScara(T_0B, 0.4, 0.325, 0.225);

[t, theta] = totalTrajectory(thetaA , thetaA1, thetaB1, thetaB, vmax);


figure
plot(t, theta(1:3, :))
title('Trajectory 2')
xlabel('time(s)')
ylabel('radians')
legend('\theta_1', '\theta_2', '\theta_3')

figure
plot(t, theta(4, :))
title('Trajectory 2')
xlabel('time(s)')
ylabel('d_4 (m)')

theta_d = [];
theta_dd = [];

for i=1:4
    theta_d(i, :) = diff(theta(i,1:end))./diff(t);
end

for i=1:4
    theta_dd(i, :) = diff(theta_d(i,1:end))./diff(t(2:end));
end

figure
plot(t(2:end),theta_d)
title('Velocity 2')
xlabel('time(s)')
legend('$\dot{\theta}_1$', '$\dot{\theta}_2$', '$\dot{\theta}_3$', '$\dot{d}_4$', 'Interpreter','latex')

figure
plot(t(3:end), theta_dd)
title('Acceleration 2')
xlabel('time(s)')
legend('$\ddot{\theta}_1$', '$\ddot{\theta}_2$', '$\ddot{\theta}_3$', '$\ddot{d}_4$', 'Interpreter','latex')

%%

[numRows,numCols] = size(theta);
figure
SCARA.plot([0 pi/2 0 0.01])
title('Trajectory 2')
hold on
for i = 1:numCols
    SCARA.plot([theta(1,i) theta(2,i) theta(3,i) -theta(4,i)]);
    FK = SCARA.fkine([theta(1,i) theta(2,i) theta(3,i) -theta(4,i)]);
    [RT, FT] = tr2rt(FK);
    plot3(FT(1),FT(2),FT(3),'r*')    
end

%% Trajectory 3

[t, theta] = totalTrajectory(thetaB , thetaB1, thetaA1, thetaA, vmax);

figure
plot(t, theta(1:3, :))
title('Trajectory 3')
xlabel('time(s)')
ylabel('radians')
legend('\theta_1', '\theta_2', '\theta_3')

figure
plot(t, theta(4, :))
title('Trajectory 3')
xlabel('time(s)')
ylabel('d_4 (m)')


theta_d = [];
theta_dd = [];

for i=1:4
    theta_d(i, :) = diff(theta(i,1:end))./diff(t);
end

for i=1:4
    theta_dd(i, :) = diff(theta_d(i,1:end))./diff(t(2:end));
end

figure
plot(t(2:end),theta_d)
title('Velocity 3')
xlabel('time(s)')
legend('$\dot{\theta}_1$', '$\dot{\theta}_2$', '$\dot{\theta}_3$', '$\dot{d}_4$', 'Interpreter','latex')

figure
plot(t(3:end), theta_dd)
title('Acceleration 3')
xlabel('time(s)')
legend('$\ddot{\theta}_1$', '$\ddot{\theta}_2$', '$\ddot{\theta}_3$', '$\ddot{d}_4$', 'Interpreter','latex')

%%

[numRows,numCols] = size(theta);
figure
SCARA.plot([0 pi/2 0 0.01])
title('Trajectory 3')
hold on
for i = 1:numCols
    SCARA.plot([theta(1,i) theta(2,i) theta(3,i) -theta(4,i)]);
    FK = SCARA.fkine([theta(1,i) theta(2,i) theta(3,i) -theta(4,i)]);
    [RT, FT] = tr2rt(FK);
    plot3(FT(1),FT(2),FT(3),'r*')    
end
%% Trajectory 4



thetaC1 = inverseKinematicsScara(T_0C1, 0.4, 0.325, 0.225);
thetaC = inverseKinematicsScara(T_0C, 0.4, 0.325, 0.225);


[t, theta] = totalTrajectory(thetaA , thetaA1, thetaC1, thetaC, vmax);

figure
plot(t, theta(1:3, :))
title('Trajectory 4')
xlabel('time(s)')
ylabel('radians')
legend('\theta_1', '\theta_2', '\theta_3')

figure
plot(t, theta(4, :))
title('Trajectory 4')
title('Trajectory 4')
xlabel('time(s)')
ylabel('d_4 (m)')


theta_d = [];
theta_dd = [];

for i=1:4
    theta_d(i, :) = diff(theta(i,1:end))./diff(t);
end

for i=1:4
    theta_dd(i, :) = diff(theta_d(i,1:end))./diff(t(2:end));
end

figure
plot(t(2:end),theta_d)
title('Velocity 4')
xlabel('time(s)')
legend('$\dot{\theta}_1$', '$\dot{\theta}_2$', '$\dot{\theta}_3$', '$\dot{d}_4$', 'Interpreter','latex')

figure
plot(t(3:end), theta_dd)
title('Acceleration 4')
xlabel('time(s)')
legend('$\ddot{\theta}_1$', '$\ddot{\theta}_2$', '$\ddot{\theta}_3$', '$\ddot{d}_4$', 'Interpreter','latex')

%%


[numRows,numCols] = size(theta);
figure
SCARA.plot([0 pi/2 0 0.01])
hold on
for i = 1:numCols
    SCARA.plot([theta(1,i) theta(2,i) theta(3,i) -theta(4,i)]);
    FK = SCARA.fkine([theta(1,i) theta(2,i) theta(3,i) -theta(4,i)]);
    [RT, FT] = tr2rt(FK);
    plot3(FT(1),FT(2),FT(3),'r*')    
end

%% Trajectory 5



[t, theta] = totalTrajectory(thetaC , thetaC1, thetaA1, thetaA, vmax);

figure
plot(t, theta(1:3, :))
title('Trajectory 5')
xlabel('time(s)')
ylabel('radians')
legend('\theta_1', '\theta_2', '\theta_3')

figure
plot(t, theta(4, :))
title('Trajectory 5')
xlabel('time(s)')
ylabel('d_4 (m)')


theta_d = [];
theta_dd = [];

for i=1:4
    theta_d(i, :) = diff(theta(i,1:end))./diff(t);
end

for i=1:4
    theta_dd(i, :) = diff(theta_d(i,1:end))./diff(t(2:end));
end

figure
plot(t(2:end),theta_d)
title('Velocity 5')
xlabel('time(s)')
legend('$\dot{\theta}_1$', '$\dot{\theta}_2$', '$\dot{\theta}_3$', '$\dot{d}_4$', 'Interpreter','latex')

figure
plot(t(3:end), theta_dd)
title('Acceleration 5')
xlabel('time(s)')
legend('$\ddot{\theta}_1$', '$\ddot{\theta}_2$', '$\ddot{\theta}_3$', '$\ddot{d}_4$', 'Interpreter','latex')

%%


[numRows,numCols] = size(theta);
figure
SCARA.plot([0 pi/2 0 0.01])
title('Trajectory 5')
hold on
for i = 1:numCols
    SCARA.plot([theta(1,i) theta(2,i) theta(3,i) -theta(4,i)]);
    FK = SCARA.fkine([theta(1,i) theta(2,i) theta(3,i) -theta(4,i)]);
    [RT, FT] = tr2rt(FK);
    plot3(FT(1),FT(2),FT(3),'r*')    
end

%% Trajectory 6



thetaD1 = inverseKinematicsScara(T_0D1, 0.4, 0.325, 0.225);
thetaD = inverseKinematicsScara(T_0D, 0.4, 0.325, 0.225);


[t, theta] = totalTrajectory(thetaA , thetaA1, thetaD1, thetaD, vmax);

figure
plot(t, theta(1:3, :))
title('Trajectory 6')
xlabel('time(s)')
ylabel('radians')
legend('\theta_1', '\theta_2', '\theta_3')

figure
plot(t, theta(4, :))
title('Trajectory 6')
xlabel('time(s)')
ylabel('d_4 (m)')


theta_d = [];
theta_dd = [];

for i=1:4
    theta_d(i, :) = diff(theta(i,1:end))./diff(t);
end

for i=1:4
    theta_dd(i, :) = diff(theta_d(i,1:end))./diff(t(2:end));
end

figure
plot(t(2:end),theta_d)
title('Velocity 6')
xlabel('time(s)')
legend('$\dot{\theta}_1$', '$\dot{\theta}_2$', '$\dot{\theta}_3$', '$\dot{d}_4$', 'Interpreter','latex')

figure
plot(t(3:end), theta_dd)
title('Acceleration 6')
xlabel('time(s)')
legend('$\ddot{\theta}_1$', '$\ddot{\theta}_2$', '$\ddot{\theta}_3$', '$\ddot{d}_4$', 'Interpreter','latex')

%%


[numRows,numCols] = size(theta);
figure
SCARA.plot([0 pi/2 0 0.01])
title('Trajectory 6')
hold on
for i = 1:numCols
    SCARA.plot([theta(1,i) theta(2,i) theta(3,i) -theta(4,i)]);
    FK = SCARA.fkine([theta(1,i) theta(2,i) theta(3,i) -theta(4,i)]);
    [RT, FT] = tr2rt(FK);
    plot3(FT(1),FT(2),FT(3),'r*')    
end


%% Trajectory 7



[t, theta] = totalTrajectory(thetaD , thetaD1, thetaA1, thetaA, vmax);

figure
plot(t, theta(1:3, :))
title('Trajectory 7')
xlabel('time(s)')
ylabel('radians')
legend('\theta_1', '\theta_2', '\theta_3')

figure
plot(t, theta(4, :))
title('Trajectory 7')
xlabel('time(s)')
ylabel('d_4 (m)')


theta_d = [];
theta_dd = [];

for i=1:4
    theta_d(i, :) = diff(theta(i,1:end))./diff(t);
end

for i=1:4
    theta_dd(i, :) = diff(theta_d(i,1:end))./diff(t(2:end));
end

figure
plot(t(2:end),theta_d)
title('Velocity 7')
xlabel('time(s)')
legend('$\dot{\theta}_1$', '$\dot{\theta}_2$', '$\dot{\theta}_3$', '$\dot{d}_4$', 'Interpreter','latex')

figure
plot(t(3:end), theta_dd)
title('Acceleration 7')
xlabel('time(s)')
legend('$\ddot{\theta}_1$', '$\ddot{\theta}_2$', '$\ddot{\theta}_3$', '$\ddot{d}_4$', 'Interpreter','latex')

%%

[numRows,numCols] = size(theta);
figure
SCARA.plot([0 pi/2 0 0.01])
title('Trajectory 7')
hold on
for i = 1:numCols
    SCARA.plot([theta(1,i) theta(2,i) theta(3,i) -theta(4,i)]);
    FK = SCARA.fkine([theta(1,i) theta(2,i) theta(3,i) -theta(4,i)]);
    [RT, FT] = tr2rt(FK);
    plot3(FT(1),FT(2),FT(3),'r*')    
end
%% Trajectory 8


thetaE1 = inverseKinematicsScara(T_0E1, 0.4, 0.325, 0.225);
thetaE = inverseKinematicsScara(T_0E, 0.4, 0.325, 0.225);


[t, theta] = totalTrajectory(thetaA , thetaA1, thetaE1, thetaE, vmax);

figure
plot(t, theta(1:3, :))
title('Trajectory 8')
xlabel('time(s)')
ylabel('radians')
legend('\theta_1', '\theta_2', '\theta_3')

figure
plot(t, theta(4, :))
title('Trajectory 8')
xlabel('time(s)')
ylabel('d_4 (m)')


theta_d = [];
theta_dd = [];

for i=1:4
    theta_d(i, :) = diff(theta(i,1:end))./diff(t);
end

for i=1:4
    theta_dd(i, :) = diff(theta_d(i,1:end))./diff(t(2:end));
end

figure
plot(t(2:end),theta_d)
title('Velocity 8')
xlabel('time(s)')
legend('$\dot{\theta}_1$', '$\dot{\theta}_2$', '$\dot{\theta}_3$', '$\dot{d}_4$', 'Interpreter','latex')

figure
plot(t(3:end), theta_dd)
title('Acceleration 8')
xlabel('time(s)')
legend('$\ddot{\theta}_1$', '$\ddot{\theta}_2$', '$\ddot{\theta}_3$', '$\ddot{d}_4$', 'Interpreter','latex')

%%


[numRows,numCols] = size(theta);
figure
SCARA.plot([0 pi/2 0 0.01])
title('Trajectory 8')
hold on
for i = 1:numCols
    SCARA.plot([theta(1,i) theta(2,i) theta(3,i) -theta(4,i)]);
    FK = SCARA.fkine([theta(1,i) theta(2,i) theta(3,i) -theta(4,i)]);
    [RT, FT] = tr2rt(FK);
    plot3(FT(1),FT(2),FT(3),'r*')    
end
%% Trajectory 9


[t, theta] = totalTrajectory(thetaE , thetaE1, thetaA1, thetaA, vmax);

figure
plot(t, theta(1:3, :))
title('Trajectory 9')
xlabel('time(s)')
ylabel('radians')
legend('\theta_1', '\theta_2', '\theta_3')

figure
plot(t, theta(4, :))
title('Trajectory 10')
xlabel('time(s)')
ylabel('d_4 (m)')


theta_d = [];
theta_dd = [];

for i=1:4
    theta_d(i, :) = diff(theta(i,1:end))./diff(t);
end

for i=1:4
    theta_dd(i, :) = diff(theta_d(i,1:end))./diff(t(2:end));
end

figure
plot(t(2:end),theta_d)
title('Velocity 9')
xlabel('time(s)')
legend('$\dot{\theta}_1$', '$\dot{\theta}_2$', '$\dot{\theta}_3$', '$\dot{d}_4$', 'Interpreter','latex')

figure
plot(t(3:end), theta_dd)
title('Acceleration 9')
xlabel('time(s)')
legend('$\ddot{\theta}_1$', '$\ddot{\theta}_2$', '$\ddot{\theta}_3$', '$\ddot{d}_4$', 'Interpreter','latex')

%%

[numRows,numCols] = size(theta);
figure
SCARA.plot([0 pi/2 0 0.01])
title('Trajectory 9')
hold on
for i = 1:numCols
    SCARA.plot([theta(1,i) theta(2,i) theta(3,i) -theta(4,i)]);
    FK = SCARA.fkine([theta(1,i) theta(2,i) theta(3,i) -theta(4,i)]);
    [RT, FT] = tr2rt(FK);
    plot3(FT(1),FT(2),FT(3),'r*')    
end