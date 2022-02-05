function [t, theta] = taskSpaceTrajectory(T_0A , T_0A1, T_0B1, T_0B, vmax)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here

    C_0A = singleAxis(T_0A);
    C_0A1 = singleAxis(T_0A1);
    C_0B1 = singleAxis(T_0B1);
    C_0B = singleAxis(T_0B);
    
    
    
    % Part 1 
    M1 = 10;
    dt = [];
    for i=1:6
        [S,SD,SDD] = lspb(C_0A(i), C_0A1(i), M1);
        C_0AA1(i,1:M1) = S';
    end
    
    for i=1:M1
        T_0AA1(1:4, 1:4, i) = singletoT(C_0AA1(1:6,i));
        thetaPiece1(1:4, i) =  inverseKinematicsScara2(T_0AA1(1:4, 1:4, i), 0.4, 0.325, 0.225);
    end
    
    for i=2:M1
        dt(1:4,i-1) =  (abs(thetaPiece1(1:4, i) - thetaPiece1(1:4, i-1)))./vmax;
    end
    tf1 = M1*max(dt, [], 'all');
    t1 = linspace(0,tf1,M1);
    
    % Part 2
    
    M2 = 100;
    dt = [];
    for i=1:6
        [S,SD,SDD] = lspb(C_0A1(i), C_0B1(i), M2);
        C_0A1B1(i,1:M2) = S';
    end
    
    for i=1:M2
        T_0A1B1(1:4, 1:4, i) = singletoT(C_0A1B1(1:6,i));
        thetaPiece2(1:4, i) =  inverseKinematicsScara2(T_0A1B1(1:4, 1:4, i), 0.4, 0.325, 0.225);
    end
    
    for i=2:M2
        dt(1:4,i-1) =  (abs(thetaPiece2(1:4, i) - thetaPiece2(1:4, i-1)))./vmax;
    end
    tf2 = M2*max(dt, [], 'all');
    t2 = linspace(0,tf2,M2);
    
    %Part 3 
    
    M3 = 10;
    dt = [];
    for i=1:6
        [S,SD,SDD] = lspb(C_0B1(i), C_0B(i), M3);
        C_0B1B(i,1:M3) = S';
    end
    
    for i=1:M3
        T_0B1B(1:4, 1:4, i) = singletoT(C_0B1B(1:6,i));
        thetaPiece3(1:4, i) =  inverseKinematicsScara2(T_0B1B(1:4, 1:4, i), 0.4, 0.325, 0.225);
    end
    
    for i=2:M3
        dt(1:4,i-1) =  (abs(thetaPiece3(1:4, i) - thetaPiece3(1:4, i-1)))./vmax;
    end
    tf3 = M3*max(dt, [], 'all');
    t3 = linspace(0,tf3,M3);
    
    t2 = t2 + tf1;
    t3 = t3 + tf1 + tf2; 
    t = [t1, t2, t3];
    theta = [thetaPiece1, thetaPiece2, thetaPiece3];
end