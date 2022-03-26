function [T_06] = FKPuma(theta)
    
    % Numercial Expressions

        DH = [      0        0       0       theta(1);     %alpha, a, d, theta
                    -pi/2     0       0.2435      theta(2);
                    0       0.4318      -0.0934      theta(3);
                    pi/2    -0.0203      0.4331      theta(4);
                    -pi/2   0       0       theta(5);
                    pi/2    0       0       theta(6)
                ];
        
        T_01 = transformationMatrix(DH(1,:));
        T_12 = transformationMatrix(DH(2,:));
        T_23 = transformationMatrix(DH(3,:));
        T_34 = transformationMatrix(DH(4,:));
        T_45 = transformationMatrix(DH(5,:));
        T_56 = transformationMatrix(DH(6,:));
        
        T_06 = T_01*T_12*T_23*T_34*T_45*T_56;
    

end