function [posError, orientError] = calculatePositionError(theta, P_required, P0_corner, O)
    T_actual = FKPuma(theta);
    T_6T = [    1       0       0       -0.1;
            0       1       0       0;
            0       0       1       0.13625;
            0       0       0       1];
    
    P = P0_corner + P_required;

    if (O==1)
        M = [   0   1   0   P(1);
                0   0   1   P(2);
                1   0   0   P(3)
                0   0   0   1];
    elseif (O==2)
        M = [   -1   0   0   P(1);
                0   1   0   P(2);
                0   0   -1   P(3)
                0   0   0   1];
    elseif (O==3)
        M = [   0   0   -1   P(1);
                -1   0   0   P(2);
                0   1   0   P(3)
                0   0   0   1];
    end

    
    
    T_required = M*(T_6T^(-1));
    
    [R_required, P_required] = tr2rt(T_required);
    [R_actual, P_actual] = tr2rt(T_actual);
    posError = sqrt(sum((P_actual - P_required) .^ 2));
    orientError = sqrt(sum((R_actual - R_required) .^ 2, 'all'));
end