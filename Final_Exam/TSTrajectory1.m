function [theta] = TSTrajectory1(O1, O2, P0_corner, n)

    P = P0_corner;

    if O1 ==1 && O2 ==2
         TA = [   0   1   0   P(1);
                0   0   1   P(2);
                1   0   0   P(3)
                0   0   0   1];
         TB = [   -1   0   0   P(1);
                0   1   0   P(2);
                0   0   -1   P(3)
                0   0   0   1];
    elseif O1 ==2 && O2 ==3
        TA = [   -1   0   0   P(1);
                0   1   0   P(2);
                0   0   -1   P(3)
                0   0   0   1];
        TB = [   0   0   -1   P(1);
                -1   0   0   P(2);
                0   1   0   P(3)
                0   0   0   1];
    
    elseif O1 == 3 && O2 ==1 
        TA = [   0   0   -1   P(1);
                -1   0   0   P(2);
                0   1   0   P(3)
                0   0   0   1];
        TB = [   0   1   0   P(1);
                0   0   1   P(2);
                1   0   0   P(3)
                0   0   0   1];

    end


    CA = singleAxis(TA);
    CB = singleAxis(TB);


    for i=1:6
        [S,SD,SDD] = lspb(CA(i), CB(i), n);
        CA(i,1:n) = S';
    end
    
    for i=1:n
        T_now(1:4, 1:4, i) = singletoT(CA(1:6,i));
        theta(1:6, i) =  IKPuma4(T_now(1:4, 1:4, i),  0.4318, -0.0203, 0.2435, -0.0934, 0.4331);
    end
    theta = theta';
end