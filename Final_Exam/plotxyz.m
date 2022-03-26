function [t] = plotxyz(theta, time, phase)
    t = linspace(0, time, length(theta(:,1)));
     T_6T = [    1       0       0       -0.1;
            0       1       0       0;
            0       0       1       0.13625;
            0       0       0       1];
     x = []; y = []; z = [];
    for i=1:length(theta(:,1))
        T_actual = FKPuma(theta(i,:));
%         T_tool = T_actual*T_6T;
%         T_tool = T_actual;
        T_tool = T_actual;
        T_tool(1,4) = T_tool(1,4) + T_6T(1,4);
        T_tool(2,4) = T_tool(2,4) + T_6T(2,4);
        T_tool(3,4) = T_tool(3,4) + T_6T(3,4);
        [R_tool, P_tool] = tr2rt(T_tool);
        x = [x, P_tool(1)];
        y = [y, P_tool(2)];
        z = [z, P_tool(3)];
    end

    figure
    hold on
    plot(t, x);
    plot(t, y);
    plot(t, z);
    hold off
    legend('x','y','z')
    xlabel('Time(s)')
    ylabel('x, y, z (m)')
    title(sprintf('X,Y, Z Positions of Tool Tip of Phase: %d', phase) )
end