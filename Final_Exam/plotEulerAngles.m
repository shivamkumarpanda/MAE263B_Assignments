function [t] = plotEulerAngles(O1,O2, time)
    t = linspace(0, time, 50);
    if O1 == 1 && O2 ==2
        alpha = (pi/50)*ones(1, 50);
        beta = 0;
        gamma = ((pi/2)/50)*ones(1, 50);
    end
    if O1 ==2 && O2 == 3
        alpha = ((-pi/2)/50)*ones(1, 50);
        beta = 0;
        gamma = ((-pi/2)/50)*ones(1, 50);
    end
    
    figure
    hold on
    plot(t, alpha);
    plot(t, beta);
    plot(t, gamma);
    hold off
    legend('$\alpha$','$\beta$','$\gamma$', 'interpreter', 'latex')
    xlabel('Time(s)')
    ylabel('Euler Angles (rad)')
end