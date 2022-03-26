function [t] = plotThetas(theta, time, phase)
    
t = linspace(0, time, length(theta(:,1)));
figure
hold on
plot(t, theta(:, 1));
plot(t, theta(:, 2));
plot(t, theta(: ,3));
plot(t, theta(: ,4));
plot(t, theta(:, 5));
plot(t, theta(:, 6));
hold off
legend('$\theta_1$','$\theta_2$','$\theta_3$','$\theta_4$','$\theta_5$','$\theta_6$', 'interpreter', 'latex')
xlabel('Time(s)')
ylabel('Joint Angles (rad)')
title(sprintf('Joint Angles of Phase: %d', phase) )
fprintf('The amount of via points used in this Phase is: %d', length(theta(:,1)))
end