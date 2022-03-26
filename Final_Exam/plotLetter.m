function [lx, ly] = plotLetter(Lxy, n, ax_lim )
    xxa=Lxy(:,1);
    yya=Lxy(:,2);
    distF=[0 ;sqrt(sum(diff([xxa,yya]).^2,2))];
    distFSum=cumsum(distF);
    t = linspace(min(distFSum),max(distFSum),n);
    lx=spline(distFSum,xxa,t);
    ly=spline(distFSum,yya,t);
    plot(lx,ly,'y','LineWidth',2)
    hold on
    plot(xxa,yya,'ko')
    axis(ax_lim)
    hold off
end