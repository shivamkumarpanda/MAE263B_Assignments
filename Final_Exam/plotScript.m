function [done] = plotScript(lx, ly, ax_lim)
    plot(lx,ly,'y','LineWidth',2)
    axis(ax_lim)
    done= 1;
end