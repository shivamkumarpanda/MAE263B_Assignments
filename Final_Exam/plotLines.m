function [lx, ly] = plotLines(Lxy, n)
    xxa=Lxy(:,1);
    yya=Lxy(:,2);
    lx = [];
    ly = [];
    for i=1:length(xxa)-1
        xxa_intermediate = linspace(xxa(i), xxa(i+1), n);
        yya_intermediate = linspace(yya(i), yya(i+1), n);
        lx = [lx, xxa_intermediate];
        ly = [ly, yya_intermediate];
    end
end