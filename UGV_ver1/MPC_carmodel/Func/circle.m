function circle(x,y,r,color)
%% Plot circle 
th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;

if color == 1
    plot(xunit, yunit,'Color',[1 0.5 0.5],'LineWidth',2);
else
    plot(xunit, yunit,'Color',[0.5 0.5 1],'LineWidth',2);
end
end