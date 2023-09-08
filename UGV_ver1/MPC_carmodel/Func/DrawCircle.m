function DrawCircle(r,x,y,linewidth)
th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
plot(xunit, yunit,'k','LineWidth',linewidth);
