function aa = fill_circle_road(x,y,r,c,a,b)
th = 0:pi/50:2*pi;
x_circle = r * cos(th) + x;
y_circle = r * sin(th) + y;
aa = fill(x_circle, y_circle, c);
aa.FaceColor = c;
aa.FaceAlpha = a;
aa.EdgeAlpha = b;
end
