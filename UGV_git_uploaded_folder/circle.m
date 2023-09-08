function [points] = circle(c,r,N)

th = linspace(0,2*pi,N);
points = [r*cos(th)-c(1);
          r*sin(th)-c(2);
          zeros(1,N)];
end