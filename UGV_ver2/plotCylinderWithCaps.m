function [h1, h2, h3] = plotCylinderWithCaps(r,cnt,height,nSides,color,a,b)
[X,Y,Z] = cylinder(r,nSides);
X = X + cnt(1); 
Y = Y + cnt(2); 
Z = Z * height; 
h1 = surf(X,Y,Z,'facecolor',color,'LineStyle','none');
h2 = fill3(X(1,:),Y(1,:),Z(1,:),color);
h3 = fill3(X(2,:),Y(2,:),Z(2,:),color);
h1.FaceAlpha = a;
h2.FaceAlpha = a;
h3.FaceAlpha = a;
h1.EdgeAlpha = b;
h2.EdgeAlpha = b;
h3.EdgeAlpha = b;
end  %only needed if this is within a script