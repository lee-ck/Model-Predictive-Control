% 
%function [handle] = plot_cylinder(H,height,radius,color,alpha,n,linestyle)
%
%handle - Vector of handles for the plots
%
%H - homogeneous transformation
%height - Heighr
%radius - Radius
%color - Color [r g b] in [0, 1]
%alpha - Transparence [0, 1]
%n - Number of side faces
%linestyle - LineStyle
%Note: hold should be on. Use hold on before calling this function

%Written by: Adriano Rezende

function [handle] = plot_cylinder(H,height,radius,color,alpha,n,linestyle)

    theta = linspace(0,2*pi,n);
    
    tb = [radius*cos(theta); radius*sin(theta); 0*theta-height/2; 0*theta+1];
    tt = [radius*cos(theta); radius*sin(theta); 0*theta+height/2; 0*theta+1];
    
    tb = H*tb;
    tt = H*tt;
    
    handle(1) = fill3(tb(1,:),tb(2,:),tb(3,:),'w','FaceColor',color,'FaceAlpha',alpha,'LineStyle',linestyle);
    handle(2) = fill3(tt(1,:),tt(2,:),tt(3,:),'w','FaceColor',color,'FaceAlpha',alpha,'LineStyle',linestyle);
    
    for k = 1:1:(n-1)
        handle(k+2) = fill3([tb(1,k) tb(1,k+1) tt(1,k+1) tt(1,k)],[tb(2,k) tb(2,k+1) tt(2,k+1) tt(2,k)],[tb(3,k) tb(3,k+1) tt(3,k+1) tt(3,k)],'w','FaceColor',color,'FaceAlpha',alpha,'LineStyle',linestyle);
    end

end
