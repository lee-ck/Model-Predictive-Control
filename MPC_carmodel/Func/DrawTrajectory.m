function DrawTrajectory(State,LineWidth,Color)
    % scatter3(State(7,1:i)',State(8,1:i)',State(9,1:i),'r'); hold on;
if Color == 1  
    plot3(State(:,1)',State(:,2)',State(:,3),'b','LineWidth',LineWidth);
elseif Color == 2
    plot3(State(:,1)',State(:,2)',State(:,3),'r','LineWidth',LineWidth);
elseif Color ==3
    plot3(State(:,1)',State(:,2)',State(:,3),'g','LineWidth',LineWidth);
elseif Color ==4
    plot3(State(:,1)',State(:,2)',State(:,3),'k','LineWidth',LineWidth);
elseif Color ==5
    plot3(State(:,1)',State(:,2)',State(:,3),'green','LineWidth',LineWidth);
end
    xlabel x;
    ylabel y;
    zlabel z;
    axis equal;
%     view([-52 30]);

end
