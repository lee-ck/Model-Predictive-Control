function [input,sim] = Make_ref_car(Map,sim,input)
% make reference for linear path (waypoint trakcing)
flag = Map.WPT - input.x0(1:2);
[~, minidx]  = min(sum(flag.*flag,2));
input.index = [];

% Map.WPT(minidx,:)
% minidx
%% Stage reference (CTE)
slack = minidx;
for i = 1:sim.Num+1
    if slack > Map.WPT_max
        slack = Map.WPT_max;
    end
    input.y(i,1) = Map.WPT(slack,1);
    input.y(i,2) = Map.WPT(slack,2);
    slack = slack +1;
end
slack = minidx;
for i = 1:sim.Num+1
    if slack >= Map.WPT_max
        slack = Map.WPT_max-1;
        Ax = Map.WPT(slack+1,1) - Map.WPT(slack,1);
        Ay = Map.WPT(slack+1,2) - Map.WPT(slack,2);
        input.y(i,3) = (atan2(Ay,Ax));
    else
        Ax = Map.WPT(slack+1,1) - Map.WPT(slack,1);
        Ay = Map.WPT(slack+1,2) - Map.WPT(slack,2);
        input.y(i,3) = (atan2(Ay,Ax));
    end
    input.index(i,1) = slack;
    slack = slack +1;
end

% for i = 1:sim.Num
%     Ax = input.y(i+1,1)-input.y(i,1);
%     Ay = input.y(i+1,2)-input.y(i,2);
%     input.y(i,3) = (atan2(Ay,Ax));
% end
% input.y(sim.Num+1,3) = input.y(sim.Num,3);

%% Terminal reference (LOS)

Ax = Map.WPT(Map.WPT_Now,1)-input.x0(1);
Ay = Map.WPT(Map.WPT_Now,2)-input.x0(2);


input.yN(1,1) = input.y(end,1);%Map.WPT(Map.WPT_Now,1);
input.yN(1,2) = input.y(end,2);%Map.WPT(Map.WPT_Now,2);
input.yN(1,3) = input.y(end,3); %atan2(Map.WPT(Map.WPT_Now,2)-input.x0(2),Map.WPT(Map.WPT_Now,1)-input.x0(1));
input.y(end,:) =[];

end