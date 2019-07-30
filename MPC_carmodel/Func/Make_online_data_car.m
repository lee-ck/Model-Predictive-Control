function  [Map,input,sim] = Make_online_data_car(Map,input,sim)   

%%%%%%%   8 OnlineDatas %%%%%%%   
% OnlineData cylinder_x
% OnlineData cylinder_y
% OnlineData cylinder_r
% 
% OnlineData cylinder2_x
% OnlineData cylinder2_y
% OnlineData cylinder2_r
% 
% OnlineData yboudary_lower
% OnlineData yboudary_upper
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  


%Map.obstacle = check_obstacle_inrange(sim,Map.obstacle);  % future work

od = zeros(1,19);
% Cylinder
od(1) = 15;
od(2) = 9.8;
od(3) = 0.5;

od(4) = 25;
od(5) = 14.7;
od(6) = 0.5;

od(7) = 30;
od(8) = 4.2;
od(9) = 0.4;

od(10) = 15;
od(11) = -4.5;
od(12) = 0.7;

od(13) = 10;
od(14) = -5.4;
od(15) = 0.7;

% path constraints
od(16) = 0;
od(17) = 100;
od(18) = 0;
od(19) = -100;

input.od_obs = od.*ones(sim.Num+1,1); %OnlineData xd yd zd radius ;


input.od = [input.od_obs ];
    
for i = 1:sim.Num
a = Map.WPT(input.index(i,1)+1,:)-Map.WPT(input.index(i,1),:);
a1 = a(2)/a(1);
xx1 = input.y(i,1) + Map.acc_bd*cos(atan(-1/a1));
yy1 = input.y(i,2) + Map.acc_bd*sin(atan(-1/a1));  
xx2 = input.y(i,1) - Map.acc_bd*cos(atan(-1/a1)); 
yy2 = input.y(i,2) - Map.acc_bd*sin(atan(-1/a1));

check_a = a1;
check_b = -1;
check_c1 = -a1*xx1+yy1;
check_c2 = -a1*xx2+yy2;

input.od(i+1,16) = check_a;
input.od(i+1,17) = check_c1;
input.od(i+1,18) = check_a;
input.od(i+1,19) = check_c2;
end
input.od(1,16) = input.od(2,16);
input.od(1,17) = input.od(2,17);
input.od(1,18) = input.od(2,18);
input.od(1,19) = input.od(2,19);
end