clear all
addpath('../funcs')

figure(1);
clf;
hold on
grid on
box on
xlabel('x [m]')
ylabel('y [m]')
axis equal
set(gca,'FontSize',16)
set(gcf,'color','white');
cdata = [1,0,0;
         0,0,1;
         0,0,1;
         0,0,1];
     
%% Visualization parameter
scale = 1; % Robot scale factor
rob.a = 0.215*scale; %foward/backward distance of the weels (from the robot's center)
rob.b = 0.18*scale; %lateral distance of the weels (from the robot's center)
rob.r = 0.151*scale; %radius of the wheels
rob.CM = [0.0; 0.0; 0.12];
rob.size_body = [0.55 0.35 0.12]*scale; %Size of the robot (x, y, z) - only for animation
H0 = eye(4); H_cm = eye(4); H_cm(1:2,4) = -rob.CM(1:2);

%% Simulation setting
dt = 0.1;
tfinal = 100;
max_speed = 2.0;
target_speed = 1.0;
max_e = 1.75; % Road boundary/2 [m]
max_r = deg2rad(30); % max rotational speed [rad/s]
LOS_dist = max_e;

%% Generate road
wpt_w = 20;
WPT = [wpt_w*1/4 wpt_w*0/4;...
       wpt_w*3/4 wpt_w*0/4;...
       wpt_w*4/4 wpt_w*1/4;...
       wpt_w*4/4 wpt_w*3/4;...
       wpt_w*3/4 wpt_w*4/4;...
       wpt_w*1/4 wpt_w*4/4;...
       wpt_w*0/4 wpt_w*3/4;...
       wpt_w*0/4 wpt_w*1/4];
WPT = [WPT;WPT;WPT;WPT;WPT;WPT;WPT;WPT;WPT];
wptnum = 2;

% Plot road
for ii = 2:size(WPT,1)
    plot_road(WPT(ii-1,:),WPT(ii,:),max_e*2,'k',1,0);
    fill_circle_road(WPT(ii,1),WPT(ii,2),LOS_dist,'k',1,0);
end
plot(WPT(:,1),WPT(:,2),'w--','LineWidth',1.5)
plot(WPT(:,1),WPT(:,2),'wo','LineWidth',1.5)


%% Initialize robot state and input
state = [0,0,0,target_speed,0]; % x, y ,psi, u, r
input = [0,0]; % du, dr
robot_safe_r = 0.3; % robot's safety radius (for MPC)

%% Video write
Record = 0;
if Record
    vv = VideoWriter('test.avi');
    open(vv);
end

%% Obstacle setting
obs(1).state = [generate_random_point_on_road(WPT(2,:),WPT(3,:),max_e*1),0.3+rand/5];
obs(2).state = [generate_random_point_on_road(WPT(3,:),WPT(4,:),max_e*1),0.3+rand/5];
obs(3).state = [generate_random_point_on_road(WPT(4,:),WPT(5,:),max_e*1),0.3+rand/5];
obs(4).state = [generate_random_point_on_road(WPT(5,:),WPT(6,:),max_e*1),0.3+rand/5];
obs(5).state = [generate_random_point_on_road(WPT(6,:),WPT(7,:),max_e*1),0.3+rand/5];
obs(6).state = [generate_random_point_on_road(WPT(7,:),WPT(8,:),max_e*1),0.3+rand/5];
obs(7).state = [generate_random_point_on_road(WPT(8,:),WPT(9,:),max_e*1),0.3+rand/5];
obs(8).state = [generate_random_point_on_road(WPT(9,:),WPT(10,:),max_e*1),0.3+rand/5];
obs(9).state = [generate_random_point_on_road(WPT(10,:),WPT(11,:),max_e*1),0.3+rand/5];
obs(10).state = [generate_random_point_on_road(WPT(11,:),WPT(12,:),max_e*1),0.3+rand/5];
num_obs = size(obs,2);
figure(1);

% Plot obstacles
for jjj = 1:num_obs
    [h1, h2, h3] = plotCylinderWithCaps(obs(jjj).state(3),...
        [obs(jjj).state(1) obs(jjj).state(2)],2,20,'w',0.7,0.7);
end

%% MPC setting        
iter_max = 50;
Num = 20;
nx = 5; nu = 2; obs_on = 1;
MPC.x0 = zeros(1,nx);
MPC.x = zeros(Num+1,nx);
MPC.u = zeros(Num,nu);
MPC.od = zeros(Num+1,36); % OnlineData u w_yaw
MPC.y = zeros(Num,nx+nu+obs_on);
MPC.yN = zeros(1,nx); % cte yaw r Steer; 

MPC.W = diag([0,... % ate
              1,...% cte
              1,... % yaw
              5,...% u
              0.0,...% r
              1,...% du
              1,...% dr
              1e2]);  % obsacle avoidance weight
MPC.WN = MPC.W(1:nx,1:nx);          
UGV_save = [];
back = 0;

for t = 0:dt:tfinal        
    %% WPT Number Update   
    if norm(state(1:2)-WPT(wptnum,:)) < LOS_dist
    wptnum = wptnum+1;
        if wptnum >= size(WPT,1)
            wptnum = size(WPT,1);
        end
    end
    
    %% NMPC Planner   
    %% State Transform (Inertial -> Body)
    C_point_MPC = closestPointOnLine(state(1:2), WPT(wptnum-1,:), WPT(wptnum,:));
    result = isLeft(WPT(wptnum,1:2), WPT(wptnum-1,1:2), state(1:2));
    cte = result*norm(C_point_MPC(1:2)-state(1:2));    
    ate = norm(C_point_MPC(1:2)-WPT(wptnum-1,:));
    if norm(C_point_MPC(1:2)-WPT(wptnum,:)) > norm(WPT(wptnum-1,:)-WPT(wptnum,:))
        ate = -ate;
    end
    wpt_psi = atan2(WPT(wptnum,2) - WPT(wptnum-1,2), WPT(wptnum,1) - WPT(wptnum-1,1));
    if (mod(wpt_psi,2*pi) > pi-pi/3) && (mod(wpt_psi,2*pi) < pi+pi/3)    
        back = 1;
    else
        back = 0;
    end
    wpt_psi = ssa(wpt_psi,back);
    MPC.x0 = [ate+0*randn/30, cte+0*randn/30, ssa(state(3),back), state(4), state(5)];

    %% Refernce
    MPC.y(:,1) = ate:target_speed*dt:ate+target_speed*dt*(Num-1);
    MPC.yN(:,1) = ate+target_speed*dt*Num;

    MPC.y(:,3) = wpt_psi;
    MPC.y(:,4) = target_speed;
    MPC.yN(:,3) = wpt_psi;
    MPC.yN(:,4) = target_speed;

    %% Online Data 
    MPC.od(:,1) = max_speed;
    MPC.od(:,2) = max_e;
    MPC.od(:,3) = max_r;
    MPC.od(:,4) = wpt_psi;
    
    % obstacle info -> MPC
    for jj = 1:num_obs
        xx = obs(jj).state(1) - WPT(wptnum-1,1);
        yy = obs(jj).state(2) - WPT(wptnum-1,2);
        d = cos(wpt_psi)*xx + sin(wpt_psi)*yy;
        e = -(-sin(wpt_psi)*xx + cos(wpt_psi)*yy);
        MPC.od(:,5+3*(jj-1)) = d;
        MPC.od(:,6+3*(jj-1)) = e;
        MPC.od(:,7+3*(jj-1)) = obs(jj).state(3) + robot_safe_r;
    end       
    MPC.od(:,35) = 1;
    MPC.od(:,36) = 6;


    %% Solve MPC   
    tic
    iterNumth = 0;
    while iterNumth < iter_max
        MPC = yaw_discontinuity(MPC);        
        output = UGV_NMPC(MPC);
        MPC.x = [output.x(2:end,:); output.x(end,:)];
        MPC.u = [output.u(2:end,:); output.u(end,:)];
        iterNumth = iterNumth + 1;
        if output.info.kktValue < 1e-4
            break;
        end
    end   
    ctime = toc;

    input = MPC.u(1,1:2);
    %% Update UGV
    statedot = UGV_kinematics(state,input);
    state = state + statedot'*dt;
    UGV_save = [UGV_save; t, state, input, ctime];
    
   
    %% Plot
    if mod(t,dt*5) < 0.01
        if t>0
            delete(MPC_Plot)
            delete(Traj_Plot)
            delete(Traj_Plot_CM)
            delete(h_body)
            delete(h_w1)
            delete(h_w2)
            delete(h_w3)
            delete(h_w4)
            
        end
        output_x = MPC.x(:,1)*cos(wpt_psi) + MPC.x(:,2)*sin(wpt_psi) + WPT(wptnum-1,1);
        output_y = MPC.x(:,1)*sin(wpt_psi) - MPC.x(:,2)*cos(wpt_psi) + WPT(wptnum-1,2);
        
        MPC_Plot = plot(output_x,output_y,'g-','LineWidth',2);
        Traj_Plot = surface([UGV_save(:,2)';UGV_save(:,2)'],...
            [UGV_save(:,3)';UGV_save(:,3)'],...
            [UGV_save(:,3)';UGV_save(:,3)']*0,...
            [(UGV_save(:,5)');(UGV_save(:,5)')],...
            'FaceColor','none','EdgeColor','interp','LineWidth',2);
        if t == 0
        caxis([-max_speed max_speed])
        colorbar;
        end

        Traj_Plot_CM = plot(state(1),state(2),'ro','LineWidth',2);

        %Plot body
        H_body = [Rot('z',state(3)), [state(1);state(2);rob.r]; 0 0 0 1];
        h_body = plot_block(H0*H_body*H_cm,rob.size_body,[0 0 0]+0.5,0.8,'-');

        width_wheel = 0.2*scale;
        number_faces = 10*scale;
        %Plot wheel 1
        H_w10 = eye(4);
        H_w10(1:3,1:3) = Rot('x',pi/2);
        H_w10(1:3,4) = [rob.a; -(rob.b+0.02); 0];
        H_w1 = H_body*H_cm*H_w10;
        H_w1(1:3,1:3) = H_w1(1:3,1:3)*Rot('z',-t*sign(state(4)));
        h_w1 = plot_cylinder(H0*H_w1,width_wheel,rob.r,[0.7 0 0]+0.1,0.7,number_faces,'-');
        %Plot wheel 2
        H_w20 = eye(4);
        H_w20(1:3,1:3) = Rot('x',pi/2);
        H_w20(1:3,4) = [-rob.a; -(rob.b+0.02); 0];
        H_w2 = H_body*H_cm*H_w20;
        H_w2(1:3,1:3) = H_w2(1:3,1:3)*Rot('z',-t*sign(state(4)));
        h_w2 = plot_cylinder(H0*H_w2,width_wheel,rob.r,'m',0.7,number_faces,'-');
        %Plot wheel 3
        H_w30 = eye(4);
        H_w30(1:3,1:3) = Rot('x',pi/2);
        H_w30(1:3,4) = [-rob.a; (rob.b+0.02); 0];
        H_w3 = H_body*H_cm*H_w30;
        H_w3(1:3,1:3) = H_w3(1:3,1:3)*Rot('z',-t*sign(state(4)));
        h_w3 = plot_cylinder(H0*H_w3,width_wheel,rob.r,'m',0.7,number_faces,'-');
        %Plot wheel 4
        H_w40 = eye(4);
        H_w40(1:3,1:3) = Rot('x',pi/2);
        H_w40(1:3,4) = [rob.a; (rob.b+0.02); 0];
        H_w4 = H_body*H_cm*H_w40;
        H_w4(1:3,1:3) = H_w4(1:3,1:3)*Rot('z',-t*sign(state(4)));
        h_w4 = plot_cylinder(H0*H_w4,width_wheel,rob.r,[0.7 0 0]+0.1,0.7,number_faces,'-');
        xlim([-2 wpt_w+2])
        ylim([-2 wpt_w+2])

        xlim([state(1)-7.5 state(1)+7.5])
        ylim([state(2)-7.5 state(2)+7.5])
        
        view([0 50])
        drawnow        
        
        if Record
            frame = getframe(gcf);
            writeVideo(vv,frame);
        end
    end
  
end
if Record
    close(vv);
end

figure(2)
clf;
set(gcf,'color','white');
subplot(1,3,1)
hold on
grid on
box on
plot(UGV_save(:,1),UGV_save(:,5),'k-');
plot(UGV_save(:,1),UGV_save(:,5)*0+max_speed,'r--');
plot(UGV_save(:,1),UGV_save(:,5)*0-max_speed,'r--');
legend('Speed','Max Speed','Min speed')
xlim tight
ylim tight
xlabel('sim. time [s]')
subplot(1,3,2)
hold on
grid on
box on
plot(UGV_save(:,1),UGV_save(:,6),'k-');
plot(UGV_save(:,1),UGV_save(:,6)*0+max_r,'r--');
plot(UGV_save(:,1),UGV_save(:,6)*0-max_r,'r--');
legend('rad/s','Max rotational speed','Min rotational speed')
xlabel('sim. time [s]')
xlim tight
subplot(1,3,3)
hold on
grid on
box on
plot(UGV_save(:,1),UGV_save(:,9)*1000,'k-');
plot(UGV_save(:,1),UGV_save(:,9)*0+100,'r--');
legend('Calculation time','100 ms (10Hz)')
xlabel('sim. time [s]')
ylabel('ms')
xlim tight
ylim tight


