%% Select type
clc
close all
clear all
warning('off')

addpath(genpath('./ACADO/'))
addpath(genpath('./Func/'))
% addpath(genpath('./Plot/'))

sim.simLength = 160;  % Simulation time
sim.Num = 50;        % Number of horizon
sim.simTs = 0.4;     % Timestep
plot_now = 1;
sim.target_velocity = 0.5;  % Target velocity (m/s)
%% MAP
Map.points = [0 5 10 10 20 25 30 35 35 30 30 20 10  0  -5   0 ;...
              0 0 0  10 10 15 15 10 5  0  -5 -5 -5 -5  -2.5 0];
        
Map = waypoint(Map,sim);
disp('Map test plot')
if plot_now
figure(1)
hold on
grid on
axis equal
plot(Map.points(1,:),Map.points(2,:),'b','LineWidth',1)
plot(Map.WPT(:,1),Map.WPT(:,2),'r','LineWidth',1)
plot(Map.WPT(:,1),Map.WPT(:,2),'ro','LineWidth',1)
% pause(2)
close
end
Map.acc_bd = 0.7;
Map.WPT_Now = 1;
Map.WPT_max = size(Map.WPT,1);

%% INITIAL PARAMETERS SIMULATION
sim = init_state(sim,Map);

%% Cost Weight
input.W  = diag([1,   1,   0.1,...    % x y yaw
                 0,   0,   0,...    % velocity steer acceleration
                 2,   2,   0]);     % del_steer del_acceleration obs

input.WN  = diag([2, 2,  0.2,...  % x y yaw
                 0, 0, 0]);        % velocity steer acceleration
                 
[sim, input] = Initial_parameter_car(sim, input);

%% SIMULATION LOOP
sim = Simulation_setting(sim.X0,sim);

%% MAIN
disp('NMPC Waypoint Tracking start');
while sim.time(end) < sim.simLength
    input.x0 = sim.state(end,:);
    sim.iterNumth = 0;
    sim.timeElapsed = 0;  
    
    
    %% Waypoint change
    sim.cur_x = input.x0(1);  

    %% Reference & Obstacle cost
    % CTE & LOS
    [input, sim] = Make_ref_car(Map,sim,input);
    [Map,input,sim] = Make_online_data_car(Map,input,sim);
    
    %% Solve NMPC OCP
    input.od = real(input.od);
    while sim.iterNumth<sim.maxIterNum
        input = yaw_discontinuity(input);
        output = ACADO_car(input);
        input.x = [output.x(2:end,:); output.x(end,:)];
        input.u = [output.u(2:end,:); output.u(end,:)];
        sim.iterNumth = sim.iterNumth + 1;
        sim.timeElapsed = sim.timeElapsed + output.info.cpuTime ;
        if output.info.kktValue < sim.tol
            break;
        end
    end

    %% Save the MPC step
    sim.KKT_MPC  = [sim.KKT_MPC; output.info.kktValue];
    sim.controls_MPC = [sim.controls_MPC; output.u(1,:)];
    sim.iterNum = [sim.iterNum;sim.iterNumth];
    sim.calTime = [sim.calTime; sim.timeElapsed];

        
    %% Simulate system
    sim.input.x = sim.state(end,:).';
    sim.input.u = output.u(1,:).';
    sim.states = integrate_car(sim.input);
    % log 
    sim.state = [sim.state; sim.states.value'];
    sim.iter = sim.iter+1;
    sim.nextTime = sim.iter*sim.simTs; 
    sim.time = [sim.time;sim.nextTime];
    sim.Terminal_ref = [sim.Terminal_ref; input.yN];
    if isnan(sim.state(end,1))
        close all
        disp('NMPC algorithm fail!');
        break;
    end
    
   %% Visualize
   if plot_now
    figure(1)
    clf;
    hold on
    grid on
    axis equal
    xlim([-5 40])
    ylim([-15 25])
    %plot map
    for i = 1: 5
        DrawCircle(input.od(1,3*i-0),input.od(1,3*i-2),input.od(1,3*i-1),1)
    end   
    plot(Map.WPT(:,1),Map.WPT(:,2),'r','LineWidth',1)

    %plot trajectory 
    plot(sim.state(:,1),sim.state(:,2),'k','LineWidth',2)
    plot(sim.state(end,1),sim.state(end,2),'bo','LineWidth',3)
    quiver(sim.state(end,1),sim.state(end,2),2*cos(sim.state(end,3)),2*sin(sim.state(end,3)),'LineWidth',2)
    
    %plot ref
    plot(input.y(:,1),input.y(:,2),'gx')
    acc_bd = 1;
    slack_v = 1;
    
    boundary1 = [];
    boundary2 = [];
    for i = 1:Map.WPT_max-1
        a = Map.WPT(i+1,:)-Map.WPT(i,:);
        a1 = a(2)/a(1);
        boundary1 = [boundary1; Map.WPT(i,1) + Map.acc_bd*cos(atan(-1/a1)) Map.WPT(i,2) + Map.acc_bd*sin(atan(-1/a1))];  
        boundary2 = [boundary2; Map.WPT(i,1) - Map.acc_bd*cos(atan(-1/a1)) Map.WPT(i,2) - Map.acc_bd*sin(atan(-1/a1))];

    end

        plot(boundary1(:,1),boundary1(:,2),'m.')
        plot(boundary2(:,1),boundary2(:,2),'m.')
     
        
    for i = 1:sim.Num
        if mod(i,1) == 0
        a = Map.WPT(input.index(i,1)+1,:)-Map.WPT(input.index(i,1),:);
            a1 = a(2)/a(1);         
            xx1 = input.y(i,1) + Map.acc_bd*cos(atan(-1/a1));
            yy1 = input.y(i,2) + Map.acc_bd*sin(atan(-1/a1));  
            xx2 = input.y(i,1) - Map.acc_bd*cos(atan(-1/a1)); 
            yy2 = input.y(i,2) - Map.acc_bd*sin(atan(-1/a1));
            plot(xx1,yy1,'c.')
            plot(xx2,yy2,'c.')
            
            check_a = a1;
            check_b = -1;
            check_c1 = -a1*xx1+yy1;
            check_c2 = -a1*xx2+yy2;
            dist1 = (check_a*input.y(i,1)-input.y(i,2)+check_c1);
            dist2 = (check_a*input.y(i,1)-input.y(i,2)+check_c2);
            if dist1*dist2 >0
                disp('error')
            end
        end
    end
    
    
    %plot output
    plot(output.x(:,1),output.x(:,2),'m','LineWidth',2)
    drawnow
   end
end

figure(2)
hold on
grid on
axis equal
%plot map
plot(Map.WPT(:,1),Map.WPT(:,2),'r','LineWidth',1)
plot(Map.WPT(:,1),Map.WPT(:,2),'ro','LineWidth',1)
for i = 1: 5
    DrawCircle(input.od(1,3*i-0),input.od(1,3*i-2),input.od(1,3*i-1),1)
end
%plot trajectory 
plot(sim.state(:,1),sim.state(:,2),'k','LineWidth',2)
plot(sim.state(end,1),sim.state(end,2),'bo','LineWidth',3)
quiver(sim.state(end,1),sim.state(end,2),2*cos(sim.state(end,3)),2*sin(sim.state(end,3)),'LineWidth',2)

%% SAVE RESULTS => rec.
result.Inputs = sim.controls_MPC;
result.States = sim.state;
result.Iter = sim.iterNum;
result.CalTime = sim.calTime;
result.KKT = sim.KKT_MPC;
result.SimTime = sim.time;

disp('Results');
disp(['Simulation Time: ' num2str(sim.simLength) 'sec']);
disp(['The number of prediction step: ' num2str(sim.Num) ]);
disp(['Timestep : ' num2str(sim.simTs) 'sec']);
disp(['All RTI Time: ' num2str(sum(sum(result.CalTime))) 'sec']);
disp(['Mean Time: ' num2str(sum(sum(result.CalTime)/(sim.iter))) 'sec']);

output = result.States;
mean_cpu_time = sum(result.CalTime)/(sim.iter);

