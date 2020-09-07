function [sim, input] = Initial_parameter_car(sim, input)
%% Initial parameter setting

sim.X0   = [sim.init_x   sim.init_y  ...   % x y 
            sim.init_psi ...   % psi
            sim.target_velocity    0    0 ];   %  velocity steer acceleration; 
sim.Xref = sim.X0;
Uref     = [0 0];      
Uinit    = [0 0];        

input.x  = repmat(sim.X0,sim.Num+1,1);
Xref     = repmat(sim.Xref,sim.Num,1);

input.od = [];

Uref     = repmat(Uref,sim.Num,1);
input.u  = repmat(Uinit,sim.Num,1);

input.y  = [Xref(1:sim.Num,:) Uref  zeros(sim.Num,1)];
input.yN = Xref(sim.Num,:);
end
