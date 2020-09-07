function sim = init_state(sim,Map)
%% Initial state 
Ax = Map.WPT(Map.WPT_Now+1,1)-Map.WPT(Map.WPT_Now,1);
Ay = Map.WPT(Map.WPT_Now+1,2)-Map.WPT(Map.WPT_Now,2);

sim.init_psi = mod(atan2(Ay,Ax),2*pi);

sim.init_x = Map.WPT(Map.WPT_Now,1);
sim.init_y = Map.WPT(Map.WPT_Now,2);
end
