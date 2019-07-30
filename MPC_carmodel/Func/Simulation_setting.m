function sim = Simulation_setting(X0,sim)

sim.tol          = 0.05;
sim.maxIterNum   = 5;
sim.iter         = 0; 
sim.time         = 0;
sim.KKT_MPC      = [];
sim.INFO_MPC     = [];
sim.state        = X0;
sim.iterNum      = [];
sim.calTime      = [];
sim.timeElapsed  = 0;
sim.controls_MPC = [];
sim.Terminal_ref = [];
sim.check =[];
sim.sensor_range = 100;
sim.mode =  '6DOF';

end