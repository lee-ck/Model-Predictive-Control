function  ACADO_main
clear all
%% your directory
addpath(genpath('/Users/user/Desktop/ACADO')) 

%% MPC sampling time
simTs = 0.1;           
%% MPC # of prediction horizon 
Num = 20;
DifferentialState ate cte yaw u r
Control du dr 

OnlineData max_U
OnlineData emax
OnlineData rmax
OnlineData w_yaw

OnlineData o1x
OnlineData o1y
OnlineData o1r

OnlineData o2x
OnlineData o2y
OnlineData o2r

OnlineData o3x
OnlineData o3y
OnlineData o3r

OnlineData o4x
OnlineData o4y
OnlineData o4r

OnlineData o5x
OnlineData o5y
OnlineData o5r

OnlineData o6x
OnlineData o6y
OnlineData o6r

OnlineData o7x
OnlineData o7y
OnlineData o7r

OnlineData o8x
OnlineData o8y
OnlineData o8r

OnlineData o9x
OnlineData o9y
OnlineData o9r

OnlineData o10x
OnlineData o10y
OnlineData o10r

OnlineData obs_softa
OnlineData obs_softb

f = dot([ate; cte; yaw; u; r]) == ...
    [cos(yaw-w_yaw)*u;
     -sin(yaw-w_yaw)*u;
     r;
     du;
     dr;
    ];

n_XD = length(diffStates);
n_U = length(controls);

eps = 1e-6;
obs1 = obs_softa/(1+exp(obs_softb*(sqrt(eps+((ate-o1x)^2/o1r^2 + (cte-o1y)^2/o1r^2) )-1)));
obs2 = obs_softa/(1+exp(obs_softb*(sqrt(eps+((ate-o2x)^2/o2r^2 + (cte-o2y)^2/o2r^2) )-1)));
obs3 = obs_softa/(1+exp(obs_softb*(sqrt(eps+((ate-o3x)^2/o3r^2 + (cte-o3y)^2/o3r^2) )-1)));
obs4 = obs_softa/(1+exp(obs_softb*(sqrt(eps+((ate-o4x)^2/o4r^2 + (cte-o4y)^2/o4r^2) )-1)));
obs5 = obs_softa/(1+exp(obs_softb*(sqrt(eps+((ate-o5x)^2/o5r^2 + (cte-o5y)^2/o5r^2) )-1)));
obs6 = obs_softa/(1+exp(obs_softb*(sqrt(eps+((ate-o6x)^2/o6r^2 + (cte-o6y)^2/o6r^2) )-1)));
obs7 = obs_softa/(1+exp(obs_softb*(sqrt(eps+((ate-o7x)^2/o7r^2 + (cte-o7y)^2/o7r^2) )-1)));
obs8 = obs_softa/(1+exp(obs_softb*(sqrt(eps+((ate-o8x)^2/o8r^2 + (cte-o8y)^2/o8r^2) )-1)));
obs9 = obs_softa/(1+exp(obs_softb*(sqrt(eps+((ate-o9x)^2/o9r^2 + (cte-o9y)^2/o9r^2) )-1)));
obs10 = obs_softa/(1+exp(obs_softb*(sqrt(eps+((ate-o10x)^2/o10r^2 + (cte-o10y)^2/o10r^2) )-1)));
obs11 = exp(obs_softb*(cte-emax));
obs12 = exp(-obs_softb*(cte+emax));
obs = obs1+obs2+obs3+obs4+obs5+obs6+obs7+obs8+obs9+obs10+obs11+obs12;
% obs = 0;

h = [diffStates; controls;obs];
% h = [diffStates; controls];
hN = [diffStates];

%% SIMexport (Integrator Setting)
acadoSet('problemname', 'sim');
sim = acado.SIMexport( simTs );
sim.setModel(f);
sim.set( 'INTEGRATOR_TYPE',             'INT_RK4' );
sim.set( 'NUM_INTEGRATOR_STEPS',         2 );
ACADO_name = './../UGV_NMPC';


%% MPCexport (optimal control setting)
acadoSet('problemname', 'NMPC');
ocp = acado.OCP( 0.0, simTs*Num, Num );
W_mat = eye(n_XD+n_U+1,n_XD+n_U+1);
% W_mat = eye(n_XD+n_U,n_XD+n_U);
WN_mat = eye(n_XD,n_XD);
W = acado.BMatrix(W_mat);
WN = acado.BMatrix(WN_mat);
ocp.minimizeLSQ( W, h );
ocp.minimizeLSQEndTerm( WN, hN );

ocp.subjectTo(  - max_U - u <= 0 );
ocp.subjectTo(  u - max_U <= 0 );
% ocp.subjectTo(  -(emax) - cte <= 0 );
% ocp.subjectTo(  cte - (emax) <= 0 );
ocp.subjectTo(  -rmax - r <= 0 );
ocp.subjectTo(  r - rmax <= 0 );

ocp.setModel(f);


mpc = acado.OCPexport( ocp );
mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
mpc.set( 'PRINTLEVEL',                  'NONE'              );
mpc.set( 'INTEGRATOR_TYPE',             'INT_EX_EULER'      );
mpc.set( 'CG_USE_OPENMP ',              'YES');
mpc.set( 'NUM_INTEGRATOR_STEPS',         2*Num                  );
mpc.set( 'LEVENBERG_MARQUARDT', 		 1e-8				);

% qpOASES dense solver
mpc.set( 'QP_SOLVER',           'QP_QPOASES'        );
mpc.set( 'SPARSE_QP_SOLUTION',  'FULL_CONDENSING_N2');
mpc.set( 'HOTSTART_QP',         'YES'             	);

mpc.exportCode( 'export_MPC' );
disp('qpOASES exported!');
% Your directory
copyfile('/Users/user/Desktop/ACADO/external_packages/qpoases','export_MPC/qpoases', 'f') 
cd export_MPC
    make_acado_solver(ACADO_name)
cd ..
end