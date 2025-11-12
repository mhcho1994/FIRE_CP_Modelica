%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Topic: 27dof vehicle dynamics considering rollover                     %
% Author(s): Minhyun                                                     %
% Description:                                                           %
% 1. High-fidelity simulation model of rover                             %
%    Referenced from MATLAB toolbox page:                                %
%    [1] https://www.mathworks.com/help/ident/ug/                        %
%    modeling-a-vehicle-dynamics-system.html                             %
%    [2] https://www.mathworks.com/help/vdynblks/ref/dugoffwheel2dof.html%
%    Referenced from the papers:                                         %
%    [3] X. Yang, “Improvements in vehicle handling and stability by     %
%    a novel wheel slip coordination control scheme,” IJVD, vol. 62,     %
%    no. 2/3/4, p. 206, 2013, doi: 10.1504/IJVD.2013.052702.             %
% 2. To be updated:                                                      %
% - Moving backward from stationary (-v_{x}, reverse rotation of motor)  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%.. clear workspace, command windows, figures
clear all;
close all;
clc;

%.. add the folder/subfolders to path
addpath(genpath(pwd));

%.. miscellnaeous constant
const.rad2deg   =   180/pi;                                                 % [-] radian to deg
const.deg2rad   =   pi/180;                                                 % [-] deg to radian
const.g         =   9.8065;                                                 % [m/s^2] gravitational acceleration
const.B0        =   3.12e-5;                                                % [T] mean value of magnetic filed at the magnetic equator of the Earth's sufrace
const.ft2m      =   0.3048;                                                 % [-] feet to meter
const.lb2N      =   4.44822;                                                % [-] pound force to newton

%.. parameters for vehicle
% parameters that can be measured/identified (from sysID)
rover.mass_total            =   4.177;                                      % [kg] rover mass
rover.mass_wheel            =   0.141;                                      % [kg] rover tire mass (average of front/rear wheels, 147/145g)
rover.mass_unsprung_front   =   6*rover.mass_wheel;                         % [kg] front unsprung mass
rover.mass_unsprung_rear    =   6*rover.mass_wheel;                         % [kg] rear unsprung mass
rover.mass_sprung           =   rover.mass_total ...
                                -rover.mass_unsprung_front ...
                                -rover.mass_unsprung_rear;                  % [kg] sprung mass
rover.l_front               =   0.136;                               	    % [m] distance from front axle to cg
rover.l_rear                =   0.139;                               	    % [m] distance from rear axle to cg
rover.l_total               =   rover.l_front+rover.l_rear;                 % [m] distance from rear axle to front axle
rover.n_wheel               =   4;                                          % [-] number of wheels
rover.tw                    =   0.174+0.06;                                 % [m] rover trackwidth
rover.r_wheel               =   0.056;                                      % [m] rover wheel radius
rover.w_wheel               =   0.03;                                       % [m] rover wheel width
rover.h_raf                 =   0.076;                                      % [m] height of front unsprung mass roll axis
rover.h_rar                 =   0.076;                                      % [m] height of rear unsprung mass roll axis
rover.h_s                   =   0.101;                                      % [m] height of sprung mass roll axis

% parameters guessed (from references)
rover.Iyy_w                 =   1/2*rover.mass_wheel*rover.r_wheel^2;                       % [kg*m^2] y-axis moment of inertia of wheels
rover.Izz_w                 =   1/12*rover.mass_wheel*(3*rover.r_wheel^2+rover.w_wheel^2);  % [kg*m^2] z-axis moment of inertia of wheels
rover.Izz_t                 =   1/12*rover.mass_total*(rover.l_total^2+rover.tw^2);         % [kg*m^2] z-axis moment of inertia of total chassis
rover.Ixx_uf                =   1/12*rover.mass_unsprung_front*(rover.tw^2+(2*(rover.h_raf-rover.r_wheel))^2);  % [kg*m^2] x-axis moment of inertia of front unsprung mass
rover.Ixx_ur                =   1/12*rover.mass_unsprung_rear*(rover.tw^2+(2*(rover.h_rar-rover.r_wheel))^2);   % [kg*m^2] x-axis moment of inertia of rear unsprung mass
rover.Ixx_s                 =   1/12*rover.mass_sprung*(rover.tw^2+(2*(rover.h_s-rover.h_raf))^2);              % [kg*m^2] x-axis moment of inertia of sprung mass
rover.Ixz_s                 =   0;                                          % [kg*m^2] xz product inertia of sprung mass
rover.Iyy_s                 =   1/12*rover.mass_sprung*(rover.tw^2+rover.l_total^2);        % [kg*m^2] y-axis moment of inertia of sprung mass
rover.mu0                   =   0.8;                                      	% [-] maximum friction scaling coefficient, cf. [0.4 1.2]
rover.As                    =   0.00;                                      	% [-] friction reduction factor
rover.c_s                   =   4.00e2;                                     % [N] longitudinal stiffness, cf. 6.8e2
rover.c_alpha               =   2.40e3;                                    	% [N/rad] lateral stiffness, cf. 1.4e2
rover.Lrelx                 =   0.185;                                      % [m] longitudinal relaxation length
rover.Lrely                 =   0.185;                                      % [m] lateral realaxation length
rover.k_lt                  =   0;                                          % [-] lateral compliance rate of tire and suspension, cf. 2.6/k_zt                                              
rover.k_scf                 =   (20.0^2)*(2*rover.Izz_w);                   % [rad/N*m] steering compliance for front steering gear, wn = 20 rad/s                                    
rover.k_scv                 =   (2*0.707*20.0)*(2*rover.Izz_w);             % [rad*s/N*m] damping for front steering gear, zeta = 0.707
rover.k_zt                  =   510*(30/25.4+2*(56/25.4))*(42/35)*const.ft2m/const.lb2N;    % [N/m] vertical spring rate of tire, cf. REF 5 page B-14, tune pressure from 35psi to 42psi
rover.k_zd                  =   0.1*rover.k_zt;                             % [N*s/m] vertical damping of tire, added for stability

rover.k_sf                  =   (0.5*rover.mass_sprung*rover.l_rear/rover.l_total)*(24.36)^2;                   % [N/m] front suspension spring rate (adapted from experimental data)
rover.k_sdf                 =   2*(0.12)*sqrt(rover.k_sf*(0.5*rover.mass_sprung*rover.l_rear/rover.l_total));   % [N*s/m] front suspension damping rate (adapted from experimental data)
rover.k_orsf                =   25.0;                                                                           % [N*m/rad] overall roll stiffness of front axle
rover.k_tsf                 =   rover.k_orsf*(rover.k_zt*rover.tw^2/2)/ ...
                                ((rover.k_zt*rover.tw^2/2)-rover.k_orsf)+rover.k_sf*rover.tw^2/2;               % [N*m/rad] front auxiliary roll stiffness
rover.l_saf                 =   2/3*rover.tw;                                                                   % [m] length of the front axle arm (wheel to suspension pivot)
rover.tw_f                  =   rover.tw;                                                                       % [m] front trackwidth
rover.k_slf                 =   0.115;                                                                          % [-] lateral slope of equivalent single suspension arm, cf. REF 5 page C-2
rover.k_sadf                =   0.0;                                                                            % [-] suspension squat/lift ratio (in deceleration)
rover.k_sad2f               =   0.0;                                                                            % [-] suspension squat/lift ratio (in acceleration)
rover.k_sr                  =   (0.5*rover.mass_sprung*rover.l_front/rover.l_total)*(24.36)^2;                  % [N/m] front suspension spring rate (adapted from experimental data)
rover.k_sdr                 =   2*(0.12)*sqrt(rover.k_sr*(0.5*rover.mass_sprung*rover.l_front/rover.l_total));  % [N*s/m] front suspension damping rate (adapted from experimental data)
rover.k_orsr                =   rover.k_orsf;                                                                   % [N*m/rad] overall roll stiffness of front axle
rover.k_tsr                 =   rover.k_orsr*(rover.k_zt*rover.tw^2/2)/ ...
                                ((rover.k_zt*rover.tw^2/2)-rover.k_orsr)+rover.k_sr*rover.tw^2/2;               % [N*m/rad] front auxiliary roll stiffness
rover.l_sar                 =   2/3*rover.tw;                                                                   % [m] length of the front axle arm (wheel to suspension pivot)
rover.tw_r                  =   rover.tw;                                                                       % [m] front trackwidth
rover.k_slr                 =   0.115;                                                                          % [-] lateral slope of equivalent single suspension arm, cf. REF 5 page C-2
rover.k_sadr                =   0.0;                                                                            % [-] suspension squat/lift ratio (in deceleration)
rover.k_sad2r               =   0.0;                                                                            % [-] suspension squat/lift ratio (in acceleration)
rover.h_bs                  =   0.03;                                                                           % [m] suspension travel to bumpt stop (measured)
rover.k_bs                  =   2.0*rover.k_sf;                                                                 % [N/m] bump stop stiffness, cf. REF 5 page B-13
rover.k_ras                 =   1.0*10^3;                                                                       % [N/m] lateral spring rate at compliant pin joint between sprung and unsprung masses, cf. REF 5 page B-13
rover.k_rad                 =   1.0*2*sqrt(2*rover.mass_unsprung_front*rover.k_ras);                            % [N*s/m] lateral damping rate at compliant pin joint between sprung and unsprung masses, cf. REF 5 page B-14

% other parmeters
rover.s_max                 =   0.999;                                     	% [-] upper bound of slip ratio
rover.s_min                 =   -0.999;                                   	% [-] lower bound of slip ratio
rover.alpha_max             =   pi/2;                                     	% [rad] upper bound of slip angle
rover.alpha_min             =   -pi/2;                                   	% [rad] lower bound of slip angle

% motor and servo parameters
rover.Vs                    =   11.1;                                       % [V] supply voltage
rover.Np                    =   4;                                          % [-] number of pole pairs in the motor
rover.Nw                    =   20;                                         % [-] number of windings in each phase
rover.A                     =   0.005*0.02;                                 % [m^2] cross-sectional area of windings stator
rover.B                     =   1.2;                                        % [T] magnetic flux density of stator
rover.eta                   =   0.99;                                       % [-] mechanical conversion efficiency
rover.Kt_phi                =   rover.eta*rover.Np*rover.Nw*rover.A*rover.B;    % [N*m/A] motor torque constant (per-phase)
rover.Kb_phi                =   rover.Np*rover.Nw*rover.A*rover.B;          % [V*s/rad] motor back emf constant (per-phase)
rover.Kt_q                  =   sqrt(3/2)*rover.Kt_phi;                     % [N*m/A] motor torque constant (total)
rover.Kb_q                  =   sqrt(3/2)*rover.Kb_phi;                     % [V*s/rad] motor back emf constant (total)
rover.R_phi                 =   0.1;                                        % [Ohm] resistance of BLDC motor (per-phase)
rover.Jm                    =   1e-4;                                       % [kg*m^2] rotor inertia
rover.Le                    =   1e-2;                                       % [H] effective inductance
rover.b                     =   6.0e-4;                                     % [N*m*s] viscous friction coefficient
rover.gratio                =   2.5;                                        % [-] gear ratio between motor and wheel shafts
rover.delta_max             =   (28.28-3.00)*const.deg2rad;                 % [rad] maximum angle of steering servo
rover.deltadot_max          =   100*const.deg2rad;                          % [rad/s] maximum angular rate of steering servo

% emi parameters
rover.nu0                   =   4*pi*1e-7;                                  % [T*m/A] vacuum permeability
rover.wire_dir              =   [1; 0; 0];                                  % [-] current flowing wire direction
rover.x_wire                =   [0; 0.01; -0.02];                           % [m] relative position from current-flowing wire
rover.dist_wire             =   sqrt(rover.x_wire(1)^2+rover.x_wire(2)^2+rover.x_wire(3)^2);    % [m] given distance from wire
rover.B_earth0              =   3.12e-5;                                    % [T] mean magnetic field strength at the magnetic equator on the Earth's surface
rover.lat0                  =   40.42244465750271*const.deg2rad;            % [rad] latitude of the rover

%.. set up simulation parameters
sim.t0      =   0.0;                                                        % [sec] simulation start time
sim.tf      =   15.0;                                                       % [sec] simulation end time
sim.dt      =   0.001;                                                     	% [sec] simulation time step
sim.t       =   sim.t0:sim.dt:sim.tf;                                       % [sec] simulation time

%.. compare with experimental results
% load('circle_traj_exp.mat');

%.. set up states and initial states
sim.x       =   NaN*ones(41,size(sim.t,2));
sim.x(:,1)  =   zeros(size(sim.x,1),1);

%.. set up derived states: 
sim.y       =   NaN*ones(26,size(sim.t,2));
sim.y(:,1)  =   zeros(26,1);

%.. set up sensor fusion states:
filter.x        =   NaN*ones(4,size(sim.t,2));
filter.x(:,1)   =   eul2quat([0+pi/5; 0+pi/10; sim.x(5,1)+pi/3]);

%.. change the initial state to match with the sysID result
if exist('traj1','var') == 1
    first_turn_start_idx    =   535;
    first_turn_end_idx      =   800;
    sim.x(1,1)  =   traj1.x(first_turn_start_idx);
    sim.x(5,1)  =   traj1.y(first_turn_start_idx);
    sim.x(3,1)  =   traj1.yaw(first_turn_start_idx)*const.deg2rad-15*const.deg2rad;
    z_sim_bias  =   traj1.z(first_turn_start_idx:first_turn_end_idx);
    z_sim_bias  =   mean(z_sim_bias(find(~isnan(z_sim_bias))));
end

%.. set up control input history: Ta_fl, Ta_fr, Ta_rl, Ta_rr, delta
sim.u           =   NaN*ones(2,size(sim.t,2));

%.. accelerate forward, maximum velocity input test
% sim.u(1,:)      =   ones(1,size(sim.t,2));                                  % [-] voltage level
% sim.u(2,:)      =   zeros(1,size(sim.t,2));                                 % [rad] steering angle

%.. circle turn: matching turn radius
% sim.u(1,:)      =   0.20*ones(1,size(sim.t,2));                             % [-] voltage level
% sim.u(2,:)      =   rover.delta_max*ones(1,size(sim.t,2));                  % [rad] steering angle

%.. rollover
sim.u(1,:)                          =   1.0*ones(1,size(sim.t,2));          % [-] voltage level
sim.u(2,:)                          =   0.0*ones(1,size(sim.t,2));
sim.u(2,10/sim.dt+1:10.25/sim.dt)   =   2.65*const.deg2rad*ones(1,size(sim.u(2,10/sim.dt+1:10.25/sim.dt),2));   % [rad] steering angle

%.. stationary
% sim.u(1,:)      =   zeros(1,size(sim.t,2));                                 % [-] voltage level
% sim.u(2,:)      =   zeros(1,size(sim.t,2));                                 % [rad] steering angle

%.. run simulation
for idx = 1:size(sim.t,2)-1
    % sim.t(idx)
    % sim.x_aux(:,idx)
    [t_temp,X_temp]     =   rk4(@(t,x,u) eight_dof_rover(t,x,u,sim.y(:,idx),rover,const),sim.t(idx),sim.x(:,idx),sim.u(:,idx),sim.dt); % ,sim.x_aux(:,idx),rover,const
    sim.x(:,idx+1)      =   X_temp(:,2);
    [~,Y_temp]          =   eight_dof_rover(sim.t(idx),sim.x(:,idx),sim.u(:,idx),sim.y(:,idx),rover,const);
    sim.y(:,idx+1)      =   Y_temp;
    % Xfil_temp           =   madgwick_AHRS(filter.x(:,idx), [0.0; 0.0; sim.x(6,idx+1)], ...
    %                             [sim.x_aux(1,idx+1); sim.x_aux(2,idx+1); 0]-quat2rot(eul2quat([sim.x(7,idx+1); 0.0; sim.x(5,idx+1)]))'*[0.0; 0.0; const.g], ...
    %                             [sim.x_aux(24,idx+1); sim.x_aux(25,idx+1); sim.x_aux(26,idx+1)]+[sim.x_aux(21,idx+1); sim.x_aux(22,idx+1); sim.x_aux(23,idx+1)], sim.dt);
    % filter.x(:,idx+1)   =   Xfil_temp;
end  

%.. plot colors
RGB =   orderedcolors("gem");
H   =   rgb2hex(RGB);

%.. trajectory plot: plotting 3D trajectory
if exist('traj1','var') == 1
    figure;
    set(gcf,'color','w');
    hold on;
    grid on;
    plot3(sim.x(1,:),sim.x(5,:),-sim.x(15,:)+z_sim_bias*ones(size(sim.x(1,:),1),size(sim.x(1,:),2)));
    plot3(traj1.x(first_turn_start_idx),traj1.y(first_turn_start_idx),traj1.z(first_turn_start_idx),'o','Color','b','MarkerSize',10,'MarkerFaceColor','#D9FFFF')
    plot3(traj1.x(first_turn_end_idx),traj1.y(first_turn_end_idx),traj1.z(first_turn_end_idx),'o','Color','r','MarkerSize',10,'MarkerFaceColor','#FFA500')
    plot3(traj1.x(1,first_turn_start_idx:first_turn_end_idx),...
        traj1.y(1,first_turn_start_idx:first_turn_end_idx),traj1.z(1,first_turn_start_idx:first_turn_end_idx));
    xlabel('X [m]');
    ylabel('Y [m]');
    zlabel('Z [m]');
    zlim([0.0,0.2]);
    view([120,30]);
    legend('Simulated Trajectory','Start Point','End Point','Experiment Trajectory');
    title('Comparison of High-fidelity Model and Experimental Data (3D Trajectory)');
    
else
    figure;
    set(gcf,'color','w');
    hold on;
    grid on;
    plot3(sim.x(1,:),sim.x(5,:),-sim.x(15,:));
    xlabel('X [m]');
    ylabel('Y [m]');
    zlabel('Z [m]');
    zlim([-0.2,0.2]);
    view([120,30]);
    legend('Simulated Trajectory');
    title('3D Trajectory');

end

%.. trajectory plot: plotting 2D trajectory
if exist('traj1','var') == 1
    figure;
    set(gcf,'color','w');
    hold on;
    grid on;
    plot(sim.x(1,:),sim.x(5,:));
    plot(traj1.x(first_turn_start_idx),traj1.y(first_turn_start_idx),'o','Color','b','MarkerSize',10,'MarkerFaceColor','#D9FFFF')
    plot(traj1.x(first_turn_end_idx),traj1.y(first_turn_end_idx),'o','Color','r','MarkerSize',10,'MarkerFaceColor','#FFA500')
    plot(traj1.x(1,first_turn_start_idx:first_turn_end_idx),traj1.y(1,first_turn_start_idx:first_turn_end_idx));
    xlabel('X [m]');
    ylabel('Y [m]');
    legend('Simulated Trajectory','Start Point','End Point','Experiment Trajectory','Location','best');
    title('Comparison of High-fidelity Model and Experimental Data (2D Trajectory)');

else
    figure;
    set(gcf,'color','w');
    hold on;
    grid on;
    plot(sim.x(1,:),sim.x(5,:),'linewidth',1.5);
    axis equal;
    xlabel('X [m]');
    ylabel('Y [m]');
    legend('Simulated Trajectory');
    title('2D Trajectory');
end

%.. velocity plot: plotting 2D velocity
if exist('traj1','var') == 1
    figure;
    set(gcf,'color','w');
    ax1(1) = subplot(2,1,1);
    hold on;
    grid on;
    plot(sim.t,sim.x(2,:),'linewidth',1.5);
    plot(sim.t,sim.x(25,:)*rover.r_tire,'linewidth',1.5,'linestyle','--')
    plot(traj1.time(first_turn_start_idx:first_turn_end_idx)-traj1.time(first_turn_start_idx),traj1.u(first_turn_start_idx:first_turn_end_idx),'linewidth',1.5,'color',H(4));
    ylabel('U [m/s]');
    legend('Simulated Data (Vehicle Velocity)', 'Simulated Data (Wheel Velocity)', 'Experimental Data (Vehicle Marker)','Location','southeast');
    title('2D Velocity')
    ax1(2) = subplot(2,1,2);
    hold on;
    grid on;
    plot(sim.t,sim.x(6,:),'linewidth',1.5);
    plot(traj1.time(first_turn_start_idx:first_turn_end_idx)-traj1.time(first_turn_start_idx),traj1.v(first_turn_start_idx:first_turn_end_idx),'linewidth',1.5,'color',H(4));
    ylabel('V [m/s]');
    xlabel('Time [s]')
    legend('Simulated Data (Vehicle Velocity)', 'Experimental Data (Vehicle Marker)','Location','southeast');

else
    figure;
    set(gcf,'color','w');
    ax1(1) = subplot(2,1,1);
    hold on;
    grid on;
    plot(sim.t,sim.x(2,:),'linewidth',1.5);
    plot(sim.t,sim.x(25,:)*rover.r_wheel,'linewidth',1.5,'linestyle','--')
    ylabel('U [m/s]');
    legend('Simulated Data (Vehicle Velocity)', 'Simulated Data (Wheel Velocity)','Location','southeast');
    title('2D Velocity');
    ax1(2) = subplot(2,1,2);
    hold on;
    grid on;
    plot(sim.t,sim.x(6,:),'linewidth',1.5);
    ylabel('V [m/s]');
    xlabel('Time [s]')
    legend('Simulated Data (Vehicle Velocity)','Location','southeast');
end

%.. trajectory/velocity plot: plotting vertical motion
if exist('traj1','var') == 1
    figure;
    set(gcf,'color','w');
    ax2(1) = subplot(2,1,1);
    hold on;
    grid on;
    plot(sim.t,-sim.x(11,:),'linewidth',1.5);
    plot(sim.t,-sim.x(13,:),'linewidth',1.5);
    plot(sim.t,-sim.x(15,:),'linewidth',1.5);
    plot(traj1.time(first_turn_start_idx:first_turn_end_idx)-traj1.time(first_turn_start_idx),traj1.z(first_turn_start_idx:first_turn_end_idx)-z_sim_bias,'linewidth',1.5,'color',H(4));
    ylabel('Z [m]');
    legend('Simulated Data (Sprung)','Simulated Data (Front Unsprung)','Simulated Data (Rear Unsprung)','Experimental Data (Vehicle Marker)');
    title('Vertical Motion');
    ax2(2) = subplot(2,1,2);
    hold on;
    grid on;
    plot(sim.t,-sim.x(12,:),'linewidth',1.5);
    plot(sim.t,-sim.x(14,:),'linewidth',1.5);
    plot(sim.t,-sim.x(16,:),'linewidth',1.5);
    plot(traj1.time(first_turn_start_idx:first_turn_end_idx)-traj1.time(first_turn_start_idx),traj1.w(first_turn_start_idx:first_turn_end_idx),'linewidth',1.5,'color',H(4));
    ylabel('W [m/s]');
    xlabel('Time [s]')

else
    figure;
    set(gcf,'color','w');
    ax2(1) = subplot(2,1,1);
    hold on;
    grid on;
    plot(sim.t,-sim.x(11,:),'linewidth',1.5);
    plot(sim.t,-sim.x(13,:),'linewidth',1.5);
    plot(sim.t,-sim.x(15,:),'linewidth',1.5);
    ylabel('Z [m]');
    legend('Simulated Data (Sprung)','Simulated Data (Front Unsprung)','Simulated Data (Rear Unsprung)');
    title('Vertical Motion');
    ax2(2) = subplot(2,1,2);
    hold on;
    grid on;
    plot(sim.t,-sim.x(12,:),'linewidth',1.5);
    plot(sim.t,-sim.x(14,:),'linewidth',1.5);
    plot(sim.t,-sim.x(16,:),'linewidth',1.5);
    ylabel('W [m/s]');
    xlabel('Time [s]')
end

%.. tire force plot: showing X, Y, Z forces in tire coordinates
figure;
set(gcf,'color','w');
ax2(1) = subplot(3,1,1);
hold on;
grid on;
plot(sim.t,sim.y(1,:),'linewidth',1.5);
plot(sim.t,sim.y(2,:),'linewidth',1.5);
plot(sim.t,sim.y(3,:),'linewidth',1.5);
plot(sim.t,sim.y(4,:),'linewidth',1.5);
ylabel('Fz [N]');
legend('Front Left', 'Front Right', 'Rear Left', 'Rear Right');
title('Tire Forces');
ax2(2) = subplot(3,1,2);
hold on;
grid on;
plot(sim.t,sim.y(5,:),'linewidth',1.5);
plot(sim.t,sim.y(6,:),'linewidth',1.5);
plot(sim.t,sim.y(7,:),'linewidth',1.5);
plot(sim.t,sim.y(8,:),'linewidth',1.5);
ylabel('Fx [N]');
ax2(3) = subplot(3,1,3);
hold on;
grid on;
plot(sim.t,sim.y(9,:),'linewidth',1.5);
plot(sim.t,sim.y(10,:),'linewidth',1.5);
plot(sim.t,sim.y(11,:),'linewidth',1.5);
plot(sim.t,sim.y(12,:),'linewidth',1.5);
ylabel('Fy [N]');

%.. attitude plot: showing attitude of sprung, unsprung masses
if exist('traj1','var') == 1
    figure;
    set(gcf,'color','w');
    ax3(1) = subplot(3,1,1);
    hold on;
    grid on;
    plot(sim.t,sim.x(21,:)*const.rad2deg,'linewidth',1.5);
    plot(sim.t,sim.x(17,:)*const.rad2deg,'linewidth',1.5);
    plot(sim.t,sim.x(19,:)*const.rad2deg,'linewidth',1.5);
    plot(traj1.time(first_turn_start_idx:first_turn_end_idx)-traj1.time(first_turn_start_idx),-traj1.roll(first_turn_start_idx:first_turn_end_idx),'linewidth',1.5,'color',H(4));
    legend('Simulated Data (Sprung)','Simulated Data (Front Unsprung)','Simulated Data (Rear Unsprung)','Experimental Data (Vehicle Marker)');
    ylabel('Roll [deg]');
    title('Attitude')
    ax3(2) = subplot(3,1,2);
    hold on;
    grid on;
    plot(sim.t,(mod(sim.x(3,:)+pi,2*pi)-pi)*const.rad2deg,'linewidth',1.5);
    plot(traj1.time(first_turn_start_idx:first_turn_end_idx)-traj1.time(first_turn_start_idx),traj1.yaw(first_turn_start_idx:first_turn_end_idx),'linewidth',1.5,'color',H(4));
    legend('Simulated Data (Total Chassis)','Experimental Data (Vehicle Marker)');
    ylabel('Yaw [deg]');
    ax3(3) = subplot(3,1,3);
    hold on;
    grid on;
    plot(sim.t,sim.x(23,:)*const.rad2deg,'linewidth',1.5);
    plot(traj1.time(first_turn_start_idx:first_turn_end_idx)-traj1.time(first_turn_start_idx),traj1.pitch(first_turn_start_idx:first_turn_end_idx),'linewidth',1.5,'color',H(4));
    legend('Simulated Data (Sprung)','Experimental Data (Vehicle Marker)');
    ylabel('Pitch [deg]');
    xlabel('Time [s]');

else
    figure;
    set(gcf,'color','w');
    ax3(1) = subplot(3,1,1);
    hold on;
    grid on;
    plot(sim.t,sim.x(21,:)*const.rad2deg,'linewidth',1.5);
    plot(sim.t,sim.x(17,:)*const.rad2deg,'linewidth',1.5);
    plot(sim.t,sim.x(19,:)*const.rad2deg,'linewidth',1.5);
    legend('Simulated Data (Sprung)','Simulated Data (Front Unsprung)','Simulated Data (Rear Unsprung)');
    ylabel('Roll [deg]');
    title('Attitude')
    ax3(2) = subplot(3,1,2);
    hold on;
    grid on;
    plot(sim.t,(mod(sim.x(3,:)+pi,2*pi)-pi)*const.rad2deg,'linewidth',1.5);
    legend('Simulated Data (Total Chassis)');
    ylabel('Yaw [deg]');
    ax3(3) = subplot(3,1,3);
    hold on;
    grid on;
    plot(sim.t,sim.x(23,:)*const.rad2deg,'linewidth',1.5);
    legend('Simulated Data (Sprung)');
    ylabel('Pitch [deg]');
    xlabel('Time [s]');
    
end

%.. rate plot: showing rate of sprung, unsprung masses
figure;
set(gcf,'color','w');
ax4(1) = subplot(3,1,1);
hold on;
grid on;
plot(sim.t,sim.x(22,:)*const.rad2deg,'linewidth',1.5);
plot(sim.t,sim.x(18,:)*const.rad2deg,'linewidth',1.5);
plot(sim.t,sim.x(20,:)*const.rad2deg,'linewidth',1.5);
legend('Simulated Data (Sprung)','Simulated Data (Front Unsprung)','Simulated Data (Rear Unsprung)');
ylabel('Roll Rate [deg/s]');
ax4(2) = subplot(3,1,2);
hold on;
grid on;
plot(sim.t,sim.x(4,:)*const.rad2deg,'linewidth',1.5);
legend('Simulated Data (Total Chassis)');
ylabel('Yaw Rate [deg/s]');
ax4(3) = subplot(3,1,3);
hold on;
grid on;
plot(sim.t,sim.x(24,:)*const.rad2deg,'linewidth',1.5);
legend('Simulated Data (Sprung)');
ylabel('Pitch Rate [deg/s]');
xlabel('Time [s]');

%.. tire slip plot: showing longitudinal, lateral slip
figure;
set(gcf,'color','w');
ax4(1) = subplot(2,1,1);
hold on;
grid on;
plot(sim.t,sim.x(31,:),'linewidth',1.5);
plot(sim.t,sim.x(32,:),'linewidth',1.5);
plot(sim.t,sim.x(33,:),'linewidth',1.5);
plot(sim.t,sim.x(34,:),'linewidth',1.5);
ylabel('Longitudinal Slip [-]');
legend('Front Left', 'Front Right', 'Rear Left', 'Rear Right');
ax2(4) = subplot(2,1,2);
hold on;
grid on;
plot(sim.t,sim.x(35,:),'linewidth',1.5);
plot(sim.t,sim.x(36,:),'linewidth',1.5);
plot(sim.t,sim.x(37,:),'linewidth',1.5);
plot(sim.t,sim.x(38,:),'linewidth',1.5);
ylabel('Lateral Slip [-]');
xlabel('Time [s]');


% figure;
% set(gcf,'color','w');
% ax4(1) = subplot(2,1,1);
% hold on;
% grid on;
% plot(sim.t,sim.u(1,:),'linewidth',1.5);
% ylabel('Throttle Voltage [V]');
% ax4(2) = subplot(2,1,2);
% hold on;
% grid on;
% plot(sim.t,sim.u(2,:)*const.rad2deg,'linewidth',1.5);
% plot(sim.t,sim.x(24,:)*const.rad2deg,'linewidth',1.5);
% legend('Command', 'Actual');
% ylabel('Steering [deg]');

% figure;
% set(gcf,'color','w');
% ax5(1) = subplot(4,1,1);
% hold on;
% grid on;
% plot(sim.t,sim.x_aux(17,:),'linewidth',1.5);
% ylabel('Transformed Voltage Input [V]');
% ax5(2) = subplot(4,1,2);
% hold on;
% grid on;
% plot(sim.t,sim.x(13,:),'linewidth',1.5);
% ylabel('Transformed Current Input [A]');
% ax5(3) = subplot(4,1,3);
% hold on;
% grid on;
% plot(sim.t,(mod(sim.x(14,:)+pi,2*pi)-pi)*const.rad2deg,'linewidth',1.5);
% ylabel('Motor Stator Position [deg]');
% ax5(4) = subplot(4,1,4);
% hold on;
% grid on;
% plot(sim.t,sim.x(15,:)*const.rad2deg,'linewidth',1.5);
% ylabel('Motor Angular Velocity [deg/s]');

% figure;
% set(gcf,'color','w');
% ax6(1) = subplot(3,1,1);
% hold on;
% grid on;
% plot(sim.t,sim.x_aux(21,:),'linewidth',1.5);
% plot(sim.t,sim.x_aux(18,:),'linewidth',1.5);
% plot(sim.t,sim.x_aux(24,:),'linewidth',1.5);
% ylabel('Magnetic Field X-Direction [T]');
% legend('Wire-Induced', 'Motor-Induced', 'Earth');
% ax6(2) = subplot(3,1,2);
% hold on;
% grid on;
% plot(sim.t,sim.x_aux(22,:),'linewidth',1.5);
% plot(sim.t,sim.x_aux(19,:),'linewidth',1.5);
% plot(sim.t,sim.x_aux(25,:),'linewidth',1.5);
% ylabel('Magnetic Field Y-Direction [T]');
% ax6(3) = subplot(3,1,3);
% hold on;
% grid on;
% plot(sim.t,sim.x_aux(23,:),'linewidth',1.5);
% plot(sim.t,sim.x_aux(20,:),'linewidth',1.5);
% plot(sim.t,sim.x_aux(26,:),'linewidth',1.5);
% ylabel('Magnetic Field Z-Direction [T]');

% filter.euler    =   NaN*ones(3,size(sim.t,2));
% sim.x_quat      =   NaN*ones(4,size(sim.t,2));
% 
% for idx = 1:size(sim.t(1:end),2)
%     filter.euler(:,idx) =   quat2eul(filter.x(:,idx));
%     sim.x_quat(:,idx)   =   eul2quat([sim.x(7,idx); 0.0; sim.x(5,idx)]);
% end
% 
% figure;
% set(gcf,'color','w');
% ax7(1) = subplot(3,1,1);
% hold on;
% grid on;
% plot(sim.t,(mod(sim.x(5,:)+pi,2*pi)-pi)*const.rad2deg,'linewidth',1.5);
% plot(sim.t,(mod(filter.euler(3,:)+pi,2*pi)-pi)*const.rad2deg,'linewidth',1.5);
% ylabel('Yaw [deg]')
% legend('True Value', 'Estimate');
% ax7(2) = subplot(3,1,2);
% hold on;
% grid on;
% plot(sim.t,sim.x(7,:)*const.rad2deg,'linewidth',1.5);
% plot(sim.t,filter.euler(1,:)*const.rad2deg,'linewidth',1.5);
% ylabel('Roll [deg]')
% legend('True Value', 'Estimate');
% ax7(3) = subplot(3,1,3);
% hold on;
% grid on;
% plot(sim.t,zeros(size(sim.x(7,:),1),size(sim.x(7,:),2))*const.rad2deg,'linewidth',1.5);
% plot(sim.t,filter.euler(2,:)*const.rad2deg,'linewidth',1.5);
% ylabel('Pitch [deg]')
% legend('True Value', 'Estimate');

% figure;
% set(gcf,'color','w');
% ax8(1) = subplot(4,1,1);
% hold on;
% grid on;
% plot(sim.t,sim.x_quat(1,:),'linewidth',1.5);
% plot(sim.t,filter.x(1,:),'linestyle','--','linewidth',1.5);
% ylabel('q0 [-]')
% legend('True Value', 'Estimate');
% ax8(2) = subplot(4,1,2);
% hold on;
% grid on;
% plot(sim.t,sim.x_quat(2,:),'linewidth',1.5);
% plot(sim.t,filter.x(2,:),'linestyle','--','linewidth',1.5);
% ylabel('q1 [-]')
% ax8(3) = subplot(4,1,3);
% hold on;
% grid on;
% plot(sim.t,sim.x_quat(3,:),'linewidth',1.5);
% plot(sim.t,filter.x(3,:),'linestyle','--','linewidth',1.5);
% ylabel('q2 [-]')
% ax8(4) = subplot(4,1,4);
% hold on;
% grid on;
% plot(sim.t,sim.x_quat(4,:),'linewidth',1.5);
% plot(sim.t,filter.x(4,:),'linestyle','--','linewidth',1.5);
% ylabel('q3 [-]')

function [xdot,x_aux_p] = eight_dof_rover(t,x,u,x_aux,param,const) % ,x_aux,param,const

    %.. get constants
    g           =   const.g;                                                % [m/s^2] gravitational acceleration
    % ft2m        =   const.ft2m;
    % lb2N        =   const.lb2N;

    %.. set up the parameters
    % parameters that can be measured/identified (from sysID)
    m_t         =   param.mass_total;                                       % [kg] rover mass
    m_uf        =   param.mass_unsprung_front;                              % [kg] front unsprung mass
    m_ur        =   param.mass_unsprung_rear;                               % [kg] rear unsprung mass
    m_s         =   param.mass_sprung;                                      % [kg] sprung mass
    lf          =   param.l_front;                                          % [m] distance from front axle to cg
    lr          =   param.l_rear;                                           % [m] distance from rear axle to cg
    lt          =   param.l_total;                                          % [m] distance from rear axle to front axle
    nw          =   param.n_wheel;                                          % [-] number of wheels
    tw          =   param.tw;                                               % [m] rover trackwidth
    rw          =   param.r_wheel;                                          % [m] rover wheel radius
    huf         =   param.h_raf;                                            % [m] height of front unsprung mass roll axis
    hur         =   param.h_rar;                                            % [m] height of rear unsprung mass roll axis
    hs          =   param.h_s;                                              % [m] height of sprung mass roll axis

    % parameters guessed (from references)
    Iyy_w       =   param.Iyy_w;                                            % [kg*m^2] y-axis moment of inertia of wheels
    Izz_w       =   param.Izz_w;                                            % [kg*m^2] z-axis moment of inertia of wheels
    Izz_t       =   param.Izz_t;                                            % [kg*m^2] z-axis moment of inertia of total chassis
    Ixx_uf      =   param.Ixx_uf;                                           % [kg*m^2] x-axis moment of inertia of front unsprung mass
    Ixx_ur      =   param.Ixx_ur;                                           % [kg*m^2] x-axis moment of inertia of rear unsprung mass
    Ixx_s       =   param.Ixx_s;                                            % [kg*m^2] x-axis moment of inertia of sprung mass
    Ixz_s       =   param.Ixz_s;                                            % [kg*m^2] xz product inertia of sprung mass
    Iyy_s       =   param.Iyy_s;                                            % [kg*m^2] y-axis moment of inertia of sprung mass
    mu0         =   param.mu0;                                              % [-] maximum friction scaling coefficient
    As          =   param.As;                                               % [-] friction reduction factor
    c_s         =   param.c_s;                                              % [N] tire longitudinal stiffness
    c_alpha     =   param.c_alpha;                                          % [N/rad] tire lateral stiffness
    Lrelx       =   param.Lrelx;                                            % [m] longitudinal relaxation length
    Lrely       =   param.Lrely;                                            % [m] lateral relaxation length
    k_lt        =   param.k_lt;                                             % [-] lateral compliance rate of tire and suspension
    k_scf       =   param.k_scf;                                            % [rad/N*m] steering compliance for front steering gear
    k_scv       =   param.k_scv;                                            % [rad*s/N*m] damping for front steering gear
    k_zt        =   param.k_zt;                                             % [N/m] vertical spring rate of tire
    k_zd        =   param.k_zd;                                             % [N*s/m] vertical damping of tire

    k_sf        =   param.k_sf;
    k_sdf       =   param.k_sdf;
    k_tsf       =   param.k_tsf;
    tw_f        =   param.tw_f;
    l_saf       =   param.l_saf;
    k_slf       =   param.k_slf;
    k_sadf      =   param.k_sadf;
    k_sad2f     =   param.k_sad2f;
    k_sr        =   param.k_sr;
    k_sdr       =   param.k_sdr;
    k_tsr       =   param.k_tsr;
    tw_r         =   param.tw_r;
    l_sar       =   param.l_sar ;
    k_slr       =   param.k_slr;
    k_sadr      =   param.k_sadr;
    k_sad2r     =   param.k_sad2r;

    h_bs        =   param.h_bs;
    k_bs        =   param.k_bs;

    k_ras       =   param.k_ras;
    k_rad       =   param.k_rad;

    % other parameters
    s_max       =   param.s_max;                                            % [-] upper bound of slip ratio
    s_min       =   param.s_min;                                            % [-] lower bound of slip ratio
    alpha_max   =   param.alpha_max;                                        % [rad] upper bound of slip angle
    alpha_min   =   param.alpha_min;                                        % [rad] lower bound of slip angle
    kappa_min   =   0;                                                      % [-] lower bound of non-dimensional tire force parameter
    kappa_max   =   1e4;                                                    % [-] upper bound of non-dimensional tire force parameter

    % motor and servo parameters (from references, guessed)
    Vs          =   param.Vs;                                               % [V] supply voltage
    Kt_q        =   param.Kt_q;                                             % [N*m/A] motor torque constant (total)
    Kb_q        =   param.Kb_q;                                             % [V*s/rad] motor back emf constant (total)
    R_phi       =   param.R_phi;                                            % [Ohm] resistance of BLDC motor (per-phase)
    Jm          =   param.Jm;                                               % [kg*m^2] rotor inertia
    Le          =   param.Le;                                               % [H] effective inductance
    b           =   param.b;                                                % [N*m*s] viscous friction coefficient
    gratio      =   param.gratio;                                           % [-] gear ratio between motor and wheel shafts
    gain        =   0.08;                                                   % [-] torque distribution coefficients through transmission
    delta_max       =   param.delta_max;                                    % [rad] maximum angle of steering servo
    deltadot_max    =   param.deltadot_max;                                 % [rad/s] maximum angular rate of steering servo

    %.. emi parameters
    nu0         =   param.nu0;                                              % [T*m/A] vacuum permeability
    wire_dir    =   param.wire_dir;                                         % [-] current flowing wire direction
    x_wire      =   param.x_wire;                                           % [m] relative position from current-flowing wire
    dist_wire   =   param.dist_wire;                                        % [m] given distance from wire
    B_earth0    =   param.B_earth0;                                         % [T] mean magnetic field strength at the magnetic equator on the Earth's surface
    lat0        =   param.lat0;                                             % [rad] latitude of the rover                      

    %.. assign the states and inputs
    x_t                 =   x(1);                                           % [m] chassis x-position
    u_t                 =   x(2);                                           % [m/s] chassis body x-velocity
    psi_t               =   x(3);                                           % [rad] chassis yaw angle
    r_t                 =   x(4);                                           % [rad/s] chassis yaw rate
    y_u                 =   x(5);                                           % [m] unsprung mass y-position (longitudinal c.g. of chassis, road plane = ned)
    v_u                 =   x(6);                                           % [m/s] unsprung mass road plane y-velocity
    delta_y_suf         =   x(7);                                           % [m] y-direction deflection between front unsprung mass and sprung mass
    delta_v_suf         =   x(8);                                           % [m/s] y-direction velocity between front unsprung mass and sprung mass
    delta_y_sur         =   x(9);                                           % [m] y-direction deflection between rear unsprung mass and sprung mass
    delta_v_sur         =   x(10);                                          % [m/s] y-direction velocity between rear unsprung mass and sprung mass
    z_uf                =   x(11);                                          % [m] front unsprung mass road plane z-position
    w_uf                =   x(12);                                          % [m/s] front unsprung mass road plane z-velocity
    z_ur                =   x(13);                                          % [m] rear unsprung mass road plane z-position
    w_ur                =   x(14);                                          % [m/s] rear unsprung mass road plane z-velocity
    z_s                 =   x(15);                                          % [m] sprung mass road plane z-position
    w_s                 =   x(16);                                          % [m/s] sprung mass road plane z-velocity
    phi_uf              =   x(17);                                          % [rad] front unsprung mass roll angle
    p_uf                =   x(18);                                          % [rad/s] front unsprung mass roll rate
    phi_ur              =   x(19);                                          % [rad] rear unsprung mass roll angle
    p_ur                =   x(20);                                          % [rad/s] rear unsprung mass roll rate
    phi_s               =   x(21);                                          % [rad] sprung mass roll angle
    p_s                 =   x(22);                                          % [rad/s] sprung mass roll rate
    theta_s             =   x(23);                                          % [rad] sprung mass pitch angle
    q_s                 =   x(24);                                          % [rad/s] sprung mass pitch rate
    omega_fl            =   x(25);                                          % [rad/s] rotational speed of front left wheel
    omega_fr            =   x(26);                                          % [rad/s] rotational speed of front right wheel
    omega_rl            =   x(27);                                          % [rad/s] rotational speed of rear left wheel
    omega_rr            =   x(28);                                          % [rad/s] rotational speed of rear right wheel
    Iq                  =   x(29);                                          % [A] motor current (d-q transformation)
    omega               =   x(30);                                          % [rad/s] motor shaft rotational speed
    s_fl                =   x(31);                                          % [-] longitudinal slip ratio
    s_fr                =   x(32);                                          % [-] longitudinal slip ratio
    s_rl                =   x(33);                                          % [-] longitudinal slip ratio
    s_rr                =   x(34);                                          % [-] longitudinal slip ratio
    alpha_fl            =   x(35);                                          % [-] lateral slip ratio
    alpha_fr            =   x(36);                                          % [-] lateral slip ratio
    alpha_rl            =   x(37);                                          % [-] lateral slip ratio
    alpha_rr            =   x(38);                                          % [-] lateral slip ratio
    delta_f             =   x(39);                                          % [-] forward steering angle
    deltadot_f          =   x(40);                                          % [-] forward steering rate

    %.. assign variables (ad-hoc)
    phiddot_s_pre       =   0;
    psiddot_pre         =   0;

    %.. assign empty vector for derivatives
    xdot        =   zeros(size(x,1),1);
    
    %.. update motor dynamics
    Vq                  =   sqrt(3/2)*Vs*max([min([u(1),1]),1/(sqrt(3/2)*Vs)*(R_phi*b/Kt_q+Kb_q)*(0.001/rw*gratio)]);
    xdot(29)            =   (Vq-R_phi*Iq-Kb_q*omega)/Le;
    xdot(30)            =   (Kt_q*Iq-b*omega-gain*((omega/gratio-omega_fl)+(omega/gratio-omega_fr)+(omega/gratio-omega_rl)+(omega/gratio-omega_rr)))/Jm;

    %.. update steering/tire dynamics and force model
    xdot(39)            =   deltadot_f;
    xdot(40)            =   1/(2*Izz_w*k_scf)*((u(2)-delta_f)-k_scv*deltadot_f);
    
    % if (x(39) >= delta_max) || (x(39) <= -delta_max)
    %     xdot(39)    =   0;
    %     xdot(40)    =   -k_scv*deltadot_f;
    % end
    % 
    % if (x(39) >= delta_max) || (x(39) <= -delta_max)
    %     xdot(39)    =   0;
    %     xdot(40)    =   -k_scv*deltadot_f;
    % end

    delta_fl            =   atan(tan(delta_f)/(1+tw/(2*lt)*tan(delta_f)));  % [rad] steering angle of front left wheel
    delta_fr            =   atan(tan(delta_f)/(1-tw/(2*lt)*tan(delta_f)));  % [rad] steering angle of front right wheel
    delta_rl            =   0.0;                                            % [rad] steering angle of rear left wheel
    delta_rr            =   0.0;                                            % [rad] steering angle of rear right wheel

    Fz_LF               =   m_s*g*lr/(2*lt)+m_uf*g/2+(z_uf+rw*(cos(phi_uf)-1)-tw/2*sin(phi_uf))*k_zt+(w_uf-rw*(sin(phi_uf)*p_uf)-tw/2*cos(phi_uf)*p_uf)*k_zd;   % [N] front left wheel vertical force
    Fz_RF               =   m_s*g*lr/(2*lt)+m_uf*g/2+(z_uf+rw*(cos(phi_uf)-1)+tw/2*sin(phi_uf))*k_zt+(w_uf-rw*(sin(phi_uf)*p_uf)+tw/2*cos(phi_uf)*p_uf)*k_zd;   % [N] front right wheel vertical force
    Fz_LR               =   m_s*g*lf/(2*lt)+m_ur*g/2+(z_ur+rw*(cos(phi_ur)-1)-tw/2*sin(phi_ur))*k_zt+(w_ur-rw*(sin(phi_uf)*p_uf)-tw/2*cos(phi_uf)*p_uf)*k_zd;   % [N] rear left wheel vertical force
    Fz_RR               =   m_s*g*lf/(2*lt)+m_ur*g/2+(z_ur+rw*(cos(phi_ur)-1)+tw/2*sin(phi_ur))*k_zt+(w_ur-rw*(sin(phi_uf)*p_uf)+tw/2*cos(phi_uf)*p_uf)*k_zd;   % [N] rear right wheel vertical force

    if (Fz_LF <= 0)
        Fz_LF   =   0;
    end
    if (Fz_RF <= 0)
        Fz_RF   =   0;
    end
    if (Fz_LR <= 0)
        Fz_LR   =   0;
    end
    if (Fz_RR <= 0)
        Fz_RR   =   0;
    end

    u_fl    =   cos(delta_fl)*(u_t+tw/2*r_t)+sin(delta_fl)*(v_u+lf*r_t-p_uf*(rw-z_uf));    % [m/s] front left wheel plane velocity (x-direction)
    v_fl    =   -sin(delta_fl)*(u_t+tw/2*r_t)+cos(delta_fl)*(v_u+lf*r_t-p_uf*(rw-z_uf));   % [m/s] front left wheel plane velocity (y-direction)
    u_fr    =   cos(delta_fr)*(u_t-tw/2*r_t)+sin(delta_fr)*(v_u+lf*r_t-p_uf*(rw-z_uf));    % [m/s] front right wheel plane velocity (x-direction)
    v_fr    =   -sin(delta_fr)*(u_t-tw/2*r_t)+cos(delta_fr)*(v_u+lf*r_t-p_uf*(rw-z_uf));   % [m/s] front right wheel plane velocity (y-direction)
    u_rl    =   cos(delta_rl)*(u_t+tw/2*r_t)+sin(delta_rl)*(v_u-lr*r_t-p_ur*(rw-z_ur));    % [m/s] rear left wheel plane velocity (x-direction)
    v_rl    =   -sin(delta_rl)*(u_t+tw/2*r_t)+cos(delta_rl)*(v_u-lr*r_t-p_ur*(rw-z_ur));   % [m/s] rear left wheel plane velocity (y-direction)
    u_rr    =   cos(delta_rr)*(u_t-tw/2*r_t)+sin(delta_rr)*(v_u-lr*r_t-p_ur*(rw-z_ur));    % [m/s] rear right wheel plane velocity (x-direction)
    v_rr    =   -sin(delta_rr)*(u_t-tw/2*r_t)+cos(delta_rr)*(v_u-lr*r_t-p_ur*(rw-z_ur));   % [m/s] rear right wheel plane velocity (y-direction)

    vs_fl   =   u_t*sqrt(s_fl^2+tan(alpha_fl)^2);                           % [-] friction reduction magnitude (approximated to use body x-velocity)
    vs_fr   =   u_t*sqrt(s_fr^2+tan(alpha_fr)^2);
    vs_rl   =   u_t*sqrt(s_rl^2+tan(alpha_rl)^2);
    vs_rr   =   u_t*sqrt(s_rr^2+tan(alpha_rr)^2);
                                      
    mu_fl   =   max([mu0*(1-As*vs_fl),0]);                                  % [-] maximum friction coefficient
    mu_fr   =   max([mu0*(1-As*vs_fr),0]);
    mu_rl   =   max([mu0*(1-As*vs_rl),0]);
    mu_rr   =   max([mu0*(1-As*vs_rr),0]);

    kappa_fl    =   min([max([mu_fl*Fz_LF*(1-s_fl)/(2*sqrt((c_s*s_fl)^2+(c_alpha*tan(alpha_fl))^2)+eps),kappa_min]),kappa_max]);
    kappa_fr    =   min([max([mu_fr*Fz_RF*(1-s_fr)/(2*sqrt((c_s*s_fr)^2+(c_alpha*tan(alpha_fr))^2)+eps),kappa_min]),kappa_max]);
    kappa_rl    =   min([max([mu_rl*Fz_LR*(1-s_rl)/(2*sqrt((c_s*s_rl)^2+(c_alpha*tan(alpha_rl))^2)+eps),kappa_min]),kappa_max]);
    kappa_rr    =   min([max([mu_rr*Fz_RR*(1-s_rr)/(2*sqrt((c_s*s_rr)^2+(c_alpha*tan(alpha_rr))^2)+eps),kappa_min]),kappa_max]);
    
    if kappa_fl < 1
        fkappa_fl   =   kappa_fl*(2-kappa_fl);
    else
        fkappa_fl   =   1;
    end
    if kappa_fr < 1
        fkappa_fr   =   kappa_fr*(2-kappa_fr);
    else
        fkappa_fr   =   1;
    end
    if kappa_rl < 1
        fkappa_rl   =   kappa_rl*(2-kappa_rl);
    else
        fkappa_rl   =   1;
    end
    if kappa_rr < 1
        fkappa_rr   =   kappa_rr*(2-kappa_rr);
    else
        fkappa_rr   =   1;
    end

    Fx_LF   =   c_s*s_fl/(1-s_fl)*fkappa_fl;
    Fx_RF   =   c_s*s_fr/(1-s_fr)*fkappa_fr;
    Fx_LR   =   c_s*s_rl/(1-s_rl)*fkappa_rl;
    Fx_RR   =   c_s*s_rr/(1-s_rr)*fkappa_rr;

    Fy_LF   =   c_alpha*tan(alpha_fl)/(1-s_fl)*fkappa_fl;
    Fy_RF   =   c_alpha*tan(alpha_fr)/(1-s_fr)*fkappa_fr;
    Fy_LR   =   c_alpha*tan(alpha_rl)/(1-s_rl)*fkappa_rl;
    Fy_RR   =   c_alpha*tan(alpha_rr)/(1-s_rr)*fkappa_rr;

    xdot(31)    =   -abs(u_fl)/min([max([tanh(abs(u_t)),0.001]),1.0])/Lrelx*s_fl+(rw*omega_fl-u_fl)/Lrelx;
    xdot(32)    =   -abs(u_fr)/min([max([tanh(abs(u_t)),0.001]),1.0])/Lrelx*s_fr+(rw*omega_fr-u_fr)/Lrelx;
    xdot(33)    =   -abs(u_rl)/min([max([tanh(abs(u_t)),0.001]),1.0])/Lrelx*s_rl+(rw*omega_rl-u_rl)/Lrelx;
    xdot(34)    =   -abs(u_rr)/min([max([tanh(abs(u_t)),0.001]),1.0])/Lrelx*s_rr+(rw*omega_rr-u_rr)/Lrelx;

    if (x(31) >= s_max) || (x(31) <= s_min)
        xdot(31)    =   0;
    end
    if (x(32) >= s_max) || (x(32) <= s_min)
        xdot(32)    =   0;
    end
    if (x(33) >= s_max) || (x(33) <= s_min)
        xdot(33)    =   0;
    end
    if (x(34) >= s_max) || (x(34) <= s_min)
        xdot(34)    =   0;
    end

    xdot(35)    =   -abs(u_fl)*tan(alpha_fl)/Lrely-v_fl/Lrely;
    xdot(36)    =   -abs(u_fr)*tan(alpha_fr)/Lrely-v_fr/Lrely;
    xdot(37)    =   -abs(u_rl)*tan(alpha_rl)/Lrely-v_rl/Lrely;
    xdot(38)    =   -abs(u_rr)*tan(alpha_rr)/Lrely-v_rr/Lrely;

    if (x(35) >= alpha_max) || (x(35) <= alpha_min)
        xdot(35)    =   0;
    end
    if (x(36) >= alpha_max) || (x(36) <= alpha_min)
        xdot(35)    =   0;
    end
    if (x(37) >= alpha_max) || (x(37) <= alpha_min)
        xdot(37)    =   0;
    end
    if (x(38) >= alpha_max) || (x(38) <= alpha_min)
        xdot(38)    =   0;
    end

    xdot(25)    =   1/Iyy_w*(gain*(omega/gratio-omega_fl)-rw*Fx_LF);
    xdot(26)    =   1/Iyy_w*(gain*(omega/gratio-omega_fr)-rw*Fx_RF);
    xdot(27)    =   1/Iyy_w*(gain*(omega/gratio-omega_rl)-rw*Fx_LR);
    xdot(28)    =   1/Iyy_w*(gain*(omega/gratio-omega_rr)-rw*Fx_RR);

    %.. update suspension dynamics and force model
    z_SLF       =   (hs-rw+z_uf-z_s)/cos(phi_s)-hs+rw+lf*theta_s+(phi_s-phi_uf)*(tw/2);
    z_SRF       =   (hs-rw+z_uf-z_s)/cos(phi_s)-hs+rw+lf*theta_s-(phi_s-phi_uf)*(tw/2);
    z_SLR       =   (hs-rw+z_ur-z_s)/cos(phi_s)-hs+rw-lr*theta_s+(phi_s-phi_ur)*(tw/2);
    z_SRR       =   (hs-rw+z_ur-z_s)/cos(phi_s)-hs+rw-lr*theta_s-(phi_s-phi_ur)*(tw/2);

    zdot_SLF    =   w_uf-w_s+lf*q_s+(p_s-p_uf)*(tw/2);
    zdot_SRF    =   w_uf-w_s+lf*q_s-(p_s-p_uf)*(tw/2);
    zdot_SLR    =   w_ur-w_s-lr*q_s+(p_s-p_ur)*(tw/2);
    zdot_SRR    =   w_ur-w_s-lr*q_s-(p_s-p_ur)*(tw/2);

    if abs(z_SLF) <= h_bs
        F_BSLF  =   0;
    else
        F_BSLF  =   (-z_SLF+sign(z_SLF)*h_bs)*k_bs;
    end

    if abs(z_SRF) <= h_bs
        F_BSRF  =   0;
    else
        F_BSRF  =   (-z_SRF+sign(z_SRF)*h_bs)*k_bs;
    end

    if abs(z_SLR) <= h_bs
        F_BSLR  =   0;
    else
        F_BSLR  =   (-z_SLR+sign(z_SLR)*h_bs)*k_bs;
    end

    if abs(z_SRR) <= h_bs
        F_BSRR  =   0;
    else
        F_BSRR  =   (-z_SRR+sign(z_SRR)*h_bs)*k_bs;
    end

    F_SQLF      =   (k_slf+z_SLF/l_saf)*(Fy_LF*cos(phi_s)-Fz_LF*sin(phi_s))-(min(Fx_LF,0)*k_sadf+max(Fx_LF,0)*k_sad2f);
    F_SQRF      =   (k_slf+z_SRF/l_saf)*(-Fy_RF*cos(phi_s)+Fz_RF*sin(phi_s))-(min(Fx_RF,0)*k_sadf+max(Fx_RF,0)*k_sad2f);
    F_SQLR      =   (k_slr+z_SLR/l_sar)*(Fy_LR*cos(phi_s)-Fz_LR*sin(phi_s))+(min(Fx_LR,0)*k_sadr+max(Fx_LR,0)*k_sad2r);
    F_SQRR      =   (k_slr+z_SRR/l_sar)*(-Fy_RR*cos(phi_s)+Fz_RR*sin(phi_s))+(min(Fx_RR,0)*k_sadr+max(Fx_RR,0)*k_sad2r);

    F_SLF       =   m_s*g*lr/(2*lt)-z_SLF*k_sf-zdot_SLF*k_sdf+(phi_s-phi_uf)*k_tsf/tw_f+F_BSLF+F_SQLF;
    F_SRF       =   m_s*g*lr/(2*lt)-z_SRF*k_sf-zdot_SRF*k_sdf-(phi_s-phi_uf)*k_tsf/tw_f+F_BSRF+F_SQRF;
    F_SLR       =   m_s*g*lf/(2*lt)-z_SLR*k_sr-zdot_SLR*k_sdr+(phi_s-phi_ur)*k_tsr/tw_r+F_BSLR+F_SQLR;
    F_SRR       =   m_s*g*lf/(2*lt)-z_SRR*k_sr-zdot_SRR*k_sdr-(phi_s-phi_ur)*k_tsr/tw_r+F_BSRR+F_SQRR;

    delta_yf    =   ((hs-rw+z_uf-z_s)*tan(phi_s)-(delta_y_suf))*cos(phi_s)-(huf-rw)*sin((phi_s-phi_uf));
    deltadot_yf =   ((hs-rw+z_uf-z_s)*cos(phi_s)-(delta_y_suf)*sin(phi_s))*p_s+(w_uf-w_s)*sin(phi_s)-delta_v_suf*cos(phi_s)-(huf-rw)*cos((phi_s-phi_uf))*(p_s-p_uf);
    
    delta_yr    =   ((hs-rw+z_ur-z_s)*tan(phi_s)-(delta_y_sur))*cos(phi_s)-(hur-rw)*sin((phi_s-phi_ur));
    deltadot_yr =   ((hs-rw+z_ur-z_s)*cos(phi_s)-(delta_y_sur)*sin(phi_s))*p_s+(w_ur-w_s)*sin(phi_s)-delta_v_sur*cos(phi_s)-(hur-rw)*cos((phi_s-phi_uf))*(p_s-p_uf);

    F_RAF   =   delta_yf*k_ras+deltadot_yf*k_rad;
    F_RAR   =   delta_yr*k_ras+deltadot_yr*k_rad;

    %.. compute dynamics
    a_x                 =   1/m_t*( ...
                            +(Fx_LF*cos(delta_fl)-Fy_LF*sin(delta_fl))+(Fx_RF*cos(delta_fr)-Fy_RF*sin(delta_fr)) ...
                            +(Fx_LR*cos(delta_rl)-Fy_LR*sin(delta_rl))+(Fx_RR*cos(delta_rr)-Fy_RR*sin(delta_rr)));

    rdot                =   1/Izz_t*( ...
                            +tw/2*(Fx_LF*cos(delta_fl)-Fy_LF*sin(delta_fl))-tw/2*(Fx_RF*cos(delta_fr)-Fy_RF*sin(delta_fr)) ...
                            +tw/2*(Fx_LR*cos(delta_rl)-Fy_LR*sin(delta_rl))-tw/2*(Fx_RR*cos(delta_rr)-Fy_RR*sin(delta_rr))...
                            +lf*(Fx_LF*sin(delta_fl)+Fy_LF*cos(delta_fl))+lf*(Fx_RF*sin(delta_fr)+Fy_RF*cos(delta_fr)) ...
                            -lr*(Fx_LR*sin(delta_rl)+Fy_LR*cos(delta_rl))-lr*(Fx_RR*sin(delta_rr)+Fy_RR*cos(delta_rr)));
    psiddot             =   -Ixz_s/Izz_t*phiddot_s_pre+rdot;                    % [rad/s^2] adding yaw-roll coupling, but by previous data

    a_y_uf              =   1/m_uf*( ...
                            +(Fx_LF*sin(delta_fl)+Fy_LF*cos(delta_fl))+(Fx_RF*sin(delta_fr)+Fy_RF*cos(delta_fr)) ...
                            -F_RAF*cos(phi_s)-(F_SLF+F_SRF)*sin(phi_s));
    a_y_ur              =   1/m_ur*( ...
                            +(Fx_LR*sin(delta_rl)+Fy_LR*cos(delta_rl))+(Fx_RR*sin(delta_rr)+Fy_RR*cos(delta_rr)) ...
                            -F_RAR*cos(phi_s)-(F_SLR+F_SRR)*sin(phi_s));



    xdot(1)             =   u_t*cos(psi_t)-v_u*sin(psi_t);
    xdot(2)             =   v_u*r_t+a_x;

    xdot(3)             =   r_t;


    xdot(4)             =   psiddot;

    xdot(5)             =   u_t*sin(psi_t)+v_u*cos(psi_t);

    xdot(6)             =   -u_t*r_t+(m_uf*a_y_uf+m_ur*a_y_ur)/(m_uf+m_ur);

    a_y_s               =   1/m_s*( ...
                            +F_RAF*cos(phi_s)+F_RAR*cos(phi_s) ...
                            +(F_SLF+F_SRF)*sin(phi_s)+(F_SLR+F_SRR)*sin(phi_s));

    xdot(7)             =   delta_v_suf;

    xdot(8)             =   a_y_s+lf*psiddot-a_y_uf;

    xdot(9)             =   delta_v_sur;

    xdot(10)            =   a_y_s-lr*psiddot-a_y_ur;

    xdot(11)            =   w_uf;

    xdot(12)            =   g-1/m_uf*( ...
                            Fz_LF+Fz_RF+F_RAF*sin(phi_s)-(F_SLF+F_SRF)*cos(phi_s));

    xdot(13)            =   w_ur;

    xdot(14)            =   g-1/m_ur*( ...
                            Fz_LR+Fz_RR+F_RAR*sin(phi_s)-(F_SLR+F_SRR)*cos(phi_s));

    xdot(15)            =   w_s;

    xdot(16)            =   g-1/m_s*( ...
                            -(F_RAF+F_RAR)*sin(phi_s)+(F_SLF+F_SRF+F_SLR+F_SRR)*cos(phi_s));

    xdot(17)            =   p_uf;

    xdot(18)            =   1/Ixx_uf*( ...
                            +F_SRF*(tw/2)-F_SLF*(tw/2)-F_RAF*(huf-rw) ...
                            +Fz_LF*(rw*sin(phi_uf)+tw/2*cos(phi_uf)-k_lt*Fy_LF) ...
                            -Fz_RF*(-rw*sin(phi_uf)+tw/2*cos(phi_uf)+k_lt*Fy_RF) ...
                            -(Fx_LF*sin(delta_fl)+Fy_LF*cos(delta_fl))*(rw-z_uf) ...
                            -(Fx_RF*sin(delta_fr)+Fy_RF*cos(delta_fr))*(rw-z_uf));

    xdot(19)            =   p_ur;

    xdot(20)            =   1/Ixx_ur*( ...
                            +F_SRR*(tw/2)-F_SLR*(tw/2)-F_RAR*(hur-rw) ...
                            +Fz_LR*(rw*sin(phi_ur)+tw/2*cos(phi_ur)-k_lt*Fy_LR) ...
                            -Fz_RR*(-rw*sin(phi_ur)+tw/2*cos(phi_ur)+k_lt*Fy_RR) ...
                            -(Fx_LR*sin(delta_rl)+Fy_LR*cos(delta_rl))*(rw-z_ur) ...
                            -(Fx_RR*sin(delta_rr)+Fy_RR*cos(delta_rr))*(rw-z_ur));

    xdot(21)            =   p_s;

    pdot_s              =   1/10/Ixx_s*( ...
                            +F_SLF*(tw/2)+F_SLR*(tw/2)-F_SRF*(tw/2)-F_SRR*(tw/2) ...
                            -F_RAF/cos(phi_s)*(hs-z_s-rw+z_uf-(huf-rw)*cos(phi_uf)) ...
                            -F_RAR/cos(phi_s)*(hs-z_s-rw+z_ur-(hur-rw)*cos(phi_ur)));
    phiddot_s           =   -Ixz_s/Ixx_s*psiddot+pdot_s;
    xdot(22)            =   phiddot_s;

    xdot(23)            =   q_s;

    xdot(24)            =   1/Iyy_s*( ...
                            +lf*(F_SLF+F_SRF)-lr*(F_SLR+F_SRR) ...
                            +(Fx_LF*cos(delta_fl)-Fy_LF*sin(delta_fl) ...
                             +Fx_RF*cos(delta_fr)-Fy_RF*sin(delta_fr) ...
                             +Fx_LR*cos(delta_rl)-Fy_LR*sin(delta_rl) ...
                             +Fx_RR*cos(delta_rr)-Fy_RR*sin(delta_rr))*(hs-z_s));

    % %.. auxiliary information: ax, ay, rdot, pdot, Fz, Fx, Fy (tire forces)
    x_aux_p         =   zeros(size(x_aux,1),1);
    x_aux_p(1:4)    =   [Fz_LF; Fz_RF; Fz_LR; Fz_RR];
    x_aux_p(5:8)    =   [Fx_LF; Fx_RF; Fx_LR; Fx_RR];
    x_aux_p(9:12)   =   [Fy_LF; Fy_RF; Fy_LR; Fy_RR];



    % x_aux_p(1)      =   ax;
    % x_aux_p(2)      =   ay;
    % x_aux_p(3)      =   rdot;
    % x_aux_p(4)      =   pdot;



    % x_aux_p(17)     =   Vq;
    % 
    %.. emi field model
    Beta_wire       =   nu0*Iq/(2*pi)/dist_wire^2*cross(wire_dir,x_wire);
    Beta_wire_x     =   Beta_wire(1);
    Beta_wire_y     =   Beta_wire(2);
    Beta_wire_z     =   Beta_wire(3);

    Beta_earth      =   eul2rot([phi_s; 0; psi_t])'*...
                        [B_earth0*sin(pi/2-lat0); 0; -2*B_earth0*cos(pi/2-lat0)];
    Beta_earth_x    =   Beta_earth(1);
    Beta_earth_y    =   Beta_earth(2);
    Beta_earth_z    =   Beta_earth(3);

    % x_aux_p(18:26)  =   [Beta_motor_x; Beta_motor_y; Beta_motor_z; Beta_wire_x; Beta_wire_y; Beta_wire_z; Beta_earth_x; Beta_earth_y; Beta_earth_z]; 

end

function x_post = madgwick_AHRS(x_pre, gyro, acc, mag, dt)

    %.. compute quaternion rate
    qDot    =   0.5*quatmulti(x_pre,[0; gyro]);

    %.. normalize acceleration and magnetometer reading
    acc_norm    =   acc/norm(acc);
    mag_norm    =   mag/norm(mag);

    %.. earth frame magnetic field computation using previous quaternion
    hE_quat     =   quatmulti(quatmulti(x_pre,[0; mag_norm]),quatinv(x_pre));
    bE_quat     =   [0.0; sqrt(hE_quat(2)^2+hE_quat(3)^2); 0.0; hE_quat(4)];

    %.. compute gradient for magnetometer measurements
    fb          =   [2*bE_quat(2)*(1/2-x_pre(3)^2-x_pre(4)^2)+2*bE_quat(4)*(x_pre(2)*x_pre(4)-x_pre(1)*x_pre(3))-mag_norm(1);
                     2*bE_quat(2)*(x_pre(2)*x_pre(3)-x_pre(1)*x_pre(4))+2*bE_quat(4)*(x_pre(1)*x_pre(2)+x_pre(3)*x_pre(4))-mag_norm(2);
                     2*bE_quat(2)*(x_pre(1)*x_pre(3)+x_pre(2)*x_pre(4))+2*bE_quat(4)*(1/2-x_pre(2)^2-x_pre(3)^2)-mag_norm(3)];
    Jb          =   [-2*bE_quat(4)*x_pre(3)                         2*bE_quat(4)*x_pre(4)                           -4*bE_quat(2)*x_pre(3)-2*bE_quat(4)*x_pre(1)    -4*bE_quat(2)*x_pre(4)+2*bE_quat(4)*x_pre(2);
                     -2*bE_quat(2)*x_pre(4)+2*bE_quat(4)*x_pre(2)   2*bE_quat(2)*x_pre(3)+2*bE_quat(4)*x_pre(1)     2*bE_quat(2)*x_pre(2)+2*bE_quat(4)*x_pre(4)     -2*bE_quat(2)*x_pre(1)+2*bE_quat(4)*x_pre(3);
                     2*bE_quat(2)*x_pre(3)                          2*bE_quat(2)*x_pre(4)-4*bE_quat(4)*x_pre(2)     2*bE_quat(2)*x_pre(1)-4*bE_quat(4)*x_pre(3)     2*bE_quat(2)*x_pre(2)];
    
    %.. compute gradient for accelerometer measurements
    fg          =   [-2*(x_pre(2)*x_pre(4)-x_pre(1)*x_pre(3))-acc_norm(1);
                     -2*(x_pre(1)*x_pre(2)+x_pre(3)*x_pre(4))-acc_norm(2);
                     -2*(1/2-x_pre(2)^2-x_pre(3)^2)-acc_norm(3)];
    Jg          =   [2*x_pre(3)     -2*x_pre(4)     2*x_pre(1)      -2*x_pre(2);
                     -2*x_pre(2)    -2*x_pre(1)     -2*x_pre(4)     -2*x_pre(3);
                     0.0            4*x_pre(2)      4*x_pre(3)      0.0];

    %.. compute total gradient
    delf        =   [Jb' Jg']*[fb; fg];

    %.. update estimate
    if abs(delf) > eps
        x_post      =   x_pre+(qDot-0.1*delf/norm(delf))*dt;
    else
        x_post      =   x_pre+(qDot)*dt;
    end
    x_post      =   x_post/norm(x_post);

end
