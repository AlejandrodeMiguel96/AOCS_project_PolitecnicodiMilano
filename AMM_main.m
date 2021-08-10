%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                                                                     %%%
%%%             Final Project of Attitude Dynamics                      %%%
%%%                     Course 2019-2020                                %%%
%%%                 Alejandro de Miguel Mendiola                        %%%
%%%                                                                     %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clearvars
close all
random = 'Y'; % if initial w and DCM are wanted to be random --> random = 'Y'
              % otherwhise --> random = 'N'

% Data
disp('Loading data');


%% Orbit parameters
mu = 3.986004418e14; %[m^3/s^2]
h = 500; % Orbit height [km]
a = (6371+h)*1e3; %Semi major axis [m]
n = sqrt( mu / a^3 ); %is this the nº of revolutions per orbit?
ww_earth = 2*pi/(24*3600)*[0;0;1]; % Earth's angular speed [rad/s]
rho_air = 4.39e-16 * 1e6/1e3; %[kg/m3] rho density at constant altitude 500km
e0 = 0; %circular orbit [-]
i0 = 0*pi/180; %equatorial orbit [rad]
O0 = 0*pi/180; % RAAN [rad]
o0 = 0*pi/180; % argument of periapsis [rad]
th0 = 0*pi/180; %true anomaly [rad]



%% Spacecraft data
Jx = 134.53e-2; %[kg m^2]
Jy = 33.47e-2; %[kg m^2]
Jz = 122.13e-2; %[kg m^2]
J = [Jx;Jy;Jz];

Ax = 0.25*0.25; %[m^2]
Ay = 0.5*0.25; %[m^2]
Az = Ay; %[m^2]
A_sp = 0.2*0.44; % Solar panels area [m^2]
A_t = [Ax;Ay;Az;Ax;Ay;Az;A_sp;A_sp;A_sp;A_sp;]; % Total spacecraft area [m^2]

rho_sMB = 0.5; % Reflectivity coefficients [-]
rho_dMB = 0.1;
rho_sSP = 0.8;
rho_dSP = 0.1;
rho = [rho_sMB;rho_dMB;rho_sSP;rho_dSP];

cd = 2.2; % Drag coefficient typical value [-]
par_ind = [0.1;0.1;0.1]; % Parasitic induction

%% Sensors    
        % Magneto-Sensor:       [New Space Systems NHRM-001-485]
MS_err     = 2*pi/180;                                          % [rad]
if (random == 'Y')
    MS_dir     = rand(3,1) - 0.5;                                   % [-]
else
    MS_dir = [-0.144926348121151, 0.497003271606648, -0.275828501016873];
end
MS_freq    = 1;                                                 % [Hz]
MS_noise   = (8e-9)^2*MS_freq;                                  % [T^2/s]
    
    % Earth Horizon Sensor: [Maryland Aerospace MAI-SES (MAI-000-00000200)]
EH_err     = 0.25*pi/180;                                       % [rad]
if (random == 'Y')
    EH_dir     = rand(3,1) - 0.5;                                   % [-]
else
    EH_dir = [0.152451072968615, 0.104990641908259, -0.112754568516865];
end
    
%% Reaction Wheel
RW_max_torque = 100e-3; %hrdot[N m]
RW_max_momentum = 400e-3; %hr[N m s]
RW_Ir = 3.4e-4; %[kg m^2]
prop1 = 1; %proportion of RW orientation towards x-axis
prop2 = 1; %proportion of RW orientation towards y-axis
prop3 = 1; %proportion of RW orientation towards z-axis
A_RW = sqrt([prop1;prop2;prop3]/(prop1+prop2+prop3)); % RW unitary direction matrix

%% Magnetic torquer
max_mx = 1.19; %magnetic moment[Am^2]
max_my = 1.19;
max_mz = 1.19;

%% Gyro
% Gyro:                 [Sensonor STIM 210]
Ir = 1e-7; % Moment of inertia [kg m^2]
Iz = 2e-7; % Moment of inertia [kg m^2]
wr = 100*pi/180; % Ang.vel. [rad/s]
Gyr_frq    = 262;                                               % [Hz]
Gyr_ARW    = (0.15*pi/180)^2 /3600;                             % [rad^2/s]
Gyr_RRW    = (0.30*pi/180)^2 * Gyr_frq / 3600^2;                % [rad^2/s^3]

%% Initial conditions
if (random == 'Y')
    dir1 = rand(1,1); % x-direction of initial tumbling
    dir2 = rand(1,1); % y-direction of initial tumbling
    dir3 = rand(1,1); % z-direction of initial tumbling
    init_DCM  = RandOrthMat(3);% Initial DCM: random orientation -> random Orthogonal matrix with det>0 [-]
    while det(init_DCM)<0
    init_DCM  = RandOrthMat(3);
    end
else
    dir1 = 0.8142848; % x-direction of initial tumbling
    dir2 = 0.2435249; % y-direction of initial tumbling
    dir3 = 0.9292636; % z-direction of initial tumbling
    init_DCM = [-0.848585405339590  -0.297006852770869  -0.437823867841608;
                 0.095646676591186  -0.900040933789052   0.425179998072807;
                -0.520340775936026   0.318925143115552   0.792169319013947];
end
dir = sqrt([dir1;dir2;dir3]/(dir1+dir2+dir3)); % total direction of initial tumbling
init_tumbl = 10*pi/180*dir; % a 10 deg/s initial tumble perturbation
init_orb = [e0,i0,O0,o0,th0];
init_gyro = init_tumbl;
init_hr = 0;

%% Tuning parameters detumbling
k_RW = .1; %gyro detumbling tuning parameter
k_mt = 1e10; %magneto torque detumbling tuning parameter
alpha = 0.5 ; %gyro tuning parameter 

%% DETUMBLING PHASE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%
%%%         DETUMBLING PHASE
%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Starting Simulink models
disp('Loading detumbling model');
if not(bdIsLoaded('AMM_detumbling'))
    open('AMM_detumbling')
end
tf_detumbling = 80; % final time of detumbling simulation [s]
set_param('AMM_detumbling','StopTime',num2str(tf_detumbling))
% Required error for convergence (0.5 deg/sec to allow sensors working)
req_err = 0.5*pi/180; % [rad/s]
req_err = 0.5*req_err; % Factor of safety

disp('Simulating detumbling model');
tic
de_tumble = sim('AMM_detumbling');

while norm(de_tumble.w.Data(end,:)) > req_err
    tf_detumbling = tf_detumbling + 50;
    set_param('AMM_detumbling','StopTime',num2str(tf_detumbling))
    disp('Tf increased to:');
    disp(tf_detumbling);
    de_tumble = sim('AMM_detumbling');
end

det_extime = toc/60; % Detumbling execution time
disp('Detumbling simulation completed. Execution time:');
disp(det_extime)

%% Configuration for plots

% DEFAULT OPTIONS FOR GRAPHICAL REPRESENTATION     

% Legend interpreter
set(0,'defaulttextinterpreter','latex')

% Font size
set(0,'DefaultTextFontSize',14)
set(0,'DefaultAxesFontSize',14)

% Title interpreter
% set(0,'DefaultTextFontName','Times')
set(groot,'DefaultTextFontName','latex')

% Axes interpreter
% set(0,'DefaultAxesFontName','Times') 
set(0,'DefaultAxesFontName','latex') 
set(0,'defaultAxesTickLabelInterpreter','latex');

% Linewidth
set(0,'defaultlinelinewidth', 2)

% Marker size
% set(0,'defaultlinemarkersize',4);

% Legend  
set(0,'defaultLegendInterpreter','latex');  

%% PLOTS detumbling
% figure(1) % w
% hold on
% plot ( de_tumble.tout, de_tumble.w.Data(:,1), '-r')
% plot ( de_tumble.tout, de_tumble.w.Data(:,2), '-g')
% plot ( de_tumble.tout, de_tumble.w.Data(:,3), '-b')
% plot ( xlim, [0 0], '-k', 'Linewidth', 0.5,'HandleVisibility','off')
% hold off
% grid minor
% legend ('$\omega$(1)',  '$\omega$(2)',  '$\omega$(3)')
% xlabel ('Time [s]')
% ylabel('$\omega$ [rad/s]')
% 
% figure(2) %error w-w_gyro
% hold on
% plot ( de_tumble.tout, de_tumble.w.Data(:,1) - de_tumble.w_gyro.Data(:,1), '-r')
% plot ( de_tumble.tout, de_tumble.w.Data(:,2) - de_tumble.w_gyro.Data(:,2), '-g')
% plot ( de_tumble.tout, de_tumble.w.Data(:,3) - de_tumble.w_gyro.Data(:,3), '-b')
% plot ( xlim, [0 0], '-k', 'Linewidth', 0.5,'HandleVisibility','off')
% hold off
% grid minor
% legend ('err(1)',  'err(2)',  'err(3)')
% xlabel ('Time [s]')
% ylabel('$\omega$ - $\omega_{est}$ [rad/s]')
% 
% figure(3) %RW momentum
% hold on
% plot ( de_tumble.tout, de_tumble.hr.Data, '-b')
% plot ( xlim, [0 0], '-k', 'Linewidth', 0.5,'HandleVisibility','off')
% hold off
% grid minor
% xlabel ('Time [s]')
% ylabel('$h_r$ [Nms]')
% 
% figure(4) % MT m
% hold on
% plot ( de_tumble.tout, de_tumble.m.Data(:,1), '-r')
% plot ( de_tumble.tout, de_tumble.m.Data(:,2), '-g')
% plot ( de_tumble.tout, de_tumble.m.Data(:,3), '-b')
% plot ( xlim, [0 0], '-k', 'Linewidth', 0.5,'HandleVisibility','off')
% hold off
% grid minor
% legend ('m_x',  'm_y',  'm_z')
% xlabel ('Time [s]')
% ylabel('m [Am^2]')
% 
% figure(5) % control torque u
% hold on
% plot ( de_tumble.tout, de_tumble.u.Data(:,1), '-r')
% plot ( de_tumble.tout, de_tumble.u.Data(:,2), '-g')
% plot ( de_tumble.tout, de_tumble.u.Data(:,3), '-b')
% plot ( xlim, [0 0], '-k', 'Linewidth', 0.5,'HandleVisibility','off')
% hold off
% grid minor
% legend ('u(1)',  'u(2)',  'u(3)')
% xlabel ('Time [s]')
% ylabel('u [Nm]')
% 
% figure(6) % Perturbations
% hold on
% plot ( de_tumble.tout, de_tumble.Mpert.Data(:,1), '-r')
% plot ( de_tumble.tout, de_tumble.Mpert.Data(:,2), '-g')
% plot ( de_tumble.tout, de_tumble.Mpert.Data(:,3), '-b')
% plot ( xlim, [0 0], '-k', 'Linewidth', 0.5,'HandleVisibility','off')
% hold off
% grid minor
% legend ('M_pert(1)',  'M_pert(2)',  'M_pert(3)')
% xlabel ('Time [s]')
% ylabel('M_pert [Nm]')
% 
% %Save figures (comment before delivering)
% % saveas(figure(1),[pwd '/Report/images/det_w.png']);
% % saveas(figure(2),[pwd '/Report/images/det_err.png']);
% % saveas(figure(3),[pwd '/Report/images/det_RW_mom.png']);
% % saveas(figure(4),[pwd '/Report/images/det_MT_m.png']);
% % saveas(figure(5),[pwd '/Report/images/det_u.png']);
% % saveas(figure(6),[pwd '/Report/images/det_M_pert.png']);

%% Tuning parameters tracking
K1 = .05; %control tune parameter
K2 = 0.5*K1^2; %control tune parameter
alpha = 0.005; %gyro tuning parameter 

%% TRACKING PHASE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%
%%%         TRACKING PHASE
%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Loading tracking model');
if not(bdIsLoaded('AMM_tracking'))
    open('AMM_tracking')
end


t0 = de_tumble.tout(end); % initial time of tracking simulation [s]
tf_track = 350; % final time of tracking simulation [s]
set_param('AMM_tracking','StopTime',num2str(tf_track));
 
%load last tumbling conditions to tracking simulation
w0_track = de_tumble.w.Data(end,:);                          
w0_gyro_track = de_tumble.w_gyro.Data(end,:);                          
DCM0_track = de_tumble.A_BN.Data(:,:,end);                          
hr0_track = de_tumble.hr.Data(end,:); 
orb0_track = de_tumble.orb_elem.Data(end,:);                          
                          
                          
init_tumbl = w0_track;
init_gyro = w0_gyro_track;
init_DCM = DCM0_track;
init_orb = orb0_track;
init_hr = hr0_track;


disp('Simulating tracking model');
disp('It may take several minutes');
tic
tracking = sim('AMM_tracking');

disp('End of simulation');
track_extime = toc/60; % tracking execution time
disp('Tracking simulation completed. Execution time:');
disp(track_extime)

disp('End of simulations');

%% PLOTS tracking
% close all;
% figure (1) %w
% xlim([0,tracking.tout(end)])
% hold on
% plot ( tracking.tout, tracking.w.Data(:,1), '-r')
% plot ( tracking.tout, tracking.w.Data(:,2), '-g')
% plot ( tracking.tout, tracking.w.Data(:,3), '-b')
% plot ( xlim, [0 0], '-k', 'Linewidth', 0.5,'HandleVisibility','off')
% hold off
% grid minor
% legend ('$\omega$(1)',  '$\omega$(2)',  '$\omega$(3)')
% xlabel ('Time [s]')
% ylabel('$\omega$ [rad/s]')
% 
% figure (2) %w-w_est error
% xlim([0,tracking.tout(end)])
% hold on
% plot ( tracking.tout, tracking.w.Data(:,1)-tracking.w_gyro.Data(:,1), '-r')
% plot ( tracking.tout, tracking.w.Data(:,2)-tracking.w_gyro.Data(:,2), '-g')
% plot ( tracking.tout, tracking.w.Data(:,3)-tracking.w_gyro.Data(:,3), '-b')
% plot ( xlim, [0 0], '-k', 'Linewidth', 0.5,'HandleVisibility','off')
% hold off
% grid minor
% legend ('$\omega$(1)',  '$\omega$(2)',  '$\omega$(3)')
% xlabel ('Time [s]')
% ylabel('$\omega$ - $\omega_{est}$ [rad/s]')
% 
% figure(3) %pointing error
% xlim([0,tracking.tout(end)])
% hold on
% plot ( tracking.tout, tracking.pnt_err.Data, '-b')
% plot ( xlim, [0 0], '-k', 'Linewidth', 0.5,'HandleVisibility','off')
% hold off
% grid minor
% xlabel ('Time [s]')
% ylabel('Pointing error [deg]')
% 
% figure(4) %control torque error
% xlim([0,tracking.tout(end)])
% hold on
% plot ( tracking.tout, tracking.u_id.Data(:,1)-tracking.u.Data(:,1), '-r')
% plot ( tracking.tout, tracking.u_id.Data(:,2)-tracking.u.Data(:,2), '-g')
% plot ( tracking.tout, tracking.u_id.Data(:,3)-tracking.u.Data(:,3), '-b')
% plot ( xlim, [0 0], '-k', 'Linewidth', 0.5,'HandleVisibility','off')
% hold off
% grid minor
% legend ('$u_{err}$(1)',  '$u_{err}$(2)',  '$u_{err}$(3)')
% xlabel ('Time [s]')
% ylabel('$u_id$ - $u$ [Nm]')

% figure (5) % RW momentum
% xlim([0,tracking.tout(end)])
% hold on
% plot ( tracking.tout, tracking.hr.Data, '-r')
% plot ( xlim, [0 0], '-k', 'Linewidth', 0.5,'HandleVisibility','off')
% hold off
% grid minor
% legend ('$h_r$(1)',  '$h_r$(2)',  '$h_r$(3)')
% xlabel ('Time [s]')
% ylabel('$\h_r$ [Nms]')

% figure(6) % Perturbations
% hold on
% plot ( tracking.tout, tracking.Mpert.Data(:,1), '-r')
% plot ( tracking.tout, tracking.Mpert.Data(:,2), '-g')
% plot ( tracking.tout, tracking.Mpert.Data(:,3), '-b')
% plot ( xlim, [0 0], '-k', 'Linewidth', 0.5,'HandleVisibility','off')
% hold off
% grid minor
% legend ('M_pert(1)',  'M_pert(2)',  'M_pert(3)')
% xlabel ('Time [s]')
% ylabel('$M_{pert}$ [Nm]')

%% FUNCTIONS
function M=RandOrthMat(n, tol)
% M = RANDORTHMAT(n)
% generates a random n x n orthogonal real matrix.
%
% M = RANDORTHMAT(n,tol)
% explicitly specifies a thresh value that measures linear dependence
% of a newly formed column with the existing columns. Defaults to 1e-6.
%
% In this version the generated matrix distribution *is* uniform over the manifold
% O(n) w.r.t. the induced R^(n^2) Lebesgue measure, at a slight computational 
% overhead (randn + normalization, as opposed to rand ). 
% 
% (c) Ofek Shilon , 2006.


    if nargin==1
	  tol=1e-6;
    end
    
    M = zeros(n); % prealloc
    
    % gram-schmidt on random column vectors
    
    vi = randn(n,1);  
    % the n-dimensional normal distribution has spherical symmetry, which implies
    % that after normalization the drawn vectors would be uniformly distributed on the
    % n-dimensional unit sphere.

    M(:,1) = vi ./ norm(vi);
    
    for i=2:n
	  nrm = 0;
	  while nrm<tol
		vi = randn(n,1);
		vi = vi -  M(:,1:i-1)  * ( M(:,1:i-1).' * vi )  ;
		nrm = norm(vi);
	  end
	  M(:,i) = vi ./ nrm;

    end %i
        
end  % RandOrthMat
    






