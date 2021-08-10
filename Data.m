%% Data for Attitude final project
clearvars;

%% Orbit parameters
mu = 3.986004418e14;
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

rho_sMB = 0.5;
rho_dMB = 0.1;
rho_sSP = 0.8;
rho_dSP = 0.1;
rho = [rho_sMB;rho_dMB;rho_sSP;rho_dSP];

cd = 2.2; % Drag coefficient typical value [-]


%% Sensors
% horiz_err = 0.1; % Earth horizon sensor error[deg]
horiz_err = 0.25*pi/180; % Earth horizon sensor error[rad]
% magn_err = 5; % Magnetometer sensor error [deg]
magn_err = 2*pi/180; % Magnetometer sensor error [rad]

% CAMBIAR!!!!!!!!!!!!!!!!!!
% horiz_ang_err= [1;0.5;2.75]*pi/180; % 3 angular measurement errors from the Earth horizon sensor [rad]
horiz_ang_err = [0.13236,-0.40246,-0.2215];%3 angular measurement errors from the Earth horizon sensor [rad]
% magn_ang_err= [2;1.5;3.75]*pi/180; % 3 angular measurement errors from the magnetometer sensor [rad]
magn_ang_err= [0.40579,-0.37301,0.41338];% 3 angular measurement errors from the magnetometer sensor [rad]



% Magneto-Sensor:       [New Space Systems NHRM-001-485]
    magn_err     = 2*pi/180; % [rad]
    magn_ang_err = rand(3,1) - 0.5; % [-]
    MS_freq    = 1; % [Hz]
    MS_noise   = (8e-9)^2*MS_freq; % [T^2/s]
    
    % Earth Horizon Sensor: [Maryland Aerospace MAI-SES (MAI-000-00000200)]
    horiz_err     = 0.25*pi/180;                                    % [rad]
    horiz_ang_err     = rand(3,1) - 0.5;                            % [-]
    
%% Reaction Wheel
% RW_max_torque = 8e-3; %hrdot[N m]
% RW_max_momentum = 50e-3; %hr[N m s]
% RW_Ir = 0.3; %[kg m^2]
RW_max_torque = 100e-3; %hrdot[N m]
RW_max_momentum = 400e-3; %hr[N m s]
RW_Ir = 3.4e-4; %[kg m^2]
% A = [1/sqrt(3);1/sqrt(3);1/sqrt(3)]; % one single Reaction wheel equal configuration
prop1 = 1; %proportion of RW orientation towards x-axis
prop2 = 1; %proportion of RW orientation towards y-axis
prop3 = 1; %proportion of RW orientation towards z-axis
A_RW = sqrt([prop1;prop2;prop3]/(prop1+prop2+prop3)); % RW unitary direction matrix

%% Magnetic torquer
max_mx = 1.19; %magnetic moment[Am^2]
max_my = 1.19;
max_mz = 1.19;


%% Gyro
Ir = 1e-7; % Moment of inertia [kg m^2]
Iz = 2e-7; % Moment of inertia [kg m^2]
wr = 100*pi/180; % Ang.vel. [rad/s] SE CAMBIA???????

%% Init.cond and tuning parameters
K1 = .05; %control tune parameter
K2 = 0.5*K1^2; %control tune parameter
alpha = 0.5; %gyroscope tuning parameter 
k_RW = 1; %gyro detumbling tuning parameter
k_mt = 1e6; %magneto torque detumbling tuning parameter

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
init_orb = [e0,i0,O0,o0,th0];
dir1 = 0.8; % x-direction of initial tumbling
dir2 = 0.14; % y-direction of initial tumbling
dir3 = 0.4; % z-direction of initial tumbling
dir = sqrt([dir1;dir2;dir3]/(dir1+dir2+dir3)); % total direction of initial tumbling
init_tumbl = 10*pi/180*dir; % a 10 deg/s initial tumble perturbation
% init_DCM = [-0.84798, -0.5296, 0.021352;
%     -0.25697, 0.44601, 0.85734;
%     -0.46358, 0.72152, -0.5143 ];
init_DCM = [0.097540404999410   0.957506835434298   0.970592781760616;
   0.278498218867048   0.964888535199277   0.957166948242946;
   0.546881519204984   0.157613081677548   0.485375648722841];
init_RW_cond = 0;
% A_d = eye(3);
