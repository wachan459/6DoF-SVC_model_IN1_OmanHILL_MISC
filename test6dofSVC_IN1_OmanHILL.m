% This file is part of 6DoF-SVC_IN model with MISC output
% Copyright (c) 2026 Takahiro Wada
% Released under the MIT License.
%
% 2026, Nov.23
% Test program for 6DoF-SVC-IN1-OmanHILL model (ssixdofSVC_In1_OmanHILL.m).
% Sinusoidal vertical movement is used.
%
%[1] Inoue, S., Dang, V.T., Liu, H. et al. Construction of a computational model of individual progression 
% of motion sickness symptoms based on subjective vertical conflict theory. Exp Brain Res 243, 108 (2025). 
% https://doi.org/10.1007/s00221-025-07052-5
%[2] V. T. Dang, M. Bourseul and T. Wada, "Prediction of Individual Motion Sickness of Bus Passengers Using 
% a 6DoF-SVC Model Driven by Handheld Tablet Motion During Visual Tasks," in IEEE Transactions on Intelligent 
% Transportation Systems, vol. 27, no. 10, pp. 13461-13472, Oct. 2026, doi: 10.1109/TITS.2026.3710322.


clc
clear

DEG2RAD=3.141592/180.0;
Dt=0.01;
Fs=floor(1/Dt);
FTIME=7200; %simulation time 

MABIKI = 10;
NTIME = 30*60*Fs;
fileID = fopen('MSIfile.dat','w');

%Parameters used in 
%[1] Inoue, S., Liu, H., Wada, T. Revisiting Motion Sickness Models Based on SVC Theory Considering Motion Perception. 
% SAE Technical Paper, 2023
%[2] T. Wada and J. Bos, Theoretical Considerations on Models of Vestibular Self-motion Perception 
%    as Inherent in Computational Frameworks of Motion Sickness, Biological Cybernetics, 2025

    param = setSVCIN1OmanHILLParam();


%omega_ts = timeseries(omega, t);
%a_ts = timeseries(a, t);

t = [0:Dt:FTIME]';       % Time data
n_time = length(t);

fmat=zeros(3,n_time);
omegamat=zeros(3,n_time);
MSIarray = zeros(n_time,1); %for File output
tarray = zeros(n_time,1);

% Longitudinal vibration
Amp_RMSG=0.234 %G
freq=0.2 %Hz

for i=1:1:n_time
    fmat(:,i) = [0.0, 0.0, 9.81+9.81*sqrt(2)*Amp_RMSG*sin(2*pi*freq*t(i))]; 
    omegamat(:,i) = [0,0,0];
end

uinput = [omegamat; fmat]';

fdummy=zeros(3,1);
fdummy(3)=9.81; %for simulation in vertical direction
fdummy = fdummy/norm(fdummy)*9.81;

%Preparation of initial value of state variable x
% set tentative initial value of state variable x
  xscc=[0;0;0]; %1-3
  v_s=fdummy;   %4-6
  hv_s = fdummy;    %7-9
  x_hscc = [0;0;0]; %10-12
  x_vc = fdummy/param.Kvc; %13-15
  g = fdummy;  %16-18
  uf = 0; %19
  dotuf = 0; %20
  us = 0; %21
  dotus = 0; %22

  x=[xscc;
 v_s;
 hv_s;
 x_hscc;
 x_vc;
 g;
 uf;
 dotuf;
 us;
 dotus];


uinputprepare = [ones(NTIME,1)*0.0,ones(NTIME,1)*0.0,ones(NTIME,1)*0.0,ones(NTIME,1)*fdummy(1),ones(NTIME,1)*fdummy(2),ones(NTIME,1)*fdummy(3)];

xdumarray=zeros(22,n_time);
xdumarray(:,1) =x;
deltaarray=ones(n_time,1);

%Calculation loop to determine initial value of state value x
for count = 1:NTIME;

	nextx = sixdofSVC_IN1_OmanHILL(x, uinputprepare(count,:), Dt, param);
    deltaarray(count)=norm(x-nextx);
    x = nextx;
    xdumarray(:,count)=nextx;

end

epsilon=10.0^(-6);
shushokugosa=deltaarray(count)
if(deltaarray(count)<=epsilon)
    disp('The convergence of the state variable initialization calculation was successful.');
else
    warning("The calculation for state variable initialization has not converged.");
end

x(16:18) = fdummy; %g(0) = fdummy16-18
x(19)=0.0; % uf=0
x(20)=0.0; % dofuf=0
x(21)=0.0; % us = 0;
x(22)=0.0; % dotus =0;

% At this point, the initial values of state variables x(0) to x(25) have been determined. 

% From here on, MSI calculations will be performed using the data.
xarray=zeros(22,n_time);
conflict=zeros(n_time,1);

for count = 1:n_time
nextx = sixdofSVC_IN1_OmanHILL(x, uinput(count,:), Dt, param);
    x = nextx;
    xarray(:,count) = x;
    dumv_s = [x(4),x(5),x(6)];
    dumhv_s = [x(7), x(8), x(9)];
    conflict(count) = norm(dumv_s - dumhv_s);
%    MSIarray(count) = x(23);
    tarray(count) = count*Dt;

    if rem(count, MABIKI) == 0
        fprintf(fileID,'%f, %f \n', count*Dt, x(19)+x(21)); %MISC
    end
end

fclose(fileID);

omega = uinput(:,1:3);
f = uinput(:,4:6);
x_scc = xarray(1:3,:)';
v_s = xarray(4:6,:)';
hv_s = xarray(7:9,:)';
x_hscc = xarray(10:12,:)';
x_vc = xarray(13:15,:)';
g = xarray(16:18,:)';
uf = xarray(19,:)';
dotuf = xarray(20,:)';
us = xarray(21,:)';
dotus = xarray(22,:)';

a=f-g;
omega_s = x_scc + omega;
a_s = f - v_s;
hg  = param.Kvc*x_vc;
hf = (1/(1+param.Kac)) * (param.Ka*a + param.Kvc*x_vc + param.Kac*(a_s + hv_s));
ha = hf-hg; 
ha_s = hf-hv_s;
homega_s = (1/(1+param.Kwc)) * (x_hscc + param.Kw*omega + param.Kwc*omega_s);
homega = param.Kw*omega + param.Kwc*(omega_s-homega_s);
MISC = us + uf;

time=tarray;

Tbl = table(time, f, homega, ha, hf, hg, g,conflict,MISC);

sp= stackedplot(Tbl, 'XVariable', 'time');
sp.AxesProperties(1).YLimits = [-20 20]; 
sp.AxesProperties(2).YLimits = [-1 1]; 
sp.AxesProperties(3).YLimits = [-5 5]; 
sp.AxesProperties(4).YLimits = [-20 20]; 
sp.AxesProperties(5).YLimits = [-20 20];
sp.AxesProperties(6).YLimits = [0 10];
sp.AxesProperties(7).YLimits = [0 5]; 
sp.AxesProperties(8).YLimits = [0 10]; 

MISCFinaltime = MISC(end)

