% This file is part of 6DoF-SVC_IN model with MISC output
%
% Copyright (c) 2026 Takahiro Wada
% Released under the MIT License.
%
%2026.Nov, 23 
% 6DoFSVC-IN1 model with only Vestibular input
%[1] Inoue, S., Dang, V.T., Liu, H. et al. Construction of a computational model of individual progression 
% of motion sickness symptoms based on subjective vertical conflict theory. Exp Brain Res 243, 108 (2025). 
% https://doi.org/10.1007/s00221-025-07052-5
%[2] V. T. Dang, M. Bourseul and T. Wada, "Prediction of Individual Motion Sickness of Bus Passengers Using 
% a 6DoF-SVC Model Driven by Handheld Tablet Motion During Visual Tasks," in IEEE Transactions on Intelligent 
% Transportation Systems, vol. 27, no. 10, pp. 13461-13472, Oct. 2026, doi: 10.1109/TITS.2026.3710322.

% xnext = sixdofSVCforEXP(xstate, uinput, Dt, prm)

%[OUTPUT]
% xnext = state at next time step (x_i+1) in 25 dimension.
%
%[INPUT]
% xstate = current state (x_i)
%{
	x_scc = xstate(1:3);
	v_s = xstate(4:6);
	hv_s = xstate(7:9);
	x_hscc = xstate(10:12);
	x_vc = xstate(13:15);
	g = xstate(16:18);
    uf = xstate(19);
    dotuf = xstate(20);
    us = xstate(21);
    dotus = xstate(22);
%}
% uinput = current input (u_i) 6 dim.
% uinput=[omega, f]
% where omega= angular velocity in R^3.
% GIF vector f:= g + a
% where g & a are gravitational & intertial accelerations in R^3, respectively. 
% Dt = sampling time [s]
% prm: Struct for model parameters


function	xnext = sixdofSVC_In1_OmanHILL(xstate, uinput, Dt, prm)

% uinput=[omega, f];

x_scc = xstate(1:3);
v_s = xstate(4:6);
hv_s = xstate(7:9);
x_hscc = xstate(10:12);
x_vc = xstate(13:15);
g = xstate(16:18);
uf = xstate(19);
dotuf = xstate(20);
us = xstate(21);
dotus = xstate(22);

omega = uinput(1:3)';

f = uinput(4:6)';
a = f - g;

% a = uinput(4:6)';  % for simulation
% f = a + g;  % for simulation

omega_s = x_scc + omega;
a_s = f - v_s;
hf = (1/(1+prm.Kac)) * (prm.Ka*a + prm.Kvc*x_vc + prm.Kac*(a_s + hv_s));
homega_s = (1/(1+prm.Kwc)) * (x_hscc + prm.Kw*omega + prm.Kwc*omega_s);
dumm = (norm(v_s-hv_s)/prm.b)^2;
x_H = dumm/(1+dumm);
MISC = us + uf;

fnc = [
-x_scc/prm.taud - omega/prm.taud;
(f-v_s)/prm.tau - cross(omega_s, v_s);
(hf-hv_s)/prm.tau - cross(homega_s, hv_s);
-x_hscc/prm.taud - (prm.Kw*omega + prm.Kwc*(omega_s - homega_s))/prm.taud;
v_s - hv_s;
- cross(omega, g);
dotuf;
-2/prm.beta1*dotuf - 1/prm.beta1^2*uf + prm.K/prm.beta1^2*x_H*us;
dotus;
-2/prm.beta2*dotus - 1/prm.beta2^2*us + 1/prm.beta2^2*x_H];

dumxnext = xstate + fnc*Dt;
dumxnext(16:18) = dumxnext(16:18)/norm(dumxnext(16:18))*9.81;
xnext = dumxnext;

end

