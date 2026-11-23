% This file is part of 6DoF-SVC_IN model.
% Copyright (c) 2025 Takahiro Wada
% Released under the MIT License.
%
% 2025, June 27
% Test program for 6DoF-SVC-IN1 model (sixdofSVC_IN1_public.m).
% Sinusoidal vertical movement is used.
%
%[1] Inoue, S., Dang, V.T., Liu, H. et al. Construction of a computational model of individual progression 
% of motion sickness symptoms based on subjective vertical conflict theory. Exp Brain Res 243, 108 (2025). 
% https://doi.org/10.1007/s00221-025-07052-5
%[2] V. T. Dang, M. Bourseul and T. Wada, "Prediction of Individual Motion Sickness of Bus Passengers Using 
% a 6DoF-SVC Model Driven by Handheld Tablet Motion During Visual Tasks," in IEEE Transactions on Intelligent 
% Transportation Systems, vol. 27, no. 10, pp. 13461-13472, Oct. 2026, doi: 10.1109/TITS.2026.3710322.

function sixdofSVCparam = setSVCIN1OmanHILLParam()

%Here, the averaged parameter for OmanHILL model in the following paper was used:
%[1] Inoue, S., Dang, V.T., Liu, H. et al. Construction of a computational model of individual progression 
% of motion sickness symptoms based on subjective vertical conflict theory. Exp Brain Res 243, 108 (2025). 
% https://doi.org/10.1007/s00221-025-07052-5

sixdofSVCparam.Ka=0.1;  
sixdofSVCparam.Kw=0.1; 
sixdofSVCparam.Kwc=10;
sixdofSVCparam.Kac=0.5;
sixdofSVCparam.Kvc=5; 
sixdofSVCparam.taud=7;  
%sixdofSVCparam.taua=190; 
sixdofSVCparam.tau=2;    %changed
sixdofSVCparam.P=85; 
sixdofSVCparam.b=1.9;
sixdofSVCparam.beta1 = 23.9;
sixdofSVCparam.beta2 = 277;
sixdofSVCparam.K = 5.97;


