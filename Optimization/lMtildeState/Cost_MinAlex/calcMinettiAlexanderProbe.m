function Edot = calcMinettiAlexanderProbe(v,vmax,Fo,a)
% INPUTS:
%        v: shortening speed [m/s]
%     vmax: max shortening speed [m/s]
%       Fo: max isometric force
%        a: activation

% OUTPUTS:
%     Edot: metabolic rate

% Minetti & Alexander (1997) model parameters
c1 = 0.054;
c2 = 0.506;
c3 = 2.46;
c4 = 1.13;
c5 = 12.8;
c6 = 1.64;

phi = (c1*ones(size(v/vmax)) + c2*(v/vmax) + c3*((v/vmax).^2))./ ... 
      (ones(size(v/vmax)) - c4*(v/vmax) + c5*((v/vmax).^2) - c6*((v/vmax).^3)); 

% Total metabolic rate
Edot = Fo*vmax*a.*phi;

