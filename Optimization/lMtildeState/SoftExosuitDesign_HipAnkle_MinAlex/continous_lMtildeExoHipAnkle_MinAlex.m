function phaseout = continous_lMtildeExoHipAnkle_MinAlex(input)

% Get input data
NMuscles        = input.auxdata.NMuscles;
Ndof            = input.auxdata.Ndof;
tauAct          = input.auxdata.tauAct;
tauDeact        = input.auxdata.tauDeact;
params          = input.auxdata.params;
splinestruct    = input.auxdata.splinestruct;
numColPoints    = size(input.phase.state,1);
Fopt_exo        = input.auxdata.Fopt_exo;

% Get controls
e       = input.phase.control(:,1:NMuscles);
aT      = input.phase.control(:,NMuscles+1:NMuscles+Ndof);
vMtilde = input.phase.control(:,NMuscles+Ndof+1:end-1);
aD      = input.phase.control(:,end);

% Get states
a       = input.phase.state(:,1:NMuscles);
lMtilde = input.phase.state(:,NMuscles+1:end);

% Get tradeoff parameter info
alpha    = input.phase.parameter;
tradeoff = input.auxdata.tradeoff;

% PATH CONSTRAINTS
% Hill-equilibrium constraint
[Hilldiff, F] = ForceEquilibrium_lMtildeStateExoHipAnkle_MinAlex(a,lMtilde,vMtilde,splinestruct.LMT,params,input.auxdata.Fvparam,input.auxdata.Fpparam,input.auxdata.Faparam);

% Moments constraint
Topt = 150;
r = 0.1;
Tdiff = zeros(numColPoints,Ndof);
for dof = 1:Ndof
    T_exp=splinestruct.ID(:,dof);
    T_exo= r*Fopt_exo(dof)*aD.*(ones(numColPoints,1)+tradeoff(dof)*alpha);
    index_sel=(dof-1)*(NMuscles)+1:(dof-1)*(NMuscles)+NMuscles;
    T_sim=sum(F.*splinestruct.MA(:,index_sel),2) + Topt*aT(:,dof) + T_exo;
    Tdiff(:,dof) =  (T_exp-T_sim);
end

phaseout.path = [Tdiff Hilldiff];

% DYNAMIC CONSTRAINTS
% Activation dynamics
dadt = ones(numColPoints,NMuscles);
for m = 1:NMuscles
    dadt(:,m) = ActivationDynamics(e(:,m),a(:,m),tauAct(m),tauDeact(m),input.auxdata.b);
end

% Contraction dynamics is implicit
dlMtildedt = 10*vMtilde;

phaseout.dynamics = [dadt dlMtildedt];

% OBJECTIVE FUNCTION

% Calculate metabolic rate from Minetti & Alexander (1997) model
vmax = params(5,:);  
Fo = params(1,:);   
Edot = zeros(numColPoints,NMuscles);
for m = 1:NMuscles
    v = vmax(1,m)*vMtilde(:,m);
    Edot(:,m) = calcMinettiAlexanderProbe(v,vmax(1,m),Fo(1,m),a(:,m));
end

m = 75;   % kg
g = 9.81; % m/s^2
v = 1.2;  % m/s

wCOT = 1/(m*g*v); % Weight by cost of transport
w1 = 1000;
phaseout.integrand = wCOT.*sum(Edot,2) + sum(a.^2,2) + sum(e.^2,2) + w1.*sum(aT.^2,2);








