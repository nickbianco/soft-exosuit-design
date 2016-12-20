function output = musdynEndpoint_lMtildeState_MinAlex(input)

q = input.phase.integral;
t0 = input.auxdata.initialtime;
tf = input.auxdata.finaltime;
output.objective = q/(tf-t0);

NMuscles = input.auxdata.NMuscles;

% Initial and end states
a_end = input.phase.finalstate(1:NMuscles);
lMtilde_end = input.phase.finalstate(NMuscles+1:end);

a_init = input.phase.initialstate(1:NMuscles);
lMtilde_init = input.phase.initialstate(NMuscles+1:end);

% Constraints - mild periodicity
pera = a_end - a_init;
perlMtilde = lMtilde_end - lMtilde_init;

output.eventgroup.event = [pera perlMtilde];
