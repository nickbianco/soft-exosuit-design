import org.opensim.modeling.*

load forceLevel0.mat

numDOFs = DatStore.nDOF;
numMuscles = DatStore.nMuscles;

time = OptInfo.result.solution.phase.time;
numColPoints = length(time);

auxdata = OptInfo.result.setup.auxdata;

% Extract experimental data.
expTime = DatStore.time;
qExp = DatStore.q_exp;
momArmsExp = DatStore.dM;
momArms = interp1(expTime, momArmsExp, time);
jointAngles = pi / 180. * interp1(expTime, qExp, time);

% Extract parts of the solution related to the device.
control = OptInfo.result.solution.phase.control;
state = OptInfo.result.solution.phase.state;

% Get controls
e       = control(:,1:numMuscles); e(e<0)=0;
aT      = control(:,numMuscles+1:numMuscles+numDOFs);
vMtilde = control(:,numMuscles+numDOFs+1:end);

% Get states
a       = state(:,1:numMuscles);
lMtilde = state(:,numMuscles+1:end);

% Joint moment breakdown.
deviceIndices = strmatch('ankle_angle', DatStore.DOFNames);
assert(length(deviceIndices) == 1);

% for idof = 1:numDOFs
%     subplot(numDOFs, 1, idof);
%     hold on;
%     plot(expTime, DatStore.T_exp(:, idof), 'k', 'LineWidth', 2);
%     legendEntries = {'net'};
%     sumMoment = zeros(length(TForce(:, 1)), 1);
%     for imusc = 1:numMuscles
%         if any(momArms(:, idof, imusc)) > 0.00001
%             thisMoment = TForce(:, imusc) .* momArms(:, idof, imusc);
%             plot(time(1:end-1), thisMoment(1:end-1));
%             legendEntries = [legendEntries {MuscleNames{imusc}}];
%             sumMoment = sumMoment + thisMoment;
%         end
%     end
%      deviceIndex = find(deviceIndices == idof);
%      if ~isempty(deviceIndex)
%          normSpringStiff = OptInfo.result.solution.parameter(1);
%          maxSpringStiff = 400; % N-m/rad.
%          rest_angle = OptInfo.result.solution.parameter(2);
%          ankleAngle = -(jointAngles(:, idof) - rest_angle);
%          deviceMoment = maxSpringStiff * normSpringStiff .* ankleAngle;
%          plot(time, deviceMoment);
%          legendEntries = [legendEntries {'device'}];
%          sumMoment = sumMoment + deviceMoment;
%      end
%     plot(time(1:end-1), sumMoment(1:end-1), 'r', 'LineWidth', 2);
%     legendEntries = [legendEntries {'sum'}];
%     legend(legendEntries, 'Interpreter', 'none');
%     title(DatStore.DOFNames{idof}, 'Interpreter', 'none');
%     if idof == numDOFs
%         xlabel('time (s)');
%     end
%     ylabel('moment (N-m)');
% end

% Metabolic cost
modelApoorva = Model('Rajagopal2015.osim');
musclesApoorva = modelApoorva.getMuscles();
% pect_r, quad_fem_r and gem_r are not in Apoorva model
% all replaced by omit
MuscleNamesApoorva = {'glut_med1_r','glut_med2_r','glut_med3_r',...
    'bifemlh_r','bifemsh_r','sar_r','add_mag2_r','tfl_r','omit',...
    'grac_r','glut_max1_r','glut_max2_r','glut_max3_r','iliacus_r',...
    'psoas_r','omit','omit','peri_r','rect_fem_r','vas_int_r'...
    'med_gas_r','soleus_r','tib_post_r','tib_ant_r'};               
muscleMap = containers.Map(MuscleNames,MuscleNamesApoorva);

probeSet = modelApoorva.getProbeSet();
probe = probeSet.get('metabolic_power');
probeUmberger = Umberger2010MuscleMetabolicsProbe.safeDownCast(probe);

rho = 1059.7; % Muscle density [kg/m^3]
maxFiberVel = 12;  % Fiber-lengths per second

lMT = interp1(DatStore.time,DatStore.LMT,time);
[F,Fiso] = calcMuscleForcesDeGroote(a,lMtilde,vMtilde,lMT,auxdata);

musc_energy_rate = NaN(numColPoints,numMuscles);
for m = 1:numMuscles
    muscleNameApoorva = muscleMap(MuscleNames{m});
    if string(muscleNameApoorva)=='omit'
        % leave as NaN
    else
        musc = musclesApoorva.get(muscleNameApoorva);
        Fmax = musc.getMaxIsometricForce;   % Max isometric force [N]
        Lceopt = musc.getOptimalFiberLength;         % Optimal fiber length [m]

        rST = probeUmberger.getRatioSlowTwitchFibers(muscleNameApoorva);
        param_rFT = 1 - rST;        % Proportion of fast-twitch muscle fibers
        param_Arel = 0.1 + 0.4*param_rFT;
        param_Brel = param_Arel*maxFiberVel;

        sigma = probeUmberger.getSpecificTension(muscleNameApoorva); % Specific tension [N/m^2]
        PCSA = Fmax/sigma;      % Physiological cross sectional area [m^2]
        mass = PCSA*rho*Lceopt; % Muscle mass [kg]

        paramsUmb = struct('Lceopt',Lceopt, 'Arel',param_Arel, ...
                    'Brel',param_Brel, 'Fmax',Fmax, 'rFT',param_rFT, ...
                    'VceMax_LceoptsPerSecond',param_Brel/param_Arel, ...
                    'muscleMass',mass, 'scalingFactorS',1.0, ...
                    'versionNumber',2010);
        VCEmax_mps = paramsUmb.VceMax_LceoptsPerSecond * Lceopt; % [m/s]

        heatRates = NaN(numColPoints,5);
        for i = 1:numColPoints
            Lce = lMtilde(i,m)*Lceopt;
            Vce = vMtilde(i,m)*VCEmax_mps;
            heatRates(i,:) = calcUmbergerProbe(Lce,Vce,F(i,m),Fiso(i,m),e(i,m),a(i,m),paramsUmb);
        end

        musc_energy_rate(:,m) = heatRates(:,5) * mass;
    end
end

bodyMass = 75; % kg
wholebody_energy_rate = nansum(musc_energy_rate,2);
norm_average_wholebody_energy_rate = mean(wholebody_energy_rate) / bodyMass





















