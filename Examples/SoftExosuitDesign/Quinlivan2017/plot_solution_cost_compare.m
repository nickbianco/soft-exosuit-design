%% Calculate metabolic cost
clear all; close all; clc;
import org.opensim.modeling.*
load ExoCurves.mat

study = 'Quinlivan2017';

cmap = zeros(11,3);
cmap(1,:) = [0 0 0];
cmap(2:end,:) = jet(10);

% Peak assistive force
peak_force=am_peak.*(75/.6695);

costdir = {'Exc_Act','Exc_Act_Hip_Shift'};
h5 = figure('units','normalized','position',[0 0 0.5 1.0]);

for dir = 1:length(costdir)

for x=1:11

    filename=strcat('forceLevel',int2str(x-1),'.mat');
    load(fullfile(study,costdir{dir},filename))

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
    T_exp = DatStore.T_exp;
%     Fopt_exo = DatStore.Fopt_exo;
%     ankleIdx = strcmp('ankle_angle_r',DatStore.DOFNames);
%     anklePeakForce = Fopt_exo(ankleIdx);
%     ankleID = T_exp(:,ankleIdx);
%     hipIdx = strcmp('hip_flexion_r',DatStore.DOFNames);
%     hipPeakForce = Fopt_exo(hipIdx);
%    	hipID = T_exp(:,hipIdx);
    
    % Interpolate inverse dynamics moments
%     timeID = linspace(0.6,1.4,length(hipID));
%     hipID = interp1(timeID,hipID,time);
%     ankleID = interp1(timeID,ankleID,time);
    
    % Extract parts of the solution related to the device.
    control = OptInfo.result.solution.phase.control;
    state = OptInfo.result.solution.phase.state;

    % Get controls
    e       = control(:,1:numMuscles); e(e<0)=0; e(e>1)=1;
    aT      = control(:,numMuscles+1:numMuscles+numDOFs);
    vMtilde = control(:,numMuscles+numDOFs+1:end);

    % Get states
    a       = state(:,1:numMuscles);
    lMtilde = state(:,numMuscles+1:end);

    % Joint moment breakdown.
    deviceIndices = strmatch('ankle_angle', DatStore.DOFNames);
    assert(length(deviceIndices) == 1);

    % Metabolic cost
    modelApoorva = Model('../Rajagopal2015.osim');
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
    norm_average_wholebody_energy_rate(x) = mean(wholebody_energy_rate) / bodyMass;
    
end



folder = [Misc.costfun '_Hip_Shift'];

%% Plot bar graphs

for i=1:length(norm_average_wholebody_energy_rate)-1
    p_change(i)= (norm_average_wholebody_energy_rate(i+1)-...
    norm_average_wholebody_energy_rate(1))/norm_average_wholebody_energy_rate(1)*100;
end




hold on
if dir==1
    paper_results=[-3.59,-6.45,-14.79,-22.83];
    for i = 1:length(paper_results)
        h=bar(peak_force(i),paper_results(i));
        set(h,'FaceColor',[1 1 1]);
        set(h,'BarWidth',10);
        set(h,'LineWidth',1.5);
    end
end

plot(peak_force,p_change,'k--','LineWidth',1.5)
box on
switch costdir{dir}
    case 'Exc_Act'
        for i = 1:length(p_change)
            plot(peak_force(i),p_change(i),'o','MarkerSize',10,'MarkerEdgeColor',cmap(i+1,:),'MarkerFaceColor',cmap(i+1,:))
        end
    case 'MinAlex'
        for i = 1:length(p_change)
            plot(peak_force(i),p_change(i),'s','MarkerSize',10,'MarkerEdgeColor',cmap(i+1,:),'MarkerFaceColor',cmap(i+1,:))
        end
    case 'Exc_Act_Hip_Shift'
        for i = 1:length(p_change)
            plot(peak_force(i),p_change(i),'^','MarkerSize',10,'MarkerEdgeColor',cmap(i+1,:),'MarkerFaceColor',cmap(i+1,:))
        end
end


hold on
ylabel('Change in Metabolic Rate [%]')
xlabel('Peak Assistive Force [% BW]')
ax = gca;
ax.LineWidth = 1.5;
ax.FontSize = 14;

end




