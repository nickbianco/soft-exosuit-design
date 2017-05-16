%% Calculate metabolic cost
clear all
import org.opensim.modeling.*
load ExoCurves.mat

for x=0:10
    filename=strcat('forceLevel',int2str(x),'.mat');
    load(filename)

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
    norm_average_wholebody_energy_rate(x+1) = mean(wholebody_energy_rate) / bodyMass;
    
    activations(:,:,x+1)=a;

end

%% Plot bar graphs
for i=1:length(norm_average_wholebody_energy_rate)-1
    p_change(i)= (norm_average_wholebody_energy_rate(i+1)-...
    norm_average_wholebody_energy_rate(1))/norm_average_wholebody_energy_rate(1)*100;
end

figure;
paper_results=[-3.59,-6.45,-14.79,-22.83];
color=[[0.498, 0.396, 0.635];[0.258, 0.670, 0.784];[0.596, 0.8, 0.403];[0.968, 0.6, 0.215]];
ind=am_peak.*(75/.6695);

hold on
for i = 1:length(paper_results)
    h=bar(ind(i),paper_results(i));
    set(h,'FaceColor',color(i,:));
    set(h,'BarWidth',10);
end

plot(ind,p_change,'k--')
plot(ind,p_change,'k.','MarkerSize',15)
hold off
ylabel('Change in Metabolic Rate [%]')
xlabel('Peak Assistive Force [% BW]')
title('Reduction in Net Metabolic Rate')
%% Plot all activations

figure;
%list of muscle numbers to include
muscles=1:24;
%these do not have muscle parameters
muscles(20)=[];
muscles(17)=[];
muscles(16)=[];
%muscle only crosses knee
muscles(9)=[];

p_stance = (time-0.6)./(1.43-0.6);
cmap=jet(11);

for i=1:length(muscles)
    subplot(5,4,i)
    % remove _r and _ from titles
    title(strrep(strrep(MuscleNames(muscles(i)),'_r',''),'_',''));
    hold on
    for j=1:11
        plot(p_stance,activations(:,muscles(i),j),'Color',cmap(j,:))
    end
end

%% Plot hip flexors/extensors activations

figure;
title('Hip Muscle Activations')
%list of muscle numbers to include
muscles=[14,15,19,4];
p_stance = (time-0.6)./(1.43-0.6);
cmap=jet(11);

for i=1:length(muscles)
    subplot(1,4,i)
    xlabel('Percent Stance Phase')
    if i==1
        ylabel('Activation')
    end
    % remove _r and _ from titles
    title(strrep(strrep(MuscleNames(muscles(i)),'_r',''),'_',''));
    hold on
    for j=1:11
        plot(p_stance,activations(:,muscles(i),j),'Color',cmap(j,:))
    end
end

%% Plot ankle flexors/extensors activations

figure;
%list of muscle numbers to include
muscles=[21,22,23,24];
p_stance = (time-0.6)./(1.43-0.6);
cmap=jet(11);

for i=1:length(muscles)
    subplot(1,4,i)
    xlabel('Percent Stance Phase')
    if i==1
        ylabel('Activation')
    end
    % remove _r and _ from titles
    title(strrep(strrep(MuscleNames(muscles(i)),'_r',''),'_',''));
    hold on
    for j=1:11
        plot(p_stance,activations(:,muscles(i),j),'Color',cmap(j,:))
    end
end

%% Plot high power activations

figure;
%list of muscle numbers to include
muscles=[7,12];
p_stance = (time-0.6)./(1.43-0.6);
cmap=jet(11);

for i=1:length(muscles)
    subplot(1,2,i)
    % remove _r and _ from titles
    title(strrep(strrep(MuscleNames(muscles(i)),'_r',''),'_',''));
    xlabel('Percent Stance Phase')
    ylabel('Activation')
    hold on
    for j=1:11
        plot(p_stance,activations(:,muscles(i),j),'Color',cmap(j,:),'LineWidth',1)
    end
end