clear all; close all; clc

%% Example
% add main folder and subfolder to matlab path (installation)
filepath=which('HipKneeAnkle.m');
[DirCurrent,~,~]=fileparts(filepath);
[DirSoftExosuitDesign,~]=fileparts(DirCurrent);
[DirExamples,~]=fileparts(DirSoftExosuitDesign);
[MainDir,~]=fileparts(DirExamples);
addpath(genpath(MainDir));

% Needed Input Arguments
if isempty(getenv('OPENSIM_HOME'))
    error('You must define the OPENSIM_HOME environment variable.');
end
Datapath = fullfile(getenv('OPENSIM_HOME'), 'Models', 'Gait2354_Simbody', ...
    'OutputReference');
IK_path=fullfile(Datapath,'subject01_walk1_ik.mot');
ID_path=fullfile(Datapath,'ResultsInverseDynamics','inverse_dynamics.sto');
model_path=fullfile(Datapath,'subject01_scaledOnly.osim');
time=[0.6 1.4];     % Right stance phase
OutPath=fullfile(DirCurrent,'Results');

%Misc.MuscleNames_Input={};      % Selects all muscles for the Input DOFS when this is left empty.
Misc.DofNames_Input={'ankle_angle_r','knee_angle_r','hip_flexion_r','hip_adduction_r','hip_rotation_r'};
Misc.Loads_path = fullfile(getenv('OPENSIM_HOME'), 'Models', 'Gait2354_Simbody','subject01_walk1_grf.xml');

% Optional Input Arguments
Misc.costfun = 'Exc_Act';   
Misc.study = 'SoftExosuitDesign/HipKneeAnkle';
Misc.model_mass = 75.1646; % kg (Gait2354 mass)

% Change to cost function subdirectory
cd(fullfile(DirCurrent,Misc.costfun))

%% Solve the problem
Misc.exo_force_level = 0;
[Time,MExcitation,MActivation,RActivation,TForcetilde,TForce,lMtilde,lM,MuscleNames,OptInfo,DatStore]=SolveMuscleRedundancy_lMtildeState(model_path,IK_path,ID_path,time,OutPath,Misc);
filename=strcat('forceLevel',int2str(Misc.exo_force_level),'.mat');
save(filename);

for i = 1:10
    % Device force level
    % 1 --> MIN
    % 2 --> MED
    % 3 --> HIGH
    % 4 --> MAX
    % >4 --> Beyond Quinlivan device peak values
    Misc.exo_force_level = i;
    [Time,MExcitation,MActivation,RActivation,TForcetilde,TForce,lMtilde,lM,MuscleNames,OptInfo,DatStore]=SolveMuscleRedundancy_lMtildeState(model_path,IK_path,ID_path,time,OutPath,Misc);
    filename=strcat('forceLevel',int2str(Misc.exo_force_level),'.mat');
    save(filename);
end
