clear all; close all; clc

%% Example
% add main folder and subfolder to matlab path (installation)
filepath=which('Quinlivan2017.m');
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
time=[0.7 1.4];     % Part of the right stance phase
OutPath=fullfile(DirCurrent,'Results');

%Misc.MuscleNames_Input={};      % Selects all muscles for the Input DOFS when this is left empty.
% TODO: select DOFs
Misc.DofNames_Input={'ankle_angle_r','knee_angle_r','hip_flexion_r'};
Misc.Loads_path = fullfile(getenv('OPENSIM_HOME'), 'Models', 'Gait2354_Simbody','subject01_walk1_grf.xml');

% Optional Input Arguments
Misc.costfun = 'Exc_Act';
Misc.study = 'SoftExosuitDesign/Quinlivan2017';

%% Solve the problem
[Time,MExcitation,MActivation,RActivation,TForcetilde,TForce,lMtilde,lM,MuscleNames,OptInfo,DatStore]=SolveMuscleRedundancy_lMtildeState(model_path,IK_path,ID_path,time,OutPath,Misc);
