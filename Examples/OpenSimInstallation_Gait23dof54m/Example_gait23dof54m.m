clear all; close all; clc
%% Choose formulation
formulation = 'lMtildeState';
% formulation = 'FtildeState';

%% Example
% add main folder and subfolder to matlab path (installation)
filepath=which('Example_Gait23dof54m.m'); [DirExample,~,~]=fileparts(filepath); [DirExample2,~,~]=fileparts(DirExample); [MainDir,~]=fileparts(DirExample2);
addpath(genpath(MainDir));

% Needed Input Arguments
Datapath='C:\OpenSim 3.3\Models\Gait2354_Simbody\OutputReference';
IK_path=fullfile(Datapath,'subject01_walk1_ik.mot');
ID_path=fullfile(Datapath,'ResultsInverseDynamics','inverse_dynamics.sto');
model_path=fullfile(Datapath,'subject01_scaledOnly.osim');
time=[0.7 1.4];     % Part of the right stance phase
OutPath=fullfile(MainDir,'Examples','OpenSimInstallation_Gait23dof54m','Results');

Misc.MuscleNames_Input={};      % Selects all muscles for the Input DOFS when this is left empty.
Misc.DofNames_Input={'ankle_angle_r','knee_angle_r','hip_flexion_r'};

%% Solve the problem
switch formulation
    case 'lMtildeState'
        [Time,MExcitation,MActivation,RActivation,TForcetilde,TForce,lMtilde,lM,MuscleNames,OptInfo,DatStore]=SolveMuscleRedundancy_lMtildeState(model_path,IK_path,ID_path,time,OutPath,Misc);
    case 'FtildeState'   
        [Time,MExcitation,MActivation,RActivation,TForcetilde,TForce,lMtilde,lM,MuscleNames,OptInfo,DatStore]=SolveMuscleRedundancy_FtildeState(model_path,IK_path,ID_path,time,OutPath,Misc);
end