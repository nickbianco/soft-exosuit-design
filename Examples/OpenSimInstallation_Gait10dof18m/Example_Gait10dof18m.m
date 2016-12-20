clear all;close all;clc

%% Choose formulation
% formulation = 'lMtildeState';
formulation = 'FtildeState';

%% Example
% add main folder and subfolder to matlab path (installation)
filepath=which('Example_Gait10dof18m.m'); [DirExample,~,~]=fileparts(filepath); [DirExample2,~,~]=fileparts(DirExample); [MainDir,~]=fileparts(DirExample2);
addpath(genpath(MainDir));

% Needed Input Arguments
Datapath='C:\OpenSim 3.3\Models\Gait10dof18musc\OutputReference';
IK_path=fullfile(Datapath,'IK','subject01_walk_IK.mot');
ID_path=[]; % compute ID from the external loads
model_path=fullfile(Datapath,'subject01.osim');
time=[0.7 1.4];     % Part of the right stance phase
OutPath=fullfile(MainDir,'Examples','OpenSimInstallation_Gait10dof18m','Results');

Misc.DofNames_Input={'ankle_angle_r','knee_angle_r','hip_flexion_r'};
Misc.Loads_path=fullfile(Datapath,'ExperimentalData','subject01_walk_grf.xml');
Misc.ID_ResultsPath=fullfile(Datapath,'ID','inversedynamics.sto');

%% Solve the problem
switch formulation
    case 'lMtildeState'
        [Time,MExcitation,MActivation,RActivation,TForcetilde,TForce,lMtilde,lM,MuscleNames,OptInfo,DatStore]=SolveMuscleRedundancy_lMtildeState(model_path,IK_path,ID_path,time,OutPath,Misc);
    case 'FtildeState'   
        [Time,MExcitation,MActivation,RActivation,TForcetilde,TForce,lMtilde,lM,MuscleNames,OptInfo,DatStore]=SolveMuscleRedundancy_FtildeState(model_path,IK_path,ID_path,time,OutPath,Misc);
end
