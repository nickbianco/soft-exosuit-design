clear all;close all;clc

%% Choose formulation
formulation = 'lMtildeState';
% formulation = 'FtildeState';

%% Example
% add main folder and subfolder to matlab path (installation)
filepath=which('Running_DeGrooteetal2016.m');
[DirExample_Running,~,~]=fileparts(filepath); [DirExample,~]=fileparts(DirExample_Running);[MainDir,~]=fileparts(DirExample);
addpath(genpath(MainDir));

% Needed Input Arguments
IK_path=fullfile(MainDir,'Examples','Running_DeGrooteetal2016','RunningData','IK_Joggen_1.mot');
ID_path=fullfile(MainDir,'Examples','Running_DeGrooteetal2016','RunningData','ID_Joggen_1.sto');
model_path=fullfile(MainDir,'Examples','Running_DeGrooteetal2016','RunningData','AdDB_Scaled_FB_FA.osim');
time=[0.05 0.98];     % Right stance phase (+50ms beginning and end of time interval, more details see see manual and publication)
OutPath=fullfile(MainDir,'Examples','Running_DeGrooteetal2016','Results');

Misc.MuscleNames_Input={};      % Selects all muscles for the Input DOFS when this is left empty.
Misc.DofNames_Input={'ankle_angle_r','knee_angle_r','hip_flexion_r','hip_adduction_r','hip_rotation_r'};

% Optional Input Arguments
Misc.Atendon = [];        % Tendon Stiffness for the selected muscles
Misc.f_cutoff_ID = 10;         % cutoff frequency filtering ID
Misc.f_order_ID = 5;             % order frequency filtering ID
Misc.f_cutoff_lMT = 10;         % cutoff frequency filtering lMT
Misc.f_order_lMT = 5;             % order frequency filtering lMT
Misc.f_cutoff_dM= 10;         % cutoff frequency filtering MA
Misc.f_order_dM = 5;             % order frequency filtering MA
Misc.f_cutoff_IK= 10;         % cutoff frequency filtering IK
Misc.f_order_IK = 5;             % order frequency filtering IK
%% Solve the problem
switch formulation
    case 'lMtildeState'
        [Time,MExcitation,MActivation,RActivation,TForcetilde,TForce,lMtilde,lM,MuscleNames,OptInfo,DatStore]=SolveMuscleRedundancy_lMtildeState(model_path,IK_path,ID_path,time,OutPath,Misc);
    case 'FtildeState'   
        [Time,MExcitation,MActivation,RActivation,TForcetilde,TForce,lMtilde,lM,MuscleNames,OptInfo,DatStore]=SolveMuscleRedundancy_FtildeState(model_path,IK_path,ID_path,time,OutPath,Misc);
end

