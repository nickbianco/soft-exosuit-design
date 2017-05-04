function [DatStore] = getMuscleInfo(IK_path,ID_path,Misc)
%   Get_dof_MuscleInfo Selects the DOF that are acuated by muscles specified by the user and selects for those dof the moment arms of the muscles
%   author: Maarten Afschrift,
%   Last Update: 12 June 2016

% load joint kinematics
[~,Misc.trialName,~]=fileparts(IK_path);
IK_data=importdata(IK_path);
Names_IK_dofs=IK_data.colheaders(2:end);

% select the IK information between the time events
t_IK=IK_data.data(:,1);     t_IK=round(t_IK*10000)/10000; 
ind0=find(t_IK>=Misc.time(1),1,'first'); ind_end=find(t_IK<=Misc.time(2),1,'last');
IK_inds=ind0:ind_end; nfr=length(IK_inds);

% filter the kinematics and kinetics
fs=1/mean(diff(t_IK));
f_cutoff = Misc.f_cutoff_IK;
orde = Misc.f_order_IK;
[B, A] = butter(orde, f_cutoff/(fs/2));

[~,DOFS] = size(IK_data.data);
for i = 2:DOFS         % filtering time vector not needed (start from 2)
    IK_data.data(:,i) = filtfilt(B,A,IK_data.data(:,i));
end

% Pre-allocate loop variables
DatStore.time=t_IK(IK_inds);
DOF_inds=nan(length(Names_IK_dofs),1);
ct=1;

% Loop over each DOF in the model
for i=1:length(Names_IK_dofs)
    
    % read the Muscle Analysis Result
    dm_Data_temp=importdata(fullfile(Misc.MuscleAnalysisPath,[Misc.trialName '_MuscleAnalysis_MomentArm_' Names_IK_dofs{i} '.sto']));
    
    % get the indexes for the selected muscle Names (only needed in first iteration)
    if i==1        
        headers=dm_Data_temp.colheaders;
        %Add a comment to this line
        Inds_muscles=nan(length(Misc.MuscleNames_Input),1);        
        ctm=1;
        for j=1:length(Misc.MuscleNames_Input)
            ind_sel=find(strcmp(Misc.MuscleNames_Input{j},headers));
            if ~isempty(ind_sel)
               Inds_muscles(ctm)=ind_sel; IndsNames_sel(ctm)=j;
                ctm=ctm+1;
            else
                disp(['Warning: The selected muscle ' Misc.MuscleNames_Input{j} ' does not exist in the selected model. This muscles is removed from the program']);                
            end
        end
        NanMuscles=find(isnan(Inds_muscles));NMuscles_Deleted=length(NanMuscles);
        Misc.MuscleNames=Misc.MuscleNames_Input(IndsNames_sel);
        Inds_muscles(isnan(Inds_muscles))=[];                               % Delete the muscles names that are not used by the user
        dM_temp=nan(nfr,length(Names_IK_dofs),length(Misc.MuscleNames));    % pre-allocate moment arms
        
        % read indexes in time frame for muscle analysis
        t_Mus=dm_Data_temp.data(:,1);           t_Mus=round(t_Mus*10000)/10000;
        ind0=find(t_Mus>=Misc.time(1),1,'first'); ind_end=find(t_Mus<=Misc.time(2),1,'last');
        Mus_inds=ind0:ind_end;
    end
    
    % Evaluate if one of the muscles spans this DOF (when moment arms > 0.01)
    dM=dm_Data_temp.data(Mus_inds,Inds_muscles);    
    if any(any(abs(dM)>0.01));
        Misc.DofNames_muscles{ct}=Names_IK_dofs{i};
        dM_temp(:,i,:)=dM;
         DOF_inds(ct)=i;
         ct=ct+1;
    else
         %disp(['DOF: ' Names_IK_dofs{i} ' is not actuated by the selected muscles. Removed from analysis']);
    end     
end
DOF_inds(isnan(DOF_inds))=[];       % Remove DOFS that are not actuated by muscles

% Combine DOFs_actuated by muscles and the DOFS selected by the user
ct=1;
Inds_deleteDOFS=[];
for i=1:length(Misc.DofNames_muscles)
    if ~any(strcmp(Misc.DofNames_Input,Misc.DofNames_muscles{i}))
         Inds_deleteDOFS(ct)=i;ct=ct+1;
    end
end
DOF_inds(Inds_deleteDOFS)=[];
Misc.DofNames=Misc.DofNames_muscles; Misc.DofNames(Inds_deleteDOFS)=[];

% warnings when not all the input DOFS are actuated by muscles
for i=1:length(Misc.DofNames_Input)
    if ~any(strcmp(Misc.DofNames_Input{i},Misc.DofNames));
        disp(['Warning DOF: The input dof: ' Misc.DofNames_Input{i} ' is not actuated by the selected muscles and therefore removed from the analysis']);
    end
end

% Filter the moment arms information and store them in DatStore.dM
dM_raw=dM_temp(:,DOF_inds,:);
t_dM = dm_Data_temp.data(:,1);
t_dM=round(t_dM*10000)/10000;
fs=1/mean(diff(t_dM));
f_cutoff = Misc.f_cutoff_dM;
orde = Misc.f_order_dM;
[B,A] = butter(orde, f_cutoff/(fs/2));
DatStore.dM = filtfilt(B,A,dM_raw);

% filter Muscle-tendon lengths and store them in DatStore.LMT
LMT_dat=importdata(fullfile(Misc.MuscleAnalysisPath,[Misc.trialName '_MuscleAnalysis_Length.sto']));
LMT_raw=LMT_dat.data(Mus_inds,Inds_muscles);
t_lMT = LMT_dat.data(:,1);
t_lMT=round(t_lMT*10000)/10000;
fs=1/mean(diff(t_lMT));
f_cutoff = Misc.f_cutoff_lMT;
orde = Misc.f_order_lMT;
[B,A] = butter(orde, f_cutoff/(fs/2));
DatStore.LMT = filtfilt(B,A,LMT_raw);

% store various informaiton in the DatStore structure
DatStore.MuscleNames=Misc.MuscleNames;
DatStore.DOFNames=Misc.DofNames;
DatStore.nMuscles=length(Misc.MuscleNames);
DatStore.nDOF=length(Misc.DofNames);
DatStore.q_exp=IK_data.data(IK_inds,DOF_inds+1);        % +1 for time vector

% Get the ID data
ID_data=importdata(ID_path);
t_ID=ID_data.data(:,1); t_ID=round(t_ID*10000)/10000;

% filter the ID data and store in Datstore.T_exp
fs=1/mean(diff(t_ID));
f_cutoff = Misc.f_cutoff_ID;         % [Hz] afsnijfrequentie van de filtering
orde = Misc.f_order_ID;
[B, A] = butter(orde, f_cutoff/(fs/2));
[~,M] = size(ID_data.data);
for i = 2:M         % filtering time vector not needed (start from 2)
    ID_data.data(:,i) = filtfilt(B,A,ID_data.data(:,i));
end

% Create DOF indices for ID data (which can be different than the order 
% for IK).
DOF_ID_inds = zeros(1, length(Misc.DofNames));
for idof = 1:length(Misc.DofNames)
    index = strmatch(Misc.DofNames{idof}, ID_data.colheaders);
    assert(length(index) == 1);
    DOF_ID_inds(idof) = index;
end

% select ID data between start and end
ID_data_int=interp1(ID_data.data(:,1),ID_data.data,IK_data.data(:,1));       % interpolate data for IK sampling frequency
t_ID=ID_data_int(:,1); t_ID=round(t_ID*10000)/10000;
ind0=find(t_ID>=Misc.time(1),1,'first'); ind_end=find(t_ID<=Misc.time(2),1,'last');
ID_inds=ind0:ind_end;
DatStore.T_exp=ID_data_int(ID_inds,DOF_ID_inds);

end
