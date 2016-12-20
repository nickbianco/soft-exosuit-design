function [] = Opensim_ID(model_dir,event,input_GRF_settings,input_motion,output_path,output_name,output_settings)
%Opensim_ID Calculates Inverse Dynamics
%   model_dir                   path+name of the model
%   event                   rowvector with start and end time
%   input_GRF_settings      path+name of the input GRF settings
%   input_motion            path+name of the input motion (ik)
%   output_path             path of the output file
%   output_name             name of the output file
%   output_settings         path+name of the settings file

import org.opensim.modeling.*

% get the generic ID settings
[FunctionPath,~]=fileparts(mfilename('fullpath'));
path_generic_file=fullfile(FunctionPath,'generic_ID_settings.xml');

% use the loaded model
model=Model(model_dir);
model.initSystem();     % initialise the model

% initialise the ID tool
idTool = InverseDynamicsTool(path_generic_file);
% idTool.setLoadModelAndInput(true)
idTool.setModel(model);
% input external loads
idTool.setExternalLoadsFileName(input_GRF_settings);
% get the name
[~, name,~]=fileparts(input_motion);

% Setup the idTool for this trial
idTool.setName(name);
idTool.setCoordinatesFileName(input_motion);
idTool.setLowpassCutoffFrequency(6)

% set up the events
idTool.setStartTime(event(1,1));
idTool.setEndTime(event(1,2));

% set output of the id tool
idTool.setResultsDir(output_path)
idTool.setOutputGenForceFileName(output_name);

% Save the settings in a setup file
idTool.print(output_settings);

%% gets out of memory=> problem (memory leak ?)
% run the idTool
idTool.run();
% 

%% command line
% 
% xml=xml_read(output_settings);
% xml.InverseDynamicsTool.model_file=model_dir;
% event_range=xml.InverseDynamicsTool.time_range;
% xml.InverseDynamicsTool.time_range=num2str(event_range);
% xml_write(output_settings,xml,'OpenSimDocument');
% 
% % trhough command line
% Command = [exe_path ' -S ' output_settings];
% % try
% %     importdata(fullfile(output_path,output_name));
% % catch
% %         system(Command);
% %         importdata(fullfile(output_path,output_name));
% % end
% system(Command);
% % 
% % clear all
% % close all
end

