% Naming convention
% af=ankle force, hf=hip force, am=ankle moment, hm=hip moment
% 1=MAX 2=HIGH 3=MED 4=LOW
clear am_peak
clear hm_peak
clear af_peak
clear hf_peak

% Import Data
af=xlsread('Force and Moment Data',1);
hf=xlsread('Force and Moment Data',2);
am=xlsread('Force and Moment Data',3);
hm=xlsread('Force and Moment Data',4);

% Associated percent gait cycle for moments
steps = 201;
xd=linspace(0,100,steps)';

for i=1:4
   af_interp(:,i)=interp_data(af,i,xd); 
   hf_interp(:,i)=interp_data(hf,i,xd);
   am_interp(:,i)=interp_data(am,i,xd); 
   hm_interp(:,i)=interp_data(hm,i,xd); 
end

% Smooth
for i=1:4
   af_smooth(:,i)=smooth(af_interp(:,i)); 
   hf_smooth(:,i)=smooth(hf_interp(:,i)); 
   am_smooth(:,i)=smooth(am_interp(:,i)); 
   hm_smooth(:,i)=smooth(hm_interp(:,i)); 
end

% Average Moment Arms
for i=1:4
   am_ma(:,i)=am_smooth(:,i)./af_smooth(:,i); 
   hm_ma(:,i)=abs(hm_smooth(:,i)./hf_smooth(:,i)); 
end

% Normalize each curve
for i=1:4
   af_normal(:,i)=af_smooth(:,i)./max(af_smooth(:,i)); 
   hf_normal(:,i)=hf_smooth(:,i)./max(abs(hf_smooth(:,i))); 
   am_normal(:,i)=am_smooth(:,i)./max(am_smooth(:,i)); 
   hm_normal(:,i)=hm_smooth(:,i)./max(abs(hm_smooth(:,i))); 
end

% Average curves for hip and ankle
af_avg = mean(af_normal,2);
hf_avg = mean(hf_normal,2);
am_avg = mean(am_normal,2);
hm_avg = mean(hm_normal,2);

% Peaks of curves

for i=1:4
    af_peak(:,i) = max(af_smooth(:,i));
    hf_peak(:,i) = max(abs(hf_smooth(:,i)));
    am_peak(:,i) = max(am_smooth(:,i));
    hm_peak(:,i) = max(abs(hm_smooth(:,i)));
end

af_peak=fliplr(af_peak);
hf_peak=fliplr(hf_peak);
am_peak=fliplr(am_peak);
hm_peak=fliplr(hm_peak);

% Calculate average delta between conditions
af_peak_delta=(af_peak(4)-af_peak(1))/3;
hf_peak_delta=(hf_peak(4)-hf_peak(1))/3;
am_peak_delta=(am_peak(4)-am_peak(1))/3;
hm_peak_delta=(hm_peak(4)-hm_peak(1))/3;

% Create extended peak values
af_extra=af_peak(4)+af_peak_delta:af_peak_delta:af_peak(4)+af_peak_delta*6;
hf_extra=hf_peak(4)+hf_peak_delta:hf_peak_delta:hf_peak(4)+hf_peak_delta*6;
am_extra=am_peak(4)+am_peak_delta:am_peak_delta:am_peak(4)+am_peak_delta*6;
hm_extra=hm_peak(4)+hm_peak_delta:hm_peak_delta:hm_peak(4)+hm_peak_delta*6;

af_peak=[af_peak,af_extra];
hf_peak=[hf_peak,hf_extra];
am_peak=[am_peak,am_extra];
hm_peak=[hm_peak,hm_extra];

% Even Spaced time vector for gait cycle from 0.6 to 1.83

% time vector to match steps of percent gait cycle vector
time_d=linspace(0.6,1.83,steps);
% time and percent gait for every 0.01 sec
time_01=(0.6:0.01:1.83);
percent_01=linspace(0,100,length(time_01));

% interpolate
am_avg_01 = interp1(xd,am_avg,percent_01,'spline');
hm_avg_01 = abs(interp1(xd,hm_avg,percent_01,'spline'));

% shift over hm_avg by 0.7 seconds
hm_norm=[zeros(1,7),hm_avg_01(1,1:length(hm_avg_01)-7)];
am_norm=am_avg_01;
time=time_01;

save('ExoCurves.mat','am_norm','hm_norm','am_peak','hm_peak','af_peak','hf_peak','time')