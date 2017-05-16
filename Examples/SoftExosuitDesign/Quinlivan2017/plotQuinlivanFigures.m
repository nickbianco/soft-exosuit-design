% DatStore = load('DatStore.mat');
time = DatStore.time;
T_exp = DatStore.T_exp;

figure(1)
plot(time,T_exp(:,5),'k-','LineWidth',2)
hold on

figure(2)
plot(time,T_exp(:,1),'k-','LineWidth',2)
hold on


T_exo = zeros(size(T_exp));
model_mass = 75.1646;


ExoCurves = load('ExoCurves.mat');
exoTime = ExoCurves.time;
exoAnkleMomentPeaks = ExoCurves.am_peak * model_mass;
exoAnkleNormalizedMoment = ExoCurves.am_norm;
exoHipMomentPeaks = ExoCurves.hm_peak * model_mass;
exoHipNormalizedMoment = ExoCurves.hm_norm;

cmap = jet(length(exoAnkleMomentPeaks));

for i=1:length(exoAnkleMomentPeaks)
    exoAnkleMoment = exoAnkleMomentPeaks(i) * exoAnkleNormalizedMoment;
    exoHipMoment = exoHipMomentPeaks(i) * exoHipNormalizedMoment;
    T_exo(:,5) = -interp1(exoTime, exoAnkleMoment, DatStore.time);
    T_exo(:,1) = interp1(exoTime, exoHipMoment, DatStore.time);
    
    figure(1)
    plot(time,T_exo(:,5),'Color',cmap(i,:),'LineWidth',1.25)
    
    figure(2)
    plot(time,T_exo(:,1),'Color',cmap(i,:),'LineWidth',1.25)

    
end

h1 = figure(1);
xlabel('Stance (s)')
ylabel('Ankle moment (N-m)')
print(h1,'ankle_moment','-djpeg')

h2 = figure(2);
xlabel('Stance (s)')
ylabel('Hip moment (N-m)')
print(h2,'hip_moment','-djpeg')
