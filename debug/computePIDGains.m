clear
clc

Data = load('./autosave/autoSaveData-2021-06-18-0920.mat');

% Pull out the time and temperature samples from the range where targetTemp
% was equal to endTemp
d = Data.targetTemp == 120;
t = Data.elapsedTime(d);
t = t ./ 1000; % Convert milliseconds to seconds

xref = Data.refTemp(d);
xsamp = Data.sampTemp(d);

Xref = smoothdata(xref);
Xsamp = smoothdata(xsamp);

subplot(2,2,1);
plot(t,xref,t,Xref)
title('Reference Sample Temperature vs. Time')
xlabel('t (s)')
ylabel('Temperature (C)')

subplot(2,2,2);
plot(t,xsamp,t,Xsamp)
title('Test Sample Temperature vs. Time')
xlabel('t (s)')
ylabel('Temperature (C)')

subplot(2,2,3);
plot(t,xref)
hold on
findpeaks(Xref,t)
title('Reference Sample Temperature vs. Time')
xlabel('t (s)')
ylabel('Temperature (C)')
hold off
[pks,locs] = findpeaks(Xref,t);
refPeriod = mean(diff(locs))

subplot(2,2,4);
plot(t,xsamp)
hold on
findpeaks(Xsamp,t)
title('Test Sample Temperature vs. Time')
xlabel('t (s)')
ylabel('Temperature (C)')
hold off
[pks,locs] = findpeaks(Xsamp,t);
sampPeriod = mean(diff(locs))

G_u = 0.5;
t_u = mean([refPeriod,sampPeriod])

P=0.60*G_u
I=2/t_u
D=t_u/8
