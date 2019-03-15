%% Read and allocate Data
file = 'CSV_landing.txt'
CSV = csvread(file);

t = CSV(:,1);
Pos = CSV(:,2:4);
Euler = CSV(:, 5:7);

Refs = CSV(:, 8:11);

T = CSV(:, 12);

window = round(size(T,1)/20);

filteredT = filter(1/window*ones(1,window),[1], T);

%% Plots
set(0,'defaulttextInterpreter','latex')

figure(1);
[axes, xyPlot, zPlot] = plotyy(t, Pos(:,1:2), t, Pos(:,3) );
xyPlot(1).Color = 'r'; xyPlot(2).Color = 'g'; zPlot(1).Color = 'b';
legend('X','Y','Z');
xlabel('Time [s]');
axes(1).YLabel.String = 'Horizontal Position [m]';
axes(1).XLim = [min(t) max(t)];
axes(1).YLim = [-1.1*max(max(abs(Pos(:,1:2)))) 1.1*max(max(abs(Pos(:,1:2))))];
axes(2).YLabel.String = 'Vertical Position [m]';
axes(2).XLim = axes(1).XLim;
axes(2).YLim = [0 1.1*max(Pos(:,3))];
axes(2).YColor = 'b'

figure(2);
% subplot(2, 1, 1);
attiPlot = plot(t, [Euler Refs(:,1:3)]);
attiPlot(1).Color = 'r'; attiPlot(2).Color = 'g'; attiPlot(3).Color = 'b';
attiPlot(4).Color = 'r'; attiPlot(5).Color = 'g'; attiPlot(6).Color = 'b';
attiPlot(4).LineStyle = '--'; attiPlot(5).LineStyle = '--'; attiPlot(6).LineStyle = '--';
axes = gca;
xlabel('Time [s]');
axes.XLim = [min(t) max(t)];
yExtreme = max(max(abs(Euler)));
axes.YLim = [-yExtreme yExtreme];
legend('\theta', '\phi', '\psi', '\theta_r', '\phi_r', '\psi_r');

figure(3)
[axes, controlPlot, posPlot] = plotyy(t, [T filteredT], t, [Pos(:,3) Refs(:,4)]);
controlPlot(1).Color = 'm';
controlPlot(2).LineStyle = '--';
posPlot(1).Color = 'b'; posPlot(2).Color = 'k';
posPlot(2).LineStyle = '--';
xlabel('Time [s]');
legend('Thrust','Filtered Thrust','Z','Z_r');
axes(1).YLabel.String = 'Thrust';
axes(1).XLim = [min(t) max(t)];
axes(1).YColor = 'm'
axes(2).YLabel.String = 'Vertical Position [m]';
axes(2).XLim = axes(1).XLim;
axes(2).YLim = [0 2*mean(Refs(:,4))];
axes(2).YColor = 'b'

figure(4)
Fs = 1/mean(diff(t));
L = size(Pos(:,3),1);
NFFT = 2^nextpow2(L); % Next power of 2 from length of y
f = Fs/2*linspace(0,1,NFFT/2+1);
Y = fft(Pos(:,3),NFFT)/L;
plot(f,2*abs(Y(1:NFFT/2+1)));
title('Single-Sided Amplitude Spectrum of Z(t)')
xlabel('Frequency (Hz)')
ylabel('|FFT(f)|')