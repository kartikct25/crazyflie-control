%% Read and allocate Data
file = 'CSV_landing.txt'
CSV = csvread(file);

t = CSV(:,1);
Pos = CSV(:,2:4);
Euler = CSV(:, 5:7);

Refs = CSV(:, 8:11);

T = CSV(:, 12);

%% Plots
set(0,'defaulttextInterpreter','latex')

figure(1)
posPlot = plot(t, Pos(:,1),'r', t, Pos(:,2), 'g', t, Pos(:,3), 'b')
legend('X','Y','Z')
xlabel('Time [s]');
ylabel('Position [m]');
% axObjs = posPlot.Children
% dataObjs = axObjs.Children

% posChild = posPlot.Children;
% axes = gca;

figure(2)
subplot(2, 1, 1)
attiPlot = plot(t, Euler)
attiPlot(1).Color = 'r'; attiPlot(2).Color = 'g'; attiPlot(3).Color = 'b';

legend('\theta','\phi','\psi')
subplot(2, 1, 2)
attiRefPlot = plot(t, Refs)
attiRefPlot(1).Color = 'r'; attiRefPlot(2).Color = 'g'; attiRefPlot(3).Color = 'b';
legend('\theta_r','\phi_r','\psi_r', 'Z_r')


figure(3)
controlPlot = plot(t, [T Pos(:,3) Refs(:,4)])
controlPlot(1).Color = 'm'; controlPlot(2).Color = 'b'; controlPlot(3).Color = 'k';
controlPlot(3).LineStyle = '--';
legend('Thrust','Z','Z_r','--')