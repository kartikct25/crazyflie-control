%% Read and allocate Data
file = 'CSV_landing.txt'
CSV = csvread(file);

t = CSV(:,1);
Pos = CSV(:,2:4);
Euler = CSV(:, 5:7);

Refs = CSV(:, 8:11);

T = CSV(:, 12);

%% Plots

figure(1)
plot(t, Pos)
legend('X','Y','Z')

figure(2)
subplot(2, 1, 1)
plot(t, Euler)
legend('Theta','Phi','Psi')
subplot(2, 1, 2)
plot(t, Refs)
legend('theta_r','phi_r','psi_r', 'Z_ref')


figure(3)
plot(t, [T Pos(:,3) Refs(:,4)])