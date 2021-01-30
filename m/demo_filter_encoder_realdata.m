clf
m = csvread ('motor00.csv', 'headerlines', 1);
n = size(m)(1);
t = m(:,1);
encoder_motor = m(:,2);
encoder_phase = m(:,3);
hold off;

%plot(m(:,1), m(:,2),";Rotor ASxxxx;",m(:,1), m(:,3),";Theta ASxxxx;", m(:,1), m(:,4),";Theta SVM;");
%hold on;
encoder_motor_max = 4096;
encoder_motor_phase_max = encoder_motor_max/11;

a = encoder_motor/encoder_motor_max*pi*2;
s = cos(a);
plot(t,s, 'LineWidth', 1, 'b','DisplayName','Input');
hold on

[f, fe1, fe1var, fe2] = filter_encoder(s', 0.4, 0.1)

plot(t,f, 'og','DisplayName','Output');
plot(t,fe1, 'r','DisplayName','Error Input-Output');
plot(t,fe1var, 'r','DisplayName','ErrorVarianz');
plot(t,fe2, 'b','DisplayName','Error on prediction');
legend