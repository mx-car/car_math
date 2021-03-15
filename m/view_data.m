clf
% mx01
m = csvread ('mx01-motor00.csv');
motor0_offest = -50
motor1_offest = +130
% mx02
% m = csvread ('mx02-motor00.csv');
% motor0_offest = -65
% motor1_offest = -80
m = m(1:500,:);
n = size(m)(1);
t = m(:,1);
pwm_angle = m(:,2);
motor0_encoder = m(:,3);
motor0_angle = m(:,4);
motor1_encoder = m(:,5);
motor1_angle = m(:,6);
hold off;

%plot(m(:,1), m(:,2),";Rotor ASxxxx;",m(:,1), m(:,3),";Theta ASxxxx;", m(:,1), m(:,4),";Theta SVM;");
%hold on;

subplot (2,2,1, "align");
plot(t,motor0_angle, 'LineWidth', 2, 'b','DisplayName','motor0 angle');
hold on
plot(t,pwm_angle, 'LineWidth', 2, 'g','DisplayName','pwm angle');
title ("raw data motor 0");
legend

subplot (2,2,3, "align");
plot(t,motor0_angle, 'LineWidth', 2, 'b','DisplayName','motor0 offset');
hold on
plot(t,mod(pwm_angle+motor0_offest,360), 'LineWidth', 4, 'g','DisplayName','pwm angle');
title (sprintf ("data motor 0 with pwm offset %d deg", motor0_offest));
legend


subplot (2,2,2, "align");
plot(t,motor1_angle, 'LineWidth', 2, 'b','DisplayName','motor1 angle');
hold on
plot(t,pwm_angle, 'LineWidth', 2, 'g','DisplayName','pwm angle');
title ("raw data motor 1");
legend

subplot (2,2,4, "align");
plot(t,motor1_angle, 'LineWidth', 2, 'b','DisplayName','motor1 offset');
hold on
plot(t,mod(pwm_angle+motor1_offest,360), 'LineWidth', 4, 'g','DisplayName','pwm angle');
title (sprintf ("data motor 1 with pwm offset %d deg", motor1_offest));
legend
