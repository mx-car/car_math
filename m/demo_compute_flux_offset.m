clf
encoder_motor_max = 16383;
m = csvread ('motor00.csv');
%m = m(1:5000,:);
n = size(m)(1);
t = m(:,1);
encoder_motor = m(:,2);
motor_phase_angle = m(:,3);
svm_phase_angle = m(:,4);
diff_phase_svm_angle = svm_phase_angle - motor_phase_angle
hold off;

%plot(m(:,1), m(:,2),";Rotor ASxxxx;",m(:,1), m(:,3),";Theta ASxxxx;", m(:,1), m(:,4),";Theta SVM;");
%hold on;

a = encoder_motor/encoder_motor_max*pi*2;
s = cos(a);
plot(t,s, 'LineWidth', 1, 'b','DisplayName','Input');
hold on
plot(t,encoder_motor/encoder_motor_max, 'LineWidth', 1, 'b','DisplayName','Input encoder');
plot(t,motor_phase_angle/360, 'LineWidth', 1, 'g','DisplayName','angle phase');
plot(t,svm_phase_angle/360, 'LineWidth', 1, 'r','DisplayName','angle svm');
plot(t,diff_phase_svm_angle/360, 'LineWidth', 1, 'm','DisplayName','angle svm');

x = 65 + motor_phase_angle
for i = 1:n
  if x(i) > 360
    x(i) = x(i) - 360;
  end
  if x(i) < 0
    x(i) = x(i) + 360;
  end
end


plot(t,x/360, 'LineWidth', 1, 'k','DisplayName','angle svm');

legend
hold on