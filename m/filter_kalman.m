function [fxu fe fPu fPp ] = filter_kalman(z)
% filters a signal by replacing single spikes with a predicted value
% a error variance is computed to juge if a predicted value is useful.
%   x               = signal input vector
%   f               = filtered vaules of x
%   ferror1         = difference between f and x
%   ferror1_var     = variance of ferror1
%   ferror2         = difference between prediction using the filter and x
%   threshold_error = threhold do identify spikes usi
%   threshold_var   = threhold on the variance of the error between prediction and signal


  x0 = [z(1); 0.5];
  P0 = [1, 0; 0, 0.2];
  A =  [1, 1; 0 ,1];
  Q = [0.1 0; 0 0.1];
  n = length(z);
  fxu = zeros(n,1);
  fe = zeros(n,1);
  fPu = zeros(n,1);
  fPp = zeros(n,1);
  fxu(1) = z(1);
  for i = 2:n
    x1p = A * x0;
    P1p = A*P0*A + Q;
    
    z0 = z(i);
    R = 2.;
    C = [1, 0];
    K = P1p * C' * inv( C * P1p * C' + R);
    x1u = x1p + K * (z0 - C*x1p);
    P1u = (eye(2,2)- K * C) * P1p;
    fxu(i) = x1u(1,1);
    fe(i) = z(i) - x1u(1,1);
    fPu(i) = P1u(1,1);
    fPp(i) = P1p(1,1);
    x0 = x1u;
    P0 = P1u;
  end
end