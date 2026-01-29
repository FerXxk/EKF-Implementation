clear all

r=0.5;
q=9;

for i=1:200
    xgt(i)=49+i;
    ygt(i)=49+i;
end

for i=1:200
    xgt(i+200)=249+i;
    ygt(i+200)=249-i;
end



for i=1:length(xgt)
    xgps(i)=xgt(i)+q*randn;
    ygps(i)=ygt(i)+q*randn;
end


hold on;

%plot(xgt,ygt,'o')
%plot(xgps,ygps,'o')


%% FILTRO KALMANN 1

R=[r*r 0;0 r*r];
Q=[q*q 0;0 q*q];
A=eye(2);
C=eye(2);

%mu=[xgps(1) ; ygps(1)];
mu=[0;0];
sigma=Q;
 vmu=mu;
  vsigma=sigma;

for i=2:length(xgt)
    mup= A * mu;

    sigmap=A* sigma * A'+ R;

    K=sigmap * C'/(C* sigmap * C' + Q);

    z=[xgps(i);  ygps(i)];

    mu=mup+K*(z - C * mup);

    sigma=(eye(2) - K * C) * sigmap;

    vmu(:,i) = mu;

    vsigma(:,:,i)= sigma;

end

    figure;
plot(xgt, ygt, 'g-', 'LineWidth', 2); hold on; % Trayectoria real
plot(xgps, ygps, 'rx', 'MarkerSize', 5); % Mediciones ruidosas
plot(vmu(1, :), vmu(2, :), 'b-', 'LineWidth', 2); % Estimación del filtro de Kalman
legend('Trayectoria real', 'Mediciones GPS', 'Estimación KF');
xlabel('X'); ylabel('Y');
title('Filtro de Kalman - Seguimiento de Trayectoria');
grid on;
    








