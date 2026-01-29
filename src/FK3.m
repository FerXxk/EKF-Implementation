clear all;
clc;

r=0.05;
q=9;
T=1;

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
for i=300:349
    xgps(i)=-1;
    ygps(i)=-1;
end




hold on;

%plot(xgt,ygt,'o')
%plot(xgps,ygps,'o')


%% FILTRO KALMANN 2

R=r^2*eye(4);
Q=q^2*eye(4);

A=[1 0 T 0; 0 1 0 T; 0 0 1 0; 0 0 0 1];
C=[1 0 0 0; 0 1 0 0; 1 0 -T 0; 0 1 0 -T];

mu=[xgps(2) ; ygps(2); (xgps(2)-xgps(1))/T; (ygps(2)-ygps(1))/T];
vmu(:,2)=mu;
mu=[xgps(1) ; ygps(1);0 ; 0];
vmu(:,1)=mu;

sigma=q^2*eye(4);

    
  vsigma=sigma;
  v_det=zeros(1)
j=1;
 

for i=3:length(xgt)
    mup= A * mu;

    sigmap= A * sigma * A'+ R;

    K=sigmap * C'/(C* sigmap * C' + Q);

    z=[xgps(i);ygps(i);xgps(i-1);ygps(i-1)];

    if ((xgps(i)==-1) && (ygps(i)==-1))
    mu=mup;

    v_det(j)=det(sigmap);

    j=j+1;
    else
    mu=mup+ K*(z - C * mup);
    end

    sigma=(eye(4) - K * C) * sigmap;

    vmu(:,i) = mu;

    vsigma(:,:,i)= sigma;

end

    figure(1);
plot(xgt, ygt, 'g-', 'LineWidth', 2); hold on; % Trayectoria real
plot(xgps, ygps, 'rx', 'MarkerSize', 5); % Mediciones ruidosas
plot(vmu(1, :), vmu(2, :), 'b-', 'LineWidth', 2); % Estimación del filtro de Kalman
legend('Trayectoria real', 'Mediciones GPS', 'Estimación KF');
xlabel('X'); ylabel('Y');
title('Filtro de Kalman - Seguimiento de Trayectoria');
grid on;
figure(2);
x=1:(j-1);
plot(x,v_det)
    