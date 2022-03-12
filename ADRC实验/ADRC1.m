% Date : 11st March 2022
% 自抗扰控制器（ADRC）实验
% 参考文献：自抗扰控制技术  作者：韩京清
clc
clear all
close all
% 数值仿真方法： 前向欧拉法
ts = 0.01;
tfinal = 30;
Ns = tfinal/ts;
% initial
x = [0 0]'; 
xo = [0 0 0]';
x1 = [0 0]'; x2 = [0 0]'; z1 = 0;
% simulation
for k = 1:1:Ns
   % reference input
   vd = (sign(k*ts-5)+1)/2;
   % TD1 tracking input
   r = 100; h = 0.05;
   x1_d = [x1(2);
          fhan(x1(1)-vd,x1(2),r,h)]; % 系统方程
   x1 = euler2(x1_d,x1,ts);
   % TD2 tracking feedback signal
   y = x(1);
   x2_d = [x2(2);
          fhan(x2(1)-y,x2(2),r,h)];
   x2 = euler2(x2_d,x2,ts);  
   % calculate error
   z1_d = x1(1) - x2(1);
   z1 = euler2(z1_d,z1,ts); % 误差积分
   z2 = x1(1) - x2(1); % 误差
   z3 = x1(2) - x2(2); % 误差微分
   % 误差非线性变换
   z11 = fal(z1,1,0.5);
   z22 = fal(z2,1,0.5);
   z33 = fal(z3,1,0.5);
   % PID控制器
   k1 = 0.001; k2 = 9; k3 = 8.2;
   u = k1*z11+k2*z22+k3*z33-xo(3);
   % plant
   fxd = -(1+0.5*cos(k*ts))*x(1)-(1-0.5*sin(k*ts))*x(2)+sign(sin(0.5*k*ts));
   x_d = [x(2);
          fxd+u];
   x = euler2(x_d,x,ts);
   % ESO
   deta =5; beta1 = 100; beta2 = 3000; beta3 = 100000;
   xo_e = xo(1)-y;
   Ao = [0 1 0; 0 0 1; 0 0 0]; 
   Bo = [beta1 0 0; 0 beta2 0;0 0 beta3];
   Eo = [xo_e;fal(xo_e,0.5,deta);fal(xo_e,0.25,deta)];
   xo_d = Ao*xo-Bo*Eo+[0;1;0]*u;
   xo = euler2(xo_d,xo,ts);
   % time series output
   vdout(k) = vd;
   xout(k,:) = x';
   fout(k) = fxd;
   xoout(k,:) = xo';
   tout(k) = (k-1)*ts;
end
% plot
figure()
plot(tout,xout(:,1),'r-',tout,xoout(:,1),'b--',tout,xout(:,2),'g-',tout,xoout(:,2),'k--','linewidth',2);
legend('实际状态x1','估计状态x1_hat','实际状态x2','估计状态x2_hat');
figure()
plot(tout,fout,'r-',tout,xoout(:,3),'b--','linewidth',2)
legend('实际扰动和非线性项fxd','fxd估计值fxd_hat');
figure()
plot(tout,xout(:,1),'r-',tout,vdout,'b--','linewidth',2)
legend('x1','vd');
