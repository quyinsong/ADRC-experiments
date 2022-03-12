% Date : 11st March 2022
% 扩张状态观测器（ESO）实验
% 参考文献：自抗扰控制技术  作者：韩京清
clc
clear all
close all
% 数值仿真方法： 前向欧拉法
ts = 0.01;
tfinal = 15;
Ns = tfinal/ts;
% initial
x = [5 5]'; 
xo = [0 0 0]';
% simulation
for k = 1:1:Ns
   % plant
   fxd = -(1+0.5*cos(k*ts))*x(1)-(1-0.5*sin(k*ts))*x(2)+sign(sin(0.5*k*ts));
   u = -xo(3)-4*x(1)-4*x(2);  % 利用估计值进行非线性反馈，然后使用极点配置方法将几点配置在-2
   x_d = [x(2);
          fxd+u];
   x = euler2(x_d,x,ts);
   y = x(1);
   % ESO
   deta =5; beta1 = 100; beta2 = 3000; beta3 = 5000;
   xo_e = xo(1)-y;
   Ao = [0 1 0; 0 0 1; 0 0 0]; 
   Bo = [beta1 0 0; 0 beta2 0;0 0 beta3];
   Eo = [xo_e;fal(xo_e,0.5,deta);fal(xo_e,0.25,deta)];
   xo_d = Ao*xo-Bo*Eo+[0;1;0]*u;
   xo = euler2(xo_d,xo,ts);
   % time series output
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
plot(tout,xout(:,1),'r-',tout,xout(:,2),'b-','linewidth',2)
legend('x1','x2');
