% Date : 11st March 2022
% 跟踪微分器（TD2）实验
% 参考文献：自抗扰控制技术  作者：韩京清
clc
clear all
close all
% 数值仿真方法： 前向欧拉法
ts = 0.01;
tfinal = 15;
Ns = tfinal/ts;
% initial
x = [0 0]'; 
vd_1 = 0;
% simulation
for k = 1:1:Ns
   % 期望跟踪信号
   % R = 3; deta = 0.00005*R;
   %-------正弦---------------
%    r = 500; h = 0.1;
%    vd = cos(k*ts)+0.1*randn();
%    vd_d = (vd-vd_1)/ts; vd_1 = vd; 
   % ------阶跃----------------
%    r = 100; h = 0.5;
%    vd = (sign(k*ts - 3)+1)/2-(sign(k*ts-8)+1)/2+0*randn(); % t = 3时阶跃
%    vd_d = (vd-vd_1)/ts; vd_1 = vd; % 求阶跃信号的导数
%    %-------速度信号-----------------
%    r = 500; h = 0.1;
%    vd = 3*k*ts+0.01*randn();
%    vd_d = (vd-vd_1)/ts; vd_1 = vd;
   %-------加速度信号-----------------
   r = 1000; h = 0.1;
   vd = (k*ts)^2/2+0.01*randn();
   vd_d = (vd-vd_1)/ts; vd_1 = vd;
   %----------------------------------------
   x1_d = x(2);
   x2_d = fhan(x(1)-vd,x(2),r,h);
   xd = [x1_d x2_d]';
   x = euler2(xd,x,ts);
   % 输出状态
   xout(k,:) = x';
   vdout(k,:) = [vd vd_d]';
   tout(k) = (k-1)*ts;
end
% plot
figure()
subplot(211)
plot(tout,xout(:,1),'r-',tout,xout(:,2),'g-',tout,vdout(:,1),'b--','linewidth',2);
legend('跟踪信号x1','跟踪微分信号x2','原始信号vd')
subplot(212)
plot(tout,xout(:,2),'r-',tout,vdout(:,2),'b--','linewidth',2);
legend('跟踪微分信号x2','原始微分信号vd_d');




