% Date : 11st March 2022
% ����΢������TD2��ʵ��
% �ο����ף��Կ��ſ��Ƽ���  ���ߣ�������
clc
clear all
close all
% ��ֵ���淽���� ǰ��ŷ����
ts = 0.01;
tfinal = 15;
Ns = tfinal/ts;
% initial
x = [0 0]'; 
vd_1 = 0;
% simulation
for k = 1:1:Ns
   % ���������ź�
   % R = 3; deta = 0.00005*R;
   %-------����---------------
%    r = 500; h = 0.1;
%    vd = cos(k*ts)+0.1*randn();
%    vd_d = (vd-vd_1)/ts; vd_1 = vd; 
   % ------��Ծ----------------
%    r = 100; h = 0.5;
%    vd = (sign(k*ts - 3)+1)/2-(sign(k*ts-8)+1)/2+0*randn(); % t = 3ʱ��Ծ
%    vd_d = (vd-vd_1)/ts; vd_1 = vd; % ���Ծ�źŵĵ���
%    %-------�ٶ��ź�-----------------
%    r = 500; h = 0.1;
%    vd = 3*k*ts+0.01*randn();
%    vd_d = (vd-vd_1)/ts; vd_1 = vd;
   %-------���ٶ��ź�-----------------
   r = 1000; h = 0.1;
   vd = (k*ts)^2/2+0.01*randn();
   vd_d = (vd-vd_1)/ts; vd_1 = vd;
   %----------------------------------------
   x1_d = x(2);
   x2_d = fhan(x(1)-vd,x(2),r,h);
   xd = [x1_d x2_d]';
   x = euler2(xd,x,ts);
   % ���״̬
   xout(k,:) = x';
   vdout(k,:) = [vd vd_d]';
   tout(k) = (k-1)*ts;
end
% plot
figure()
subplot(211)
plot(tout,xout(:,1),'r-',tout,xout(:,2),'g-',tout,vdout(:,1),'b--','linewidth',2);
legend('�����ź�x1','����΢���ź�x2','ԭʼ�ź�vd')
subplot(212)
plot(tout,xout(:,2),'r-',tout,vdout(:,2),'b--','linewidth',2);
legend('����΢���ź�x2','ԭʼ΢���ź�vd_d');




