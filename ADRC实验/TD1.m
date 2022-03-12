% Date : 10th March 2022
% ����΢������TD��ʵ��
% �ο����ף������Ը���-΢����  ���ߣ�������
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
%    vd = cos(k*ts)+0.1*randn();
%    vd_d = -sin(k*ts)+0.1*randn();
%    vd_d0 = (vd-vd_1)/ts; vd_1 = vd; 
   % ------��Ծ----------------
%    R = 5; deta = 0.0005*R;
%    vd = (sign(k*ts - 3)+1)/2-(sign(k*ts-8)+1)/2+0*randn(); % t = 3ʱ��Ծ
%    vd_d = 0;
%    if k*ts == 3
%        vd_d = 10;
%    end
%    vd_d0 = 0;
   %-------���ٶ��ź�-----------------
   R = 5; deta = 0.00005*R;
   vd = 3*k*ts+0.01*randn(); % t = 3ʱ��Ծ
   vd_d = 3;
   vd_d0 = (vd-vd_1)/ts; vd_1 = vd;
   %----------------------------------------
   x1_d = x(2);
   x2_d = -R*sat0(x(1)-vd+abs(x(2))*x(2)/(2*R),deta);
   xd = [x1_d x2_d]';
   x = euler2(xd,x,ts);
   % ���״̬
   xout(k,:) = x';
   vdout(k,:) = [vd vd_d vd_d0]';
   tout(k) = (k-1)*ts;
end
% plot
figure()
subplot(311)
plot(tout,xout(:,1),'r-',tout,vdout(:,1),'b--','linewidth',2);
legend('�����ź�x1','ԭʼ�ź�vd')
subplot(312)
plot(tout,xout(:,2),'r-',tout,vdout(:,2),'b--','linewidth',2);
legend('����΢���ź�x2','ԭʼ΢���ź�vd_d');
subplot(313)
plot(tout,vdout(:,3),'b--',tout,xout(:,2),'r-','linewidth',2);
legend('����΢���ź�vd_d0','����΢���ź�x2');



