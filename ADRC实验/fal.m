function y = fal( x,a,deta )
%FAL ���任����
% Date : 10th March 2022
% ����΢������TD��ʵ��
% �ο����ף������Ը���-΢����  ���ߣ�������
if deta<=0
    error('deta must be > 0!');end
if abs(x) >= deta
   y = abs(x)^a*sign(x);
else
    y = x/deta^(1-a);
end

end
