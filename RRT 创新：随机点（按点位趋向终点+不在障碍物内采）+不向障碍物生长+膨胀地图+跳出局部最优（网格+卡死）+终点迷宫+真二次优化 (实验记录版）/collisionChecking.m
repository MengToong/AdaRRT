function feasible=collisionChecking(startPose,goalPose,map)%�ж��������߼������ϰ������ֵΪfeasible
 %�ж��ϰ����ǿ������ǲ��Ǻ�ɫ
feasible=true;%Ĭ��·������
dir=atan2(goalPose(2)-startPose(2),goalPose(1)-startPose(1));%�������ʼ�㵽Ŀ���ĽǶȡ�atan2������������֮�����ߵĽǶ�
for r=0:0.5:sqrt(sum((startPose-goalPose).^2))%��0��ʼ��ÿ����0.5�Ĳ���������ֱ����ʼ�㵽Ŀ����ŷ�Ͼ��롣������������ʼ�㵽Ŀ����ֱ�߼��·���ϵ�ÿ���㡣
    posCheck = startPose + r.*[cos(dir) sin(dir)];%���������������������е������
    if ~(feasiblePoint(ceil(posCheck),map) && feasiblePoint(floor(posCheck),map) && ...%���ڵ�����ΪС������ͼ����Ϊ����������Ҫ����������ֱ���������ȡ������4�����
            feasiblePoint([ceil(posCheck(1)) floor(posCheck(2))],map) && feasiblePoint([floor(posCheck(1)) ceil(posCheck(2))],map))
        feasible=false;break;%�������������������ȡ����4����������һ�����ϰ�����Ϊfalse
    end
    if ~feasiblePoint([floor(goalPose(1)),ceil(goalPose(2))],map), feasible=false; end
 
end
 
function feasible=feasiblePoint(point,map)
feasible=true;%���ϰ�Ϊtrue
if ~(point(1)>=1 &&  point(1)<=size(map,2) && point(2)>=1 && point(2)<=size(map,1) && map(point(2),point(1))==255)
    feasible=false;%��������겻����ͼ���ҽ磬�����겻����ͼ���½磬�ҵ�ͼ�ڴ˵�����ֵΪ255����Ϊtrue������Ϊfalse
end