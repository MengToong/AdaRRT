function feasible=feasiblePoint(point,map)
feasible=true;%���ϰ�Ϊtrue
if ~(point(1)>=1 &&  point(1)<=size(map,2) && point(2)>=1 && point(2)<=size(map,1) && map(point(2),point(1))==255)
    feasible=false;%��������겻����ͼ���ҽ磬�����겻����ͼ���½磬�ҵ�ͼ�ڴ˵�����ֵΪ255����Ϊtrue������Ϊfalse
end
