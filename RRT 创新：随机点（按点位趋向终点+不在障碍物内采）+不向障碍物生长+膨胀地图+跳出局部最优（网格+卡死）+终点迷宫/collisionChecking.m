function feasible=collisionChecking(startPose,goalPose,map)%判断两点连线间有无障碍物，返回值为feasible
 %判断障碍就是看像素是不是黑色
feasible=true;%默认路径可行
dir=atan2(goalPose(2)-startPose(2),goalPose(1)-startPose(1));%计算从起始点到目标点的角度。atan2函数返回两点之间连线的角度
for r=0:0.5:sqrt(sum((startPose-goalPose).^2))%从0开始，每次以0.5的步长递增，直到起始点到目标点的欧氏距离。这是在沿着起始点到目标点的直线检查路径上的每个点。
    posCheck = startPose + r.*[cos(dir) sin(dir)];%计算最近点与新生点间所有点的坐标
    if ~(feasiblePoint(ceil(posCheck),map) && feasiblePoint(floor(posCheck),map) && ...%由于点坐标为小数而地图像素为整数，所以要将横纵坐标分别向上向下取整，分4种情况
            feasiblePoint([ceil(posCheck(1)) floor(posCheck(2))],map) && feasiblePoint([floor(posCheck(1)) ceil(posCheck(2))],map))
        feasible=false;break;%若点横纵坐标向上向下取整的4个点有任意一个在障碍里则为false
    end
    if ~feasiblePoint([floor(goalPose(1)),ceil(goalPose(2))],map), feasible=false; end
 
end
 
function feasible=feasiblePoint(point,map)
feasible=true;%无障碍为true
if ~(point(1)>=1 &&  point(1)<=size(map,2) && point(2)>=1 && point(2)<=size(map,1) && map(point(2),point(1))==255)
    feasible=false;%若点横坐标不超地图左右界，纵坐标不超地图上下界，且地图在此点像素值为255，则为true，否则为false
end