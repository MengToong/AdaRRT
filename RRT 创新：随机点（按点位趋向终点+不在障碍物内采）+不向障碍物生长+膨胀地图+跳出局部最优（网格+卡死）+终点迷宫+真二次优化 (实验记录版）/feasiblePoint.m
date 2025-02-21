function feasible=feasiblePoint(point,map)
feasible=true;%无障碍为true
if ~(point(1)>=1 &&  point(1)<=size(map,2) && point(2)>=1 && point(2)<=size(map,1) && map(point(2),point(1))==255)
    feasible=false;%若点横坐标不超地图左右界，纵坐标不超地图上下界，且地图在此点像素值为255，则为true，否则为false
end
