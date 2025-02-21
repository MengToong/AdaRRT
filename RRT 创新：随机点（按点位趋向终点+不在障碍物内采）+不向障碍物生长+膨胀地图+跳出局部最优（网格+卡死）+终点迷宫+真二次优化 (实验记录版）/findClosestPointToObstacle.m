function cutPoint = findClosestPointToObstacle(startPoint, endPoint, map, stepSize)
    direction = endPoint - startPoint;%计算从起始点到终点的向量。这个向量指明了直线路径的方向。
    normalizedDirection = direction / norm(direction);%将方向向量单位化（即使其长度为1）
    cutPoint = startPoint;
    maxDistance = 0;  % 初始化最小距离为无限大
    
    % 确保起点是整数
    startPoint = round(startPoint);
    endPoint = round(endPoint);

    for t = 0:stepSize:norm(direction)
        testPoint = startPoint + t * normalizedDirection;
        % 确保测试点是整数坐标
        testPoint = round(testPoint);
        
        % 如果测试点超出图像边界，终止循环
        if any(testPoint < 1) || testPoint(1) > size(map, 2) || testPoint(2) > size(map, 1)
            break;
        end

        distance = estimateDistanceToObstacle(testPoint, map);  % 估计到障碍物的距离
                 
        if distance~=0 && distance >= maxDistance
            cutPoint = testPoint;
            maxDistance = distance;
        end

    end
%         fprintf('Testing point: (%d, %d), Distance: %d\n', testPoint, maxDistance);% 打印测试点和距离信息来观察程序执行状态      
        plot(cutPoint(1), cutPoint(2), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r'); % 使用红色圆圈标记切点

    
end

%其实是每个点周围的黑色像素点数，周围多大是checkSize定的
function distance = estimateDistanceToObstacle(point, map)
    % 确保索引是整数
    point = round(point);
    % 定义检查的区域大小
    checkSize = 5;  % 以点为中心，周围1个像素的范围，实际检查区域为3x3
    
    % 计算检查区域的边界
    rowMin = max(1, point(2) - checkSize);
    rowMax = min(size(map,1), point(2) + checkSize);
    colMin = max(1, point(1) - checkSize);
    colMax = min(size(map,2), point(1) + checkSize);
    
    % 提取检查区域
    region = map(rowMin:rowMax, colMin:colMax);
    
    % 计算区域内非白色像素的数量作为距离的估计
    distance = nnz(region < 255);  % 非白色像素的数量，nnz计算非零元素的数量
end
