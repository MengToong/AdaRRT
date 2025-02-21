% 检查路径是否与障碍物保持安全距离
% function isSafe = isSafeDistance(point1, point2, image, safeDist)
%     numPoints = round(1.5 * norm(point2 - point1));
%     x = linspace(point1(1), point2(1), numPoints);
%     y = linspace(point1(2), point2(2), numPoints);
% 
%     for i = 1:numPoints
%         if ~isAreaFree(image, ceil(y(i)), ceil(x(i)), safeDist)
%             isSafe = false;
%             return;
%         end
%     end
%     isSafe = true;
% end
% 检查路径是否与障碍物保持安全距离
function [isSafe, safePoint] = isSafeDistance(point1, point2, image, safeDist)
    numPoints = round(1.5 * norm(point2 - point1));
    x = linspace(point1(1), point2(1), numPoints);
    y = linspace(point1(2), point2(2), numPoints);
    safePoint = point1; % 初始化为起始点

    for i = 1:numPoints
        if ~isAreaFree(image, ceil(y(i)), ceil(x(i)), safeDist)
            isSafe = false;
            return;
        else
            safePoint = [x(i), y(i)]; % 更新最远安全点的位置
        end
    end
    isSafe = true;
end

% 检查给定点周围的区域是否与障碍物保持安全距离
function isFree = isAreaFree(image, x, y, safeDist)
    [rows, cols] = size(image);
    for i = max(1, x - safeDist):min(rows, x + safeDist)
        for j = max(1, y - safeDist):min(cols, y + safeDist)
            if image(i, j) < 255 % 如果该点是障碍物
                isFree = false;
                return;
            end
        end
    end
    isFree = true;
end