% ���·���Ƿ����ϰ��ﱣ�ְ�ȫ����
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
% ���·���Ƿ����ϰ��ﱣ�ְ�ȫ����
function [isSafe, safePoint] = isSafeDistance(point1, point2, image, safeDist)
    numPoints = round(1.5 * norm(point2 - point1));
    x = linspace(point1(1), point2(1), numPoints);
    y = linspace(point1(2), point2(2), numPoints);
    safePoint = point1; % ��ʼ��Ϊ��ʼ��

    for i = 1:numPoints
        if ~isAreaFree(image, ceil(y(i)), ceil(x(i)), safeDist)
            isSafe = false;
            return;
        else
            safePoint = [x(i), y(i)]; % ������Զ��ȫ���λ��
        end
    end
    isSafe = true;
end

% ����������Χ�������Ƿ����ϰ��ﱣ�ְ�ȫ����
function isFree = isAreaFree(image, x, y, safeDist)
    [rows, cols] = size(image);
    for i = max(1, x - safeDist):min(rows, x + safeDist)
        for j = max(1, y - safeDist):min(cols, y + safeDist)
            if image(i, j) < 255 % ����õ����ϰ���
                isFree = false;
                return;
            end
        end
    end
    isFree = true;
end