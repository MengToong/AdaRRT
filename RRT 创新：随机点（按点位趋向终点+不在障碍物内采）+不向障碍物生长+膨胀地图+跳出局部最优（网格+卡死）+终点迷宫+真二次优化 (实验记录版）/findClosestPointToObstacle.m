function cutPoint = findClosestPointToObstacle(startPoint, endPoint, map, stepSize)
    direction = endPoint - startPoint;%�������ʼ�㵽�յ���������������ָ����ֱ��·���ķ���
    normalizedDirection = direction / norm(direction);%������������λ������ʹ�䳤��Ϊ1��
    cutPoint = startPoint;
    maxDistance = 0;  % ��ʼ����С����Ϊ���޴�
    
    % ȷ�����������
    startPoint = round(startPoint);
    endPoint = round(endPoint);

    for t = 0:stepSize:norm(direction)
        testPoint = startPoint + t * normalizedDirection;
        % ȷ�����Ե�����������
        testPoint = round(testPoint);
        
        % ������Ե㳬��ͼ��߽磬��ֹѭ��
        if any(testPoint < 1) || testPoint(1) > size(map, 2) || testPoint(2) > size(map, 1)
            break;
        end

        distance = estimateDistanceToObstacle(testPoint, map);  % ���Ƶ��ϰ���ľ���
                 
        if distance~=0 && distance >= maxDistance
            cutPoint = testPoint;
            maxDistance = distance;
        end

    end
%         fprintf('Testing point: (%d, %d), Distance: %d\n', testPoint, maxDistance);% ��ӡ���Ե�;�����Ϣ���۲����ִ��״̬      
        plot(cutPoint(1), cutPoint(2), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r'); % ʹ�ú�ɫԲȦ����е�

    
end

%��ʵ��ÿ������Χ�ĺ�ɫ���ص�������Χ�����checkSize����
function distance = estimateDistanceToObstacle(point, map)
    % ȷ������������
    point = round(point);
    % ������������С
    checkSize = 5;  % �Ե�Ϊ���ģ���Χ1�����صķ�Χ��ʵ�ʼ������Ϊ3x3
    
    % ����������ı߽�
    rowMin = max(1, point(2) - checkSize);
    rowMax = min(size(map,1), point(2) + checkSize);
    colMin = max(1, point(1) - checkSize);
    colMax = min(size(map,2), point(1) + checkSize);
    
    % ��ȡ�������
    region = map(rowMin:rowMax, colMin:colMax);
    
    % ���������ڷǰ�ɫ���ص�������Ϊ����Ĺ���
    distance = nnz(region < 255);  % �ǰ�ɫ���ص�������nnz�������Ԫ�ص�����
end
