%   1. ��ʼ����Ҫ�滮��2D��3D��ͼ�������ƶ������˻��е��Ҫ�˶��������յ����꣬���ĸ�������㡣
%   2. ��ʼ�����������������Ĺ�������Ϊ��a) �����ڵ�ͼ�����ѡһ��Prand��b) �����������еĵ㣬
%      �������ҵ���Prand����ĵ㣬����Pnear��ѡȡPnear��Prand�����������ѡȡһ���̶�����d���õ�һ����Pnear֮�����Ϊd�ģ�
%      ����ΪPnear��Prand�����һ���µĵ�Pnew��c���ж�Pnear��Pnew֮���Ƿ����ײ����Χ�Ļ��������ж�Pnear��Pnew֮���Ƿ���collision-free�ģ���
%      �������ײ�����Pnew�������У�����¼Pnew�ĸ��ڵ�ΪPnear��������һ�����������ײ����ص�����a)����ѡ�㣻d) �ж�Pnew�Ƿ񵽴�Ŀ��㸽����
%      ��ŷ�Ͼ������жϣ�������С��һ����ֵ����ΪPnew������Ŀ��㸽����ֹͣ�����������δ����Ŀ��㸽������ص�a)����ѭ���������̣�ֱ��Pnew����Ŀ��㸽����
%   3. �ҵ�Ŀ���󣬴�Ŀ��㿪ʼ�ҵ�����·��������Ϊ��������ÿ����ĸ��ڵ㣬ֱ���ҵ����Ϊֹ��
%   4. �õ�����·������ʱ���·������ƽ������������������B��������ȣ����ﲻ�����ܡ�
%% ���̳�ʼ��

results = zeros(100, 6);

for cishu = 1:100
    cishu %��ʾʵ�����
    clearvars -except cishu results% �������cishu��results֮������б���    
    close all;% �ر�����ͼ�δ���
    % clear all; close all;
    m=0;
%     x_I=1; y_I=1;           % ���ó�ʼ��%%%%%%%%%%sy1
%     x_G=790; y_G=790;       % ����Ŀ���
    x_I=1; y_I=1;           % ���ó�ʼ��%%%%%%%%%%sy2
    x_G=400; y_G=700;       % ����Ŀ���
%     x_I=10; y_I=790;           % ���ó�ʼ��%%%%%%%%%%%%%sy3
%     x_G=790; y_G=10;       % ����Ŀ���
    Thr=30;                 % ����Ŀ�����ֵ ��ֵ���˻ᵼ�����һ����ֱ�������ϰ���
    Delta= 50;              % ������չ����
    initial_radius=10;       %��ʼ�������뾶
    delta_radius=80;         %�뾶���Ӳ���
    safeDistance=5;         %��ȫ����
    a=3;%����ײ�����������ӡ���Collision Escape Amplification Factor, ��дΪ CEAF��
    n=0;                    %���µ㿨�ϰ����еĴ���
    last_safepoint=[0,0];
    distance_shengyu=sqrt((x_G - x_I)^2 + (y_G - y_I)^2);%��ʼ���ڵ����յ���������
    deadEndThreshold = 4; % ����ͬ�������ֵ��deadEndGrid���������ֵ�����򽫱���Ϊ����ͬ��

    %% ������ʼ��  TΪ�ṹ�壬vΪ�ṹ�������飬T.v(1)��ʾ���е�һ���ڵ�
    T.v(1).x = x_I;         % T������Ҫ��������v�ǽڵ㣬�����Ȱ���ʼ����뵽T������
    T.v(1).y = y_I; 
    T.v(1).xPrev = x_I;     % ��ʼ�ڵ�ĸ��ڵ���Ȼ���䱾��
    T.v(1).yPrev = y_I;
    T.v(1).dist=0;          % �Ӹ��ڵ㵽�ýڵ�ľ��룬�����ȡŷ�Ͼ���
    T.v(1).indPrev = 0;     % 
    %% ��ʼ������������
    figure(1);
    % ImpRgb=imread('newmap.png');%��ȡ��ͼ
    ImpRgb=imread('sy24.png');%��ȡ��ͼ
    % Imp=rgb2gray(ImpRgb);
    % imshow(Imp)
    % ���ͼ���ǲ�ɫ�ģ�����ת��Ϊ�Ҷ�ͼ��
    if size(ImpRgb, 3) == 3
        Imp = rgb2gray(ImpRgb);
    else
        Imp = ImpRgb;
    end
    % ���Ҷ�ͼ��ת��Ϊ��ֵͼ��
    binaryImage = Imp < 255; % �����ϰ����Ǻ�ɫ��
    % ����һ���ṹԪ�أ����С������İ�ȫ�������Ӧ
    se = strel('disk', safeDistance); % ʹ�ð뾶Ϊ10��Բ�νṹԪ��
    % �Զ�ֵͼ��������Ͳ�������ģ�ⰲȫ����
    dilatedImage = imdilate(binaryImage, se);
    % �����ͺ�Ķ�ֵͼ��ת��Ϊ�׵׺��ϰ���ĻҶ�ͼ��
    dilatedImageInvertedGray = uint8(~dilatedImage) * 255;
    imshow(dilatedImageInvertedGray);
    % imshow(Imp);

    %%%%%%%%%%%%%%%%%%%%%%%%% ��ʼ����������¼�������ܶȺ�����ͬ����
    gridSize = [10, 10]; % ��������ĳߴ磬����ʵ�ʵ�ͼ��С����
    samplingGrid = zeros(gridSize);%���ڼ�¼ÿ����������Ĳ���������
    deadEndGrid = zeros(gridSize);%��δ���ɹ���չʱ���������ڼ�¼ÿ�������Ƿ񱻱��Ϊ����ͬ
    deadEndThreshold = 4; % ����ͬ�������ֵ��deadEndGrid���������ֵ�����򽫱���Ϊ����ͬ��



    xL=size(Imp,1);%��ͼx�᳤��800
    yL=size(Imp,2);%��ͼy�᳤��800
    hold on
%     plot(x_I, y_I, 'ro', 'MarkerSize',5, 'MarkerFaceColor','r');
%     plot(x_G, y_G, 'go', 'MarkerSize',5, 'MarkerFaceColor','g');% ��������Ŀ���
    count=1;%���еĽڵ�����
    tic; % ��ʼ��ʱ
    reset_interval=10;

    for iter = 1:10000%�������3000�������

    %     if mod(iter, reset_interval) == 0
    %         % ÿ��һ���ĵ�������������scale_factorΪ�ϴ�ֵ
    %         scale_factor = 1; % ���������������������ʵ�ֵ
    %     else
    %         % ��������scale_factor
    %         scale_factor = distance_shengyu / sqrt((x_G - x_I)^2 + (y_G - y_I)^2);
    %     end

        p_rand=[];%�����һ��Ϊ�����꣬�ڶ���Ϊ������

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%����㴴��%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
        % �趨һ�����ŵ������Ӷ���С����������
    %     scale_factor = 1 - (count / 500);  % �� 1 �𽥼�С�� 0��/�����ɸ�
        scale_factor=(distance_shengyu*(a^n))/(sqrt((x_G - x_I)^2 + (y_G - y_I)^2));%��1�𽥼�С��0
        if scale_factor>1
            scale_factor=1;
        end
        flag=1;
        while flag
           % ������������꣬ʹ������count���ӣ���Χ����С
           p_rand(1) = x_G + (ceil(rand() * xL) - x_G) * (1.6*scale_factor);%+�ź�ɸģ�Խ��Խ���   %�������Բ�ֵ���������x���յ�x֮���ֵ����ƫ��˭��scale_factor���� ��scale_factorԽ����1Խ��������㣬Խ����0Խ�����յ㣩
           p_rand(2) = y_G + (ceil(rand() * yL) - y_G) * (1.6*scale_factor);
           gridIndex = ceil(p_rand ./ (size(Imp) ./ gridSize)); % ���������������������
           if feasiblePoint(ceil(p_rand),dilatedImageInvertedGray) && feasiblePoint(floor(p_rand),dilatedImageInvertedGray) && feasiblePoint([ceil(p_rand(1)) floor(p_rand(2))],dilatedImageInvertedGray) && feasiblePoint([floor(p_rand(1)) ceil(p_rand(2))],dilatedImageInvertedGray)&&(samplingGrid(gridIndex(1), gridIndex(2)) < deadEndThreshold)%�����������ϰ�����
                flag=0;%�����û���ϰ����ھ���������������ѡ��
           else
           end
        end
%         plot(p_rand(1), p_rand(2), 'yo', 'MarkerSize',3, 'MarkerFaceColor','y');
        m=m+1;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    %     %Step 1: �ڵ�ͼ���������һ����x_rand
    %     %��ʾ���ã�p_rand(1),p_rand(2)����ʾ�����в����������
    %     p_rand(1)=ceil(rand()*xL);%�������������� % rand()���ɵ���0~1���ȷֲ��������������800������ȡ��������Ϊ[1,800]�������
    %     p_rand(2)=ceil(rand()*yL);%���������������

     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%�����㴴��%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
           % ��ʼ�������뾶
        search_radius = initial_radius; % initial_radius �������õĳ�ʼ�����뾶
        found = false;
        p_near=[];%�������
        min_distance = 1000;
        while ~found
            % �洢�������뾶�������ϰ���Ľڵ�
            valid_nodes = [];
            %count����
            for i=1:count%��ȡ�������뾶��Χ�����в����ϰ���Ľڵ�ŵ�valid_nodes��    �����������Ѿ����� T �еĽڵ㣬���������뾶Բ����֮�����ϰ���򽫸ýڵ���ӵ� valid_nodes �����У�
                if isInCircle(p_rand, [T.v(i).x, T.v(i).y], search_radius) && collisionChecking([T.v(i).x, T.v(i).y], p_rand, dilatedImageInvertedGray)
                    valid_nodes = [valid_nodes; T.v(i).x, T.v(i).y, i];%valid_nodesÿһ��Ϊһ�����ýڵ㣬3�зֱ�Ϊ�ýڵ�������꼰����i���������Ӳ������﷨ [A; B] ��ʾ������ B ��Ϊ�µ��и��ӵ����� A �ĵײ�����
                end
            end

            % ����ҵ���Ч�ڵ㣬��ѡ�������
            if ~isempty(valid_nodes)%�������뾶��Χ���в����ϰ���Ŀ��ýڵ���    ��������������ݽṹû���κ�Ԫ�أ�isempty ���� true�����򷵻� false����

                nearest_idx = -1;
                for j = 1:size(valid_nodes, 1)%�ڿ��õ���������ĵ㼰��Ӧ����С����
                    distance = sqrt((valid_nodes(j, 1) - p_rand(1))^2 + (valid_nodes(j, 2) - p_rand(2))^2);
                    if distance < min_distance
                        min_distance = distance;%������С����
                        nearest_idx = j;%�������������valid_nodes�ĵڼ��У���Ӧ���е����һ�м������Ľڵ�����i��
                    end
                end
                if nearest_idx ~= -1%��Ȼִ�У���Ϊִ���������������п��õ㣬�ͱ�Ȼ������С���룬������С���������
                    nearest_node = valid_nodes(nearest_idx, :);%��ȡvalid_nodes����������е������У���nearest_nodeΪ����������е����飬ǰ����Ϊ�����xy���꣬������Ϊ�ڵ�����
                    p_near = [nearest_node(1), nearest_node(2)];%���������
                    index = nearest_node(3);%���������
                    found = true;%���ҵ������
                end
            else%��һ���뾶���޲����ϰ���ĵ㣬�����������뾶
                search_radius = search_radius + delta_radius; % delta_radius �ǰ뾶���ӵĲ���
            end
            if search_radius>=300%size(dilatedImageInvertedGray,1)%����뾶�Ѿ��������û�У��򲻿�������ϰ�����������㷨�����ڵ�  >=�����ɸ�

    %             found = true;
               for i=1:count
                    distance = sqrt( ( T.v(i).x - p_rand(1) )^2 + ( T.v(i).y - p_rand(2) )^2 );%����ÿ��������������
                    if distance < min_distance
                         min_distance = distance;%������С����
                         index = i;%��������������
                    end
               end
               p_near(1) = T.v(index).x;%���ڽ��ڵ������=����ڵ������
               p_near(2) = T.v(index).y;   
               found = true;
            end        
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %     p_near=[];
    %     %Step 2: ���������������ҵ�����ڽ���p_near 
    %     %��ʾ��p_near�Ѿ�����T��
    %     min_distance = 1000;%��ʼĬ����С����
    %     for i=1:count
    %         distance = sqrt( ( T.v(i).x - p_rand(1) )^2 + ( T.v(i).y - p_rand(2) )^2 );%����ÿ��������������
    %         if distance < min_distance
    %             min_distance = distance;%������С����
    %             index = i;%��������������
    %         end
    %     end
    %     p_near(1) = T.v(index).x;%���ڽ��ڵ������=����ڵ������
    %     p_near(2) = T.v(index).y;


        %Step 3: ��չ�õ�p_new�ڵ�
        %��ʾ��ע��ʹ����չ����Delta
    %     p_new(1) = p_near(1) + round( ( p_rand(1)-p_near(1) ) * Delta/min_distance );%���ɣ�
    %     p_new(2) = p_near(2) + round( ( p_rand(2)-p_near(2) ) * Delta/min_distance );

    %������ȷ�Ĵ���ʽӦ�������ж� min_distance �� Delta �Ĺ�ϵ����� min_distance С�� Delta����ֱ�ӽ� p_new ����Ϊ p_rand����� min_distance ���ڻ���� Delta����ʹ��ԭ�ȵļ��㷽����ȷ�� p_new ��λ�á�


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%�����㿼�ǰ�ȫ����%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        p_new=[];

        if min_distance <= Delta
            p_new = p_rand;
        else
            p_new(1) = p_near(1) + round( ( p_rand(1) - p_near(1) ) * Delta / min_distance );
            p_new(2) = p_near(2) + round( ( p_rand(2) - p_near(2) ) * Delta / min_distance );
            gridIndex = ceil(p_new ./ (size(Imp) ./ gridSize)); % �����½ڵ���������
    %         disp(['Grid Index Before Update: ', num2str(gridIndex)]);
        end
    %%%%%%%%%%%%%%%%%%%%%%%%%�����ֲ�����%%%%%%%%%����
    %     [isSafe, safePoint] = isSafeDistance(p_near, p_new, Imp, safeDistance);
    %     if (last_safepoint(1)==safePoint(1))&&(last_safepoint(2)==safePoint(2))
    %         n=n+1;%�������ȡ�İ�ȫ�㶼һ���Ļ�˵�����ֲ����ſ�ס�ˣ���¼��ס�Ĵ���������scale_factor
    %     else
    %         last_safepoint=safePoint;
    %         n=0;
    %     end
    %     p_new = temp_p_new;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


        %���ڵ��Ƿ���collision-free
        if ~collisionChecking(p_near,p_new,dilatedImageInvertedGray) %���½ڵ�����ײ��ֱ�ӽ����¸���ѭ�����²���
            n=n+1;
            deadEndGrid(gridIndex(1), gridIndex(2)) = deadEndGrid(gridIndex(1), gridIndex(2)) + 1;%��������ʧ�ܴ���+1
            disp(['n: ', num2str(n)]);
           continue;
        else
            samplingGrid(gridIndex(1), gridIndex(2)) = samplingGrid(gridIndex(1), gridIndex(2)) + 1;%��������ڵ����+1
            n=0;
    %         disp(['Sampling Grid Value Before Update: ', num2str(samplingGrid(gridIndex(1), gridIndex(2)))]);
        end
        disp(['n: ', num2str(n)]);
        count=count+1;

    %%%%%%%%%%%%%%%%%%%%%%%%%%�����㰴�սڵ�����յ���������������յ���%%%%%%%%%
        distance_pnew_goal=sqrt((x_G - p_new(1))^2 + (y_G - p_new(2))^2);%�����µ����յ����룬��������̾���
        if distance_pnew_goal<distance_shengyu
            distance_shengyu=distance_pnew_goal;%distance_shengyu������յ����������0
        end
        if distance_shengyu<=100 
            deadEndThreshold = 20;
        end;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %Step 4: ��p_new������T 
        %��ʾ���½ڵ�p_new�ĸ��ڵ���p_near
        T.v(count).x = p_new(1);         
        T.v(count).y = p_new(2); 
        T.v(count).xPrev = p_near(1);%������������Ǹ����ٽ�����������ĸ��ڵ�    
        T.v(count).yPrev = p_near(2);
    %     T.v(count).dist = min_distance; %���ɣ�Ӧ�����½ڵ��븸�ڵ���� �������Ǹ��ڵ�����������
        T.v(count).dist = sqrt((p_new(1) - p_near(1))^2 + (p_new(2) -p_near(2))^2); %��


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if collisionChecking(p_new, [x_G, y_G], dilatedImageInvertedGray)%���pnew���յ�֮�����ϰ�����ֱ������
        % ���p_new��Ŀ���֮�����ϰ�
%         elapsedTime = toc; % ������ʱ����ȡ������ʱ��
        T.v(count+1).x = x_G; % ��Ŀ�����Ϊ�µĽڵ��������
        T.v(count+1).y = y_G;
        T.v(count+1).xPrev = p_new(1);
        T.v(count+1).yPrev = p_new(2);
        T.v(count+1).dist = sqrt((x_G - p_new(1))^2 + (y_G - p_new(2))^2); % �������
        T.v(count+1).indPrev = count; % ����ǰ�ڵ�����Ϊ�½ڵ�ĸ��ڵ�

        % ����·������
%         plot(x_G, y_G, 'go', 'MarkerSize',5, 'MarkerFaceColor','g'); % ����Ŀ���
%         line([p_new(1), x_G], [p_new(2), y_G], 'Marker','.','LineStyle','-','color','r'); % �����½ڵ��Ŀ���
        break; % ����ѭ��
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %Step 5:����Ƿ񵽴�Ŀ��㸽�� 
        %��ʾ��ע��ʹ��Ŀ�����ֵThr������ǰ�ڵ���յ��ŷʽ����С��Thr����������ǰforѭ��
        new_distance = sqrt( ( p_new(1) - x_G )^2 + ( p_new(2) - y_G )^2 );
        [isSafe, safePoint] = isSafeDistance(p_new, [x_G,y_G], Imp, safeDistance);
        if new_distance <= Thr && collisionChecking(p_new,[x_G,y_G],dilatedImageInvertedGray)&&isSafe%С���£��յ�����ϰ�����
%             elapsedTime = toc; % ������ʱ����ȡ������ʱ��
%             plot(p_new(1), p_new(2), 'bo', 'MarkerSize',2, 'MarkerFaceColor','b'); % ����p_new��p_new(1), p_new(2)�ǵ��x��y���ꡣ�����ʽ���趨Ϊ��ɫԲȦ��'bo'����MarkerSize�趨Ϊ2����Ĵ�С����MarkerFaceColor�趨Ϊ��ɫ����������ɫ��
%             line( [p_new(1) p_near(1)], [p_new(2) p_near(2)], 'Marker','.','LineStyle','-'); %����p_near��p_new���߶ε������p_near������ڽڵ㣩���յ���p_new���½ڵ㣩��[p_new(1) p_near(1)]��[p_new(2) p_near(2)]�ֱ����߶���x��y���ϵ����ꡣ'Marker','.'ָ���߶��ϵĵ����ʽ��'LineStyle','-'ָ���߶ε���ʽΪʵ�ߡ�
%             line( [x_G p_new(1)], [y_G p_new(2)], 'Marker','.','LineStyle','-'); %����Goal��p_new
            break;
        end

        %Step 6:��p_near��p_new֮���·��������
        %��ʾ 1��ʹ��plot���ƣ���ΪҪ�����ͬһ��ͼ�ϻ����߶Σ�����ÿ��ʹ��plot����Ҫ����hold on����
        %��ʾ 2�����ж��յ���������forѭ��ǰ���ǵð�p_near��p_new֮���·��������
%         plot(p_new(1), p_new(2), 'bo', 'MarkerSize',2, 'MarkerFaceColor','b'); % ����p_new
%         line( [p_new(1) p_near(1)], [p_new(2) p_near(2)], 'Marker','.','LineStyle','-'); %����p_near��p_new
        hold on;

    %     pause(0.01); %��ͣ0.1s��ʹ��RRT��չ�������׹۲�
        deadEndGrid(deadEndGrid >= deadEndThreshold) = deadEndThreshold;
    end

    %% ����·��
    T_LIST = zeros(size(T.v, 2), 5);%size(T.v, 2)������ T.v �ڵڶ�ά�ȵĴ�С������ T.v ��һ��һά�ṹ�����飨��һ������������� size(T.v, 2) ʵ���Ϸ��ص��Ǹ�������Ԫ�ص������������нڵ��������
    for i=1:size(T.v, 2)%T_LISTÿ��Ϊ1���ڵ㣬ÿ������Ϊ����ڵ�� x ���ꡢy ���ꡢ���ĸ��ڵ�� x ���ꡢ���ڵ�� y ����ͽڵ��������
        T_LIST(i,1) = T.v(i).x;
        T_LIST(i,2) = T.v(i).y;
        T_LIST(i,3) = T.v(i).xPrev;
        T_LIST(i,4) = T.v(i).yPrev;
        T_LIST(i,5) = i;
    end

    path = [];%���������洢����㵽�յ��·������ʼΪ�����飬֮���ᵽ���м������Ͷ�̬��Ϊ���м���
              %path��һ��Ϊ���һ���㣬�ڶ���Ϊ���ڶ�����......
    path_count = 1;%��¼·���е������
    path(path_count,1) = x_G;%·������ĵ�һ�У�����һ��·���㣩��һ�У��������꣩=�յ������
    path(path_count,2) = y_G;
    path_count = path_count + 1;%�ڶ�·����
    path(path_count,1) = p_new(1);%·������ĵڶ��У����ڶ���·���㣩��һ�У��������꣩=���һ�������������
    path(path_count,2) = p_new(2);
    n_index = node_index(T_LIST, p_new(1), p_new(2));%������һ���������������T_LIST�еڼ����ڵ㣩
    path_count = path_count + 1;%����·����
    path(path_count,1) = T_LIST(i,3);%·������ĵ����У���������·���㣩��һ�У��������꣩= ���һ��������ĸ��ڵ������
    path(path_count,2) = T_LIST(i,4);
    while path(path_count,1) ~= x_I || path(path_count,2) ~= y_I%�ӵ����ڶ��������㣨���һ��������ĸ��ڵ㣩��ʼ����ǰ·����ֻҪ��������һֱѭ��
        new_n_index = node_index(T_LIST, path(path_count,1), path(path_count,2));%���ݽڵ������ô˽ڵ�����
        path_count = path_count + 1;
        path(path_count,1) = T_LIST(new_n_index,3);%���ݴ˽ڵ������õ��˽ڵ㸸�ڵ�����
        path(path_count,2) = T_LIST(new_n_index,4);
        n_index = new_n_index;
    end

    % for i=size(path,1)-1 :-1: 1%��������size(path,1) ���� path �������������·���е��������ѭ���� size(path,1)-1 ��ʼ����path�����ڶ��м��ڶ����㿪ʼ��i+1Ϊ��һ����
    %     line( [path(i,1) path(i+1,1)], [path(i,2) path(i+1,2)], 'Marker','.','LineStyle','-','color','r'); %����p_near��p_new
    %     hold on;
    %     pause(0.1); %��ͣ0.1s��ʹ��RRT��չ�������׹۲�
    % end
    for i=1 :1: size(path,1)-1%�Ľ�������������path(1)��path(2)������һ���㵽���ڶ����㿪ʼ���ߣ�����path���ڶ��е����һ�м��ڶ����㵽��һ�������
        line( [path(i,1) path(i+1,1)], [path(i,2) path(i+1,2)], 'Marker','.','LineStyle','-','color','r'); %����p_near��p_new
        hold on;
    %     pause(0.01); %��ͣ0.1s��ʹ��RRT��չ�������׹۲�
    end






    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  �����Ż�  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % �Ż�·��

    currentNode = [x_I, y_I];%�Ż�·������ʼ��Ϊ���
    optimizedPath = [currentNode];%�洢�Ż����·��,·����һ�����Ȼ����㣬�ȼ���
    furthest_i=size(path, 1);
    while ~isequal(currentNode, [x_G, y_G])%һֱִ�У�ֱ�� currentNode����ǰ�ڵ㣩����Ŀ��� [x_G, y_G]
        furthestNode = currentNode;%furthestNode �����洢�� currentNode ��ʼ��·���ɰ�ȫ�������Զ�ڵ㡣
        for i = furthest_i:-1:1
            [isSafe, safePoint] = isSafeDistance(currentNode, path(i, :), Imp, safeDistance);
            if collisionChecking(currentNode, path(i, :), dilatedImageInvertedGray) %&& isSafe
                furthestNode = path(i, :);
                furthest_i=i;
            else
                break;
            end
        end
        optimizedPath = [optimizedPath; furthestNode];%���ｫ�ҵ�����Զ��ȫ�ڵ� furthestNode ��ӵ��Ż�·�� optimizedPath �У����� currentNode ����Ϊ furthestNode ��������һ�ε���

        currentNode = furthestNode;%�� currentNode ����Ϊ furthestNode ��������һ�ε�����
        disp(['Current Node: ', num2str(currentNode)]);
        disp(['Furthest Node: ', num2str(furthestNode)]);

    end
    samplingGrid
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ���� optimizedPath ���в�ֵ���� densePath
densePath = []; % ��ʼ����ֵ���·��
for i = 1:size(optimizedPath, 1)-1
    segmentStart = optimizedPath(i,:);
    segmentEnd = optimizedPath(i+1,:);
    % ���Բ�ֵ�������м��
    interpolatedX = linspace(segmentStart(1), segmentEnd(1), 10);%����������Ϊ��ֵ�ܶ�
    interpolatedY = linspace(segmentStart(2), segmentEnd(2), 10);
    segmentPoints = [interpolatedX' interpolatedY'];
    densePath = [densePath; segmentPoints(1:end-1,:)]; % �����ظ�����յ�
end
densePath = [densePath; optimizedPath(end,:)]; % ȷ������·�������յ�
% plot(densePath(:,1), densePath(:,2), 'bo', 'MarkerSize', 2, 'MarkerFaceColor', 'b');




cutPoint = [0, 0]; 
finalPath = [densePath(1, :)]; % ��ʼ�����
i = 2; % ��densePath�ĵڶ����㿪ʼ

while i <= size(densePath, 1)
    currentPoint = finalPath(end, :); % ��ǰ��ʼ����finalPath�����һ����
    nextPathPoint = densePath(i, :); % ��һ��·������densePath�еĵ�
    
    % ���ӵ�ǰ�㵽��һ��·�����Ƿ�����ײ
    if ~collisionChecking(currentPoint, densePath(i, :), dilatedImageInvertedGray)
        % �������ײ���ҵ��ӵ�ǰ�㵽��һ���ɹ����ӵ�·����֮����е�
        cutPoint = findClosestPointToObstacle(currentPoint, densePath(i-1, :), dilatedImageInvertedGray, 30);
        k=i
        finalPath = [finalPath; cutPoint]; % ����е㵽����·��
      
                % ��cutPoint��densePath(i, :)���������ϰ��﷢������ײ
        if ~collisionChecking(cutPoint, densePath(i, :), dilatedImageInvertedGray)
            % ��������������������Ǵ�cutPoint���˵�densePath(i-1, :)
            growDirection = densePath(i-1, :) - cutPoint;
            % ��λ����������
            growDirectionNormalized = growDirection / norm(growDirection);
            % ����������������Ҫ����
            growStepSize = 20; 
            
            while true
                % �����������������ƶ�һ��
                growPoint = cutPoint + growStepSize * growDirectionNormalized;
                % ���growPoint��densePath(i, :)�Ƿ�����ײ
                if collisionChecking(growPoint, densePath(i, :), dilatedImageInvertedGray)
                    % ���û�з�����ײ����growPoint����finalPath������ѭ��
                    finalPath = [finalPath; growPoint];
                    
                    break;
                else
                    % �����Ȼ������ײ������cutPointΪ�µ�growPoint������ѭ��
                    cutPoint = growPoint;
                end
            end
        else
            % ���cutPoint��densePath(i, :)û����ײ��ֱ�ӽ�cutPoint����finalPath
%             finalPath = [finalPath; cutPoint];
%         plot(cutPoint(1), cutPoint(2), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r'); % ʹ�ú�ɫԲȦ����е�

        end

        
        
        
%         plot(cutPoint(1), cutPoint(2), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r'); % ʹ�ú�ɫԲȦ����е�
        % ����i��������ǰ������ײ�ĵ㣬������������
%         i = i + 2;
    else
%         finalPath = [finalPath; nextPathPoint]; % û����ײֱ�������һ��·����
        i = i + 1; % �ƶ�����һ��·����
        
    end
%     plot(finalPath(:,1), finalPath(:,2), 'm-', 'LineWidth', 2); % ʹ����ɫ����

    % ����Ƿ��Ѿ�����ֱ�����ӵ��յ�
    if collisionChecking(finalPath(end, :), densePath(end, :), dilatedImageInvertedGray)
            elapsedTime = toc; % ������ʱ����ȡ������ʱ��

        finalPath = [finalPath; densePath(end, :)]; % ֱ�����ӵ��յ�
        break; % ���·������
    end
end
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
    
    % �����Ż����·��
    total_length = 0;%�����Ż����ܳ���
    for i = 1:size(finalPath, 1) - 1
        line([finalPath(i, 1), finalPath(i + 1, 1)], [finalPath(i, 2), finalPath(i + 1, 2)], 'Color', 'k', 'LineWidth', 2);%�����Ż�����
        segment_length = sqrt((finalPath(i+1, 1) - finalPath(i, 1))^2 + (finalPath(i+1, 2) - finalPath(i, 2))^2);%�����Ż�ÿ�γ���
        total_length = total_length + segment_length;%�����Ż����ܳ���
    end
    
    nodes_before = size(path, 1);
    num_of_nodes = size(finalPath, 1);%�����Ż���ڵ�����������յ㣩

%     disp(['time: ', num2str(elapsedTime)]);
%     disp(['(�����Ż���)·������: ', num2str(total_length)]);
%     disp(['��������: ',num2str(m)]);
%     disp(['�ܽڵ���: ', num2str(count)]);
%     disp(['(�����Ż�ǰ)·���ڵ���: ', num2str(nodes_before)]);
%     disp(['(�����Ż���)·���ڵ���: ', num2str(num_of_nodes)]);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    results(cishu, :) = [elapsedTime, total_length, m, count, nodes_before,num_of_nodes];%��ÿ��ʵ�����ŵ���������ʵ�������
    % �����д�� Excel �ļ�
    
    filename = 'imRRT_sy24_final(100)2.xlsx';
xlswrite(filename, results, 'Sheet1', 'A2');%���������ŵ�exel��A2��ʼ��Ԫ

% ��ѡ���� Excel �ļ�����ӱ�ͷ
header = {'Time', 'Path Length', 'Iterations', 'Total Nodes','Nodes Before',  'Path Nodes'};
xlswrite(filename, header, 'Sheet1', 'A1');%��ͷ����exel��A1��ʼ��Ԫ

end


