%   1. 初始化需要规划的2D或3D地图，给出移动机器人或机械臂要运动的起点和终点坐标，树的根部在起点。
%   2. 开始搜索并构建树。树的构建过程为：a) 首先在地图上随机选一点Prand；b) 遍历树上所有的点，
%      从树中找到与Prand最近的点，记作Pnear，选取Pnear到Prand方向的向量，选取一个固定长度d，得到一个和Pnear之间距离为d的，
%      方向为Pnear到Prand方向的一个新的点Pnew；c）判断Pnear和Pnew之间是否会碰撞到周围的环境（即判断Pnear和Pnew之间是否是collision-free的），
%      如果无碰撞，则把Pnew加入数中，并记录Pnew的父节点为Pnear，进行下一步，如果有碰撞，则回到步骤a)重新选点；d) 判断Pnew是否到达目标点附近，
%      用欧氏距离来判断，当距离小于一个阈值后，认为Pnew到达了目标点附近，停止建树，如果还未到达目标点附近，则回到a)继续循环整个过程，直到Pnew到达目标点附近。
%   3. 找到目标点后，从目标点开始找到整条路径。方法为查找树上每个点的父节点，直到找到起点为止。
%   4. 得到整条路径后，有时会对路径进行平滑处理，处理方法有三次B样条处理等，这里不作介绍。
%% 流程初始化

results = zeros(100, 6);

for cishu = 1:100
    cishu %显示实验次数
    clearvars -except cishu results% 清除除了cishu和results之外的所有变量    
    close all;% 关闭所有图形窗口
    % clear all; close all;
    m=0;
%     x_I=1; y_I=1;           % 设置初始点%%%%%%%%%%sy1
%     x_G=790; y_G=790;       % 设置目标点
    x_I=1; y_I=1;           % 设置初始点%%%%%%%%%%sy2
    x_G=400; y_G=700;       % 设置目标点
%     x_I=10; y_I=790;           % 设置初始点%%%%%%%%%%%%%sy3
%     x_G=790; y_G=10;       % 设置目标点
    Thr=30;                 % 设置目标点阈值 阈值大了会导致最后一个点直连穿过障碍物
    Delta= 50;              % 设置扩展步长
    initial_radius=10;       %初始化搜索半径
    delta_radius=80;         %半径增加步长
    safeDistance=5;         %安全距离
    a=3;%“碰撞逃逸增幅因子”（Collision Escape Amplification Factor, 缩写为 CEAF）
    n=0;                    %记新点卡障碍物中的次数
    last_safepoint=[0,0];
    distance_shengyu=sqrt((x_G - x_I)^2 + (y_G - y_I)^2);%初始化节点与终点的最近距离
    deadEndThreshold = 4; % 死胡同区域的阈值，deadEndGrid超过这个阈值的区域将被视为死胡同。

    %% 建树初始化  T为结构体，v为结构体中数组，T.v(1)表示树中第一个节点
    T.v(1).x = x_I;         % T是我们要做的树，v是节点，这里先把起始点加入到T里面来
    T.v(1).y = y_I; 
    T.v(1).xPrev = x_I;     % 起始节点的父节点仍然是其本身
    T.v(1).yPrev = y_I;
    T.v(1).dist=0;          % 从父节点到该节点的距离，这里可取欧氏距离
    T.v(1).indPrev = 0;     % 
    %% 开始搜索并构建树
    figure(1);
    % ImpRgb=imread('newmap.png');%读取地图
    ImpRgb=imread('sy24.png');%读取地图
    % Imp=rgb2gray(ImpRgb);
    % imshow(Imp)
    % 如果图像是彩色的，则将其转换为灰度图像
    if size(ImpRgb, 3) == 3
        Imp = rgb2gray(ImpRgb);
    else
        Imp = ImpRgb;
    end
    % 将灰度图像转换为二值图像
    binaryImage = Imp < 255; % 假设障碍物是黑色的
    % 创建一个结构元素，其大小与所需的安全距离相对应
    se = strel('disk', safeDistance); % 使用半径为10的圆形结构元素
    % 对二值图像进行膨胀操作，以模拟安全距离
    dilatedImage = imdilate(binaryImage, se);
    % 将膨胀后的二值图像转换为白底黑障碍物的灰度图像
    dilatedImageInvertedGray = uint8(~dilatedImage) * 255;
    imshow(dilatedImageInvertedGray);
    % imshow(Imp);

    %%%%%%%%%%%%%%%%%%%%%%%%% 初始化网格来记录采样点密度和死胡同区域
    gridSize = [10, 10]; % 设置网格的尺寸，根据实际地图大小调整
    samplingGrid = zeros(gridSize);%用于记录每个网格区域的采样点数量
    deadEndGrid = zeros(gridSize);%树未被成功扩展时计数，用于记录每个区域是否被标记为死胡同
    deadEndThreshold = 4; % 死胡同区域的阈值，deadEndGrid超过这个阈值的区域将被视为死胡同。



    xL=size(Imp,1);%地图x轴长度800
    yL=size(Imp,2);%地图y轴长度800
    hold on
%     plot(x_I, y_I, 'ro', 'MarkerSize',5, 'MarkerFaceColor','r');
%     plot(x_G, y_G, 'go', 'MarkerSize',5, 'MarkerFaceColor','g');% 绘制起点和目标点
    count=1;%已有的节点数量
    tic; % 开始计时
    reset_interval=10;

    for iter = 1:10000%最多生成3000次随机点

    %     if mod(iter, reset_interval) == 0
    %         % 每隔一定的迭代次数，重置scale_factor为较大值
    %         scale_factor = 1; % 或者其他根据您环境的适当值
    %     else
    %         % 正常更新scale_factor
    %         scale_factor = distance_shengyu / sqrt((x_G - x_I)^2 + (y_G - y_I)^2);
    %     end

        p_rand=[];%数组第一项为横坐标，第二项为纵坐标

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%随机点创新%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
        % 设定一个随着迭代增加而减小的缩放因子
    %     scale_factor = 1 - (count / 500);  % 从 1 逐渐减小到 0，/后数可改
        scale_factor=(distance_shengyu*(a^n))/(sqrt((x_G - x_I)^2 + (y_G - y_I)^2));%从1逐渐减小到0
        if scale_factor>1
            scale_factor=1;
        end
        flag=1;
        while flag
           % 计算随机点坐标，使其随着count增加，范围逐渐缩小
           p_rand(1) = x_G + (ceil(rand() * xL) - x_G) * (1.6*scale_factor);%+号后可改，越大越随机   %特殊线性插值，即在随机x与终点x之间插值，但偏向谁由scale_factor决定 （scale_factor越趋近1越趋向随机点，越趋近0越趋向终点）
           p_rand(2) = y_G + (ceil(rand() * yL) - y_G) * (1.6*scale_factor);
           gridIndex = ceil(p_rand ./ (size(Imp) ./ gridSize)); % 计算随机点所在网格坐标
           if feasiblePoint(ceil(p_rand),dilatedImageInvertedGray) && feasiblePoint(floor(p_rand),dilatedImageInvertedGray) && feasiblePoint([ceil(p_rand(1)) floor(p_rand(2))],dilatedImageInvertedGray) && feasiblePoint([floor(p_rand(1)) ceil(p_rand(2))],dilatedImageInvertedGray)&&(samplingGrid(gridIndex(1), gridIndex(2)) < deadEndThreshold)%若采样点在障碍物内
                flag=0;%随机点没在障碍物内就跳出，否则重新选点
           else
           end
        end
%         plot(p_rand(1), p_rand(2), 'yo', 'MarkerSize',3, 'MarkerFaceColor','y');
        m=m+1;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    %     %Step 1: 在地图中随机采样一个点x_rand
    %     %提示：用（p_rand(1),p_rand(2)）表示环境中采样点的坐标
    %     p_rand(1)=ceil(rand()*xL);%随机采样点横坐标 % rand()生成的是0~1均匀分布的随机数，乘以800再向上取整，数便为[1,800]间的整数
    %     p_rand(2)=ceil(rand()*yL);%随机采样点纵坐标

     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%生长点创新%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
           % 初始化搜索半径
        search_radius = initial_radius; % initial_radius 是你设置的初始搜索半径
        found = false;
        p_near=[];%找最近点
        min_distance = 1000;
        while ~found
            % 存储在搜索半径内且无障碍物的节点
            valid_nodes = [];
            %count存疑
            for i=1:count%提取出随机点半径范围内所有不隔障碍物的节点放到valid_nodes中    （遍历所有已经在树 T 中的节点，若在随机点半径圆内且之间无障碍物，则将该节点添加到 valid_nodes 数组中）
                if isInCircle(p_rand, [T.v(i).x, T.v(i).y], search_radius) && collisionChecking([T.v(i).x, T.v(i).y], p_rand, dilatedImageInvertedGray)
                    valid_nodes = [valid_nodes; T.v(i).x, T.v(i).y, i];%valid_nodes每一行为一个可用节点，3列分别为该节点横纵坐标及索引i（数组连接操作的语法 [A; B] 表示将矩阵 B 作为新的行附加到矩阵 A 的底部。）
                end
            end

            % 如果找到有效节点，则选择最近的
            if ~isempty(valid_nodes)%若随机点半径范围内有不隔障碍物的可用节点则    （如果所检查的数据结构没有任何元素，isempty 返回 true，否则返回 false。）

                nearest_idx = -1;
                for j = 1:size(valid_nodes, 1)%在可用点中找最近的点及对应的最小距离
                    distance = sqrt((valid_nodes(j, 1) - p_rand(1))^2 + (valid_nodes(j, 2) - p_rand(2))^2);
                    if distance < min_distance
                        min_distance = distance;%更新最小距离
                        nearest_idx = j;%更新最近点是在valid_nodes的第几行（对应那行的最后一列即真正的节点索引i）
                    end
                end
                if nearest_idx ~= -1%必然执行，因为执行它的条件就是有可用点，就必然存在最小距离，更新最小距离点索引
                    nearest_node = valid_nodes(nearest_idx, :);%提取valid_nodes最近点所在行的所有列，即nearest_node为最近点所在行的数组，前两列为最近点xy坐标，第三列为节点索引
                    p_near = [nearest_node(1), nearest_node(2)];%最近点坐标
                    index = nearest_node(3);%最近点索引
                    found = true;%已找到最近点
                end
            else%若一定半径内无不隔障碍物的点，则扩大搜索半径
                search_radius = search_radius + delta_radius; % delta_radius 是半径增加的步长
            end
            if search_radius>=300%size(dilatedImageInvertedGray,1)%如果半径已经扩到最大还没有，则不考虑相隔障碍物，按照正常算法生长节点  >=后数可改

    %             found = true;
               for i=1:count
                    distance = sqrt( ( T.v(i).x - p_rand(1) )^2 + ( T.v(i).y - p_rand(2) )^2 );%计算每个点与随机点距离
                    if distance < min_distance
                         min_distance = distance;%更新最小距离
                         index = i;%更新最近点的索引
                    end
               end
               p_near(1) = T.v(index).x;%最邻近节点横坐标=最近节点横坐标
               p_near(2) = T.v(index).y;   
               found = true;
            end        
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %     p_near=[];
    %     %Step 2: 遍历树，从树中找到最近邻近点p_near 
    %     %提示：p_near已经在树T里
    %     min_distance = 1000;%初始默认最小距离
    %     for i=1:count
    %         distance = sqrt( ( T.v(i).x - p_rand(1) )^2 + ( T.v(i).y - p_rand(2) )^2 );%计算每个点与随机点距离
    %         if distance < min_distance
    %             min_distance = distance;%更新最小距离
    %             index = i;%更新最近点的索引
    %         end
    %     end
    %     p_near(1) = T.v(index).x;%最邻近节点横坐标=最近节点横坐标
    %     p_near(2) = T.v(index).y;


        %Step 3: 扩展得到p_new节点
        %提示：注意使用扩展步长Delta
    %     p_new(1) = p_near(1) + round( ( p_rand(1)-p_near(1) ) * Delta/min_distance );%存疑，
    %     p_new(2) = p_near(2) + round( ( p_rand(2)-p_near(2) ) * Delta/min_distance );

    %建议正确的处理方式应该是先判断 min_distance 与 Delta 的关系，如果 min_distance 小于 Delta，则直接将 p_new 设置为 p_rand。如果 min_distance 大于或等于 Delta，才使用原先的计算方法来确定 p_new 的位置。


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%生长点考虑安全距离%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        p_new=[];

        if min_distance <= Delta
            p_new = p_rand;
        else
            p_new(1) = p_near(1) + round( ( p_rand(1) - p_near(1) ) * Delta / min_distance );
            p_new(2) = p_near(2) + round( ( p_rand(2) - p_near(2) ) * Delta / min_distance );
            gridIndex = ceil(p_new ./ (size(Imp) ./ gridSize)); % 计算新节点网格坐标
    %         disp(['Grid Index Before Update: ', num2str(gridIndex)]);
        end
    %%%%%%%%%%%%%%%%%%%%%%%%%跳出局部最优%%%%%%%%%存疑
    %     [isSafe, safePoint] = isSafeDistance(p_near, p_new, Imp, safeDistance);
    %     if (last_safepoint(1)==safePoint(1))&&(last_safepoint(2)==safePoint(2))
    %         n=n+1;%如果两次取的安全点都一样的话说明卡局部最优卡住了，记录卡住的次数来调整scale_factor
    %     else
    %         last_safepoint=safePoint;
    %         n=0;
    %     end
    %     p_new = temp_p_new;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


        %检查节点是否是collision-free
        if ~collisionChecking(p_near,p_new,dilatedImageInvertedGray) %若新节点有碰撞则直接进入下个大循环重新采样
            n=n+1;
            deadEndGrid(gridIndex(1), gridIndex(2)) = deadEndGrid(gridIndex(1), gridIndex(2)) + 1;%所在网格失败次数+1
            disp(['n: ', num2str(n)]);
           continue;
        else
            samplingGrid(gridIndex(1), gridIndex(2)) = samplingGrid(gridIndex(1), gridIndex(2)) + 1;%所在网格节点计数+1
            n=0;
    %         disp(['Sampling Grid Value Before Update: ', num2str(samplingGrid(gridIndex(1), gridIndex(2)))]);
        end
        disp(['n: ', num2str(n)]);
        count=count+1;

    %%%%%%%%%%%%%%%%%%%%%%%%%%采样点按照节点距离终点最近距离趋向于终点用%%%%%%%%%
        distance_pnew_goal=sqrt((x_G - p_new(1))^2 + (y_G - p_new(2))^2);%计算新点与终点间距离，并更新最短距离
        if distance_pnew_goal<distance_shengyu
            distance_shengyu=distance_pnew_goal;%distance_shengyu从起点终点距离逐渐趋向0
        end
        if distance_shengyu<=100 
            deadEndThreshold = 20;
        end;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %Step 4: 将p_new插入树T 
        %提示：新节点p_new的父节点是p_near
        T.v(count).x = p_new(1);         
        T.v(count).y = p_new(2); 
        T.v(count).xPrev = p_near(1);%产生生长点的那个最临近点是生长点的父节点    
        T.v(count).yPrev = p_near(2);
    %     T.v(count).dist = min_distance; %存疑，应该是新节点与父节点距离 ，而不是父节点与随机点距离
        T.v(count).dist = sqrt((p_new(1) - p_near(1))^2 + (p_new(2) -p_near(2))^2); %改


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if collisionChecking(p_new, [x_G, y_G], dilatedImageInvertedGray)%如果pnew与终点之间无障碍物则直接连接
        % 如果p_new到目标点之间无障碍
%         elapsedTime = toc; % 结束计时并获取经过的时间
        T.v(count+1).x = x_G; % 将目标点作为新的节点加入树中
        T.v(count+1).y = y_G;
        T.v(count+1).xPrev = p_new(1);
        T.v(count+1).yPrev = p_new(2);
        T.v(count+1).dist = sqrt((x_G - p_new(1))^2 + (y_G - p_new(2))^2); % 计算距离
        T.v(count+1).indPrev = count; % 将当前节点设置为新节点的父节点

        % 更新路径绘制
%         plot(x_G, y_G, 'go', 'MarkerSize',5, 'MarkerFaceColor','g'); % 绘制目标点
%         line([p_new(1), x_G], [p_new(2), y_G], 'Marker','.','LineStyle','-','color','r'); % 连接新节点和目标点
        break; % 跳出循环
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %Step 5:检查是否到达目标点附近 
        %提示：注意使用目标点阈值Thr，若当前节点和终点的欧式距离小于Thr，则跳出当前for循环
        new_distance = sqrt( ( p_new(1) - x_G )^2 + ( p_new(2) - y_G )^2 );
        [isSafe, safePoint] = isSafeDistance(p_new, [x_G,y_G], Imp, safeDistance);
        if new_distance <= Thr && collisionChecking(p_new,[x_G,y_G],dilatedImageInvertedGray)&&isSafe%小创新，终点加入障碍物检测
%             elapsedTime = toc; % 结束计时并获取经过的时间
%             plot(p_new(1), p_new(2), 'bo', 'MarkerSize',2, 'MarkerFaceColor','b'); % 绘制p_new，p_new(1), p_new(2)是点的x和y坐标。点的样式被设定为蓝色圆圈（'bo'），MarkerSize设定为2（点的大小），MarkerFaceColor设定为蓝色（点的填充颜色）
%             line( [p_new(1) p_near(1)], [p_new(2) p_near(2)], 'Marker','.','LineStyle','-'); %连接p_near和p_new，线段的起点是p_near（最近邻节点），终点是p_new（新节点）。[p_new(1) p_near(1)]和[p_new(2) p_near(2)]分别是线段在x和y轴上的坐标。'Marker','.'指定线段上的点的样式，'LineStyle','-'指定线段的样式为实线。
%             line( [x_G p_new(1)], [y_G p_new(2)], 'Marker','.','LineStyle','-'); %连接Goal和p_new
            break;
        end

        %Step 6:将p_near和p_new之间的路径画出来
        %提示 1：使用plot绘制，因为要多次在同一张图上绘制线段，所以每次使用plot后需要接上hold on命令
        %提示 2：在判断终点条件弹出for循环前，记得把p_near和p_new之间的路径画出来
%         plot(p_new(1), p_new(2), 'bo', 'MarkerSize',2, 'MarkerFaceColor','b'); % 绘制p_new
%         line( [p_new(1) p_near(1)], [p_new(2) p_near(2)], 'Marker','.','LineStyle','-'); %连接p_near和p_new
        hold on;

    %     pause(0.01); %暂停0.1s，使得RRT扩展过程容易观察
        deadEndGrid(deadEndGrid >= deadEndThreshold) = deadEndThreshold;
    end

    %% 画出路径
    T_LIST = zeros(size(T.v, 2), 5);%size(T.v, 2)：返回 T.v 在第二维度的大小。由于 T.v 是一个一维结构体数组（即一个向量），因此 size(T.v, 2) 实际上返回的是该数组中元素的数量，即树中节点的总数。
    for i=1:size(T.v, 2)%T_LIST每行为1个节点，每列依次为这个节点的 x 坐标、y 坐标、它的父节点的 x 坐标、父节点的 y 坐标和节点的索引。
        T_LIST(i,1) = T.v(i).x;
        T_LIST(i,2) = T.v(i).y;
        T_LIST(i,3) = T.v(i).xPrev;
        T_LIST(i,4) = T.v(i).yPrev;
        T_LIST(i,5) = i;
    end

    path = [];%数组用来存储从起点到终点的路径，初始为空数组，之后提到几行几列它就动态变为几行几列
              %path第一行为最后一个点，第二行为倒第二个点......
    path_count = 1;%记录路径中点的数量
    path(path_count,1) = x_G;%路径数组的第一行（即第一个路径点）第一列（即横坐标）=终点横坐标
    path(path_count,2) = y_G;
    path_count = path_count + 1;%第二路径点
    path(path_count,1) = p_new(1);%路径数组的第二行（即第二个路径点）第一列（即横坐标）=最后一个生长点横坐标
    path(path_count,2) = p_new(2);
    n_index = node_index(T_LIST, p_new(1), p_new(2));%获得最后一个生长点的索引（T_LIST中第几个节点）
    path_count = path_count + 1;%第三路径点
    path(path_count,1) = T_LIST(i,3);%路径数组的第三行（即第三个路径点）第一列（即横坐标）= 最后一个生长点的父节点横坐标
    path(path_count,2) = T_LIST(i,4);
    while path(path_count,1) ~= x_I || path(path_count,2) ~= y_I%从倒数第二个生长点（最后一个生长点的父节点）开始，当前路径点只要不是起点就一直循环
        new_n_index = node_index(T_LIST, path(path_count,1), path(path_count,2));%根据节点坐标获得此节点索引
        path_count = path_count + 1;
        path(path_count,1) = T_LIST(new_n_index,3);%根据此节点索引得到此节点父节点坐标
        path(path_count,2) = T_LIST(new_n_index,4);
        n_index = new_n_index;
    end

    % for i=size(path,1)-1 :-1: 1%正着连：size(path,1) 返回 path 数组的行数，即路径中点的总数。循环从 size(path,1)-1 开始，即path倒数第二行即第二个点开始，i+1为第一个点
    %     line( [path(i,1) path(i+1,1)], [path(i,2) path(i+1,2)], 'Marker','.','LineStyle','-','color','r'); %连接p_near和p_new
    %     hold on;
    %     pause(0.1); %暂停0.1s，使得RRT扩展过程容易观察
    % end
    for i=1 :1: size(path,1)-1%改进：倒着连，从path(1)到path(2)即倒第一个点到倒第二个点开始连线，连到path倒第二行到最后一行即第二个点到第一个点结束
        line( [path(i,1) path(i+1,1)], [path(i,2) path(i+1,2)], 'Marker','.','LineStyle','-','color','r'); %连接p_near和p_new
        hold on;
    %     pause(0.01); %暂停0.1s，使得RRT扩展过程容易观察
    end






    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  二次优化  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % 优化路径

    currentNode = [x_I, y_I];%优化路径的起始点为起点
    optimizedPath = [currentNode];%存储优化后的路径,路径第一个点必然是起点，先加里
    furthest_i=size(path, 1);
    while ~isequal(currentNode, [x_G, y_G])%一直执行，直到 currentNode（当前节点）到达目标点 [x_G, y_G]
        furthestNode = currentNode;%furthestNode 用来存储从 currentNode 开始沿路径可安全到达的最远节点。
        for i = furthest_i:-1:1
            [isSafe, safePoint] = isSafeDistance(currentNode, path(i, :), Imp, safeDistance);
            if collisionChecking(currentNode, path(i, :), dilatedImageInvertedGray) %&& isSafe
                furthestNode = path(i, :);
                furthest_i=i;
            else
                break;
            end
        end
        optimizedPath = [optimizedPath; furthestNode];%这里将找到的最远安全节点 furthestNode 添加到优化路径 optimizedPath 中，并将 currentNode 更新为 furthestNode 以用于下一次迭代

        currentNode = furthestNode;%将 currentNode 更新为 furthestNode 以用于下一次迭代。
        disp(['Current Node: ', num2str(currentNode)]);
        disp(['Furthest Node: ', num2str(furthestNode)]);

    end
    samplingGrid
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 基于 optimizedPath 进行插值生成 densePath
densePath = []; % 初始化插值后的路径
for i = 1:size(optimizedPath, 1)-1
    segmentStart = optimizedPath(i,:);
    segmentEnd = optimizedPath(i+1,:);
    % 线性插值来生成中间点
    interpolatedX = linspace(segmentStart(1), segmentEnd(1), 10);%第三个参数为插值密度
    interpolatedY = linspace(segmentStart(2), segmentEnd(2), 10);
    segmentPoints = [interpolatedX' interpolatedY'];
    densePath = [densePath; segmentPoints(1:end-1,:)]; % 避免重复添加终点
end
densePath = [densePath; optimizedPath(end,:)]; % 确保最终路径包括终点
% plot(densePath(:,1), densePath(:,2), 'bo', 'MarkerSize', 2, 'MarkerFaceColor', 'b');




cutPoint = [0, 0]; 
finalPath = [densePath(1, :)]; % 开始于起点
i = 2; % 从densePath的第二个点开始

while i <= size(densePath, 1)
    currentPoint = finalPath(end, :); % 当前点始终是finalPath的最后一个点
    nextPathPoint = densePath(i, :); % 下一个路径点是densePath中的点
    
    % 检查从当前点到下一个路径点是否有碰撞
    if ~collisionChecking(currentPoint, densePath(i, :), dilatedImageInvertedGray)
        % 如果有碰撞，找到从当前点到上一个成功连接的路径点之间的切点
        cutPoint = findClosestPointToObstacle(currentPoint, densePath(i-1, :), dilatedImageInvertedGray, 30);
        k=i
        finalPath = [finalPath; cutPoint]; % 添加切点到最终路径
      
                % 若cutPoint与densePath(i, :)的连线与障碍物发生了碰撞
        if ~collisionChecking(cutPoint, densePath(i, :), dilatedImageInvertedGray)
            % 定义生长方向，这里假设是从cutPoint回退到densePath(i-1, :)
            growDirection = densePath(i-1, :) - cutPoint;
            % 单位化生长方向
            growDirectionNormalized = growDirection / norm(growDirection);
            % 生长步长，根据需要调整
            growStepSize = 20; 
            
            while true
                % 尝试在生长方向上移动一步
                growPoint = cutPoint + growStepSize * growDirectionNormalized;
                % 检查growPoint到densePath(i, :)是否发生碰撞
                if collisionChecking(growPoint, densePath(i, :), dilatedImageInvertedGray)
                    % 如果没有发生碰撞，将growPoint加入finalPath并跳出循环
                    finalPath = [finalPath; growPoint];
                    
                    break;
                else
                    % 如果仍然发生碰撞，更新cutPoint为新的growPoint并继续循环
                    cutPoint = growPoint;
                end
            end
        else
            % 如果cutPoint到densePath(i, :)没有碰撞，直接将cutPoint加入finalPath
%             finalPath = [finalPath; cutPoint];
%         plot(cutPoint(1), cutPoint(2), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r'); % 使用红色圆圈标记切点

        end

        
        
        
%         plot(cutPoint(1), cutPoint(2), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r'); % 使用红色圆圈标记切点
        % 更新i以跳过当前发生碰撞的点，继续检查后续点
%         i = i + 2;
    else
%         finalPath = [finalPath; nextPathPoint]; % 没有碰撞直接添加下一个路径点
        i = i + 1; % 移动到下一个路径点
        
    end
%     plot(finalPath(:,1), finalPath(:,2), 'm-', 'LineWidth', 2); % 使用紫色线条

    % 检查是否已经可以直接连接到终点
    if collisionChecking(finalPath(end, :), densePath(end, :), dilatedImageInvertedGray)
            elapsedTime = toc; % 结束计时并获取经过的时间

        finalPath = [finalPath; densePath(end, :)]; % 直接连接到终点
        break; % 完成路径构建
    end
end
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
    
    % 画出优化后的路径
    total_length = 0;%二次优化后总长度
    for i = 1:size(finalPath, 1) - 1
        line([finalPath(i, 1), finalPath(i + 1, 1)], [finalPath(i, 2), finalPath(i + 1, 2)], 'Color', 'k', 'LineWidth', 2);%二次优化连线
        segment_length = sqrt((finalPath(i+1, 1) - finalPath(i, 1))^2 + (finalPath(i+1, 2) - finalPath(i, 2))^2);%二次优化每段长度
        total_length = total_length + segment_length;%二次优化后总长度
    end
    
    nodes_before = size(path, 1);
    num_of_nodes = size(finalPath, 1);%二次优化后节点数（算起点终点）

%     disp(['time: ', num2str(elapsedTime)]);
%     disp(['(二次优化后)路径长度: ', num2str(total_length)]);
%     disp(['迭代次数: ',num2str(m)]);
%     disp(['总节点数: ', num2str(count)]);
%     disp(['(二次优化前)路径节点数: ', num2str(nodes_before)]);
%     disp(['(二次优化后)路径节点数: ', num2str(num_of_nodes)]);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    results(cishu, :) = [elapsedTime, total_length, m, count, nodes_before,num_of_nodes];%将每次实验结果放到结果矩阵第实验次数行
    % 将结果写入 Excel 文件
    
    filename = 'imRRT_sy24_final(100)2.xlsx';
xlswrite(filename, results, 'Sheet1', 'A2');%将结果矩阵放到exel的A2开始单元

% 可选：在 Excel 文件中添加表头
header = {'Time', 'Path Length', 'Iterations', 'Total Nodes','Nodes Before',  'Path Nodes'};
xlswrite(filename, header, 'Sheet1', 'A1');%表头放在exel的A1开始单元

end


