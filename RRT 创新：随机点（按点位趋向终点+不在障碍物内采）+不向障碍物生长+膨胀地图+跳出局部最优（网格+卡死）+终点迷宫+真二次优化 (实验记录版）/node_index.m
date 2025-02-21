function n_index = node_index(T_LIST,xval,yval)%在 T_LIST 数组中查找特定坐标 (xval, yval) 的节点，并返回该节点的索引。
    %This function returns the index of the location of a node in the T_LIST
    i=1;%初始化索引变量
    while ( T_LIST(i,1) ~= xval || T_LIST(i,2) ~= yval )%只要第i个节点x或y坐标与要找的不一样就换i+1节点继续找，若都一样说明找到则退出循环
        i=i+1;
    end
    n_index=i;%得到要找的特定坐标节点是几号节点
end