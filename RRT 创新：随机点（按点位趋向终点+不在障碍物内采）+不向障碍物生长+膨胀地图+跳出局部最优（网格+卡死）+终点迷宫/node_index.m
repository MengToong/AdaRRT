function n_index = node_index(T_LIST,xval,yval)%�� T_LIST �����в����ض����� (xval, yval) �Ľڵ㣬�����ظýڵ��������
    %This function returns the index of the location of a node in the T_LIST
    i=1;%��ʼ����������
    while ( T_LIST(i,1) ~= xval || T_LIST(i,2) ~= yval )%ֻҪ��i���ڵ�x��y������Ҫ�ҵĲ�һ���ͻ�i+1�ڵ�����ң�����һ��˵���ҵ����˳�ѭ��
        i=i+1;
    end
    n_index=i;%�õ�Ҫ�ҵ��ض�����ڵ��Ǽ��Žڵ�
end