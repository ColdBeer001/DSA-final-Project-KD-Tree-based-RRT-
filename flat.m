function data = flat(KDTree, RebuildNode, data)

currentNode=RebuildNode;
i=1;
data(i)=currentNode;


for a = 1: KDTree.size(RebuildNode)
    currentNode=data(a);
    if KDTree.lchild(currentNode) ~= 0
        i=i+1;
        data(i)=KDTree.lchild(currentNode);
    end
    if KDTree.rchild(currentNode) ~= 0
        i=i+1;
        data(i)=KDTree.rchild(currentNode);
    end
end
    
    
    
    
