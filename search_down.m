function node = search_down(KDTree, currentNode,target)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Starting from the current node to search down until leaf node
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while  KDTree.rchild(currentNode) ~= 0 || KDTree.rchild(currentNode) ~= 0
    if KDTree.rchild(currentNode) == 0
        currentNode=KDTree.lchild(currentNode);
    elseif KDTree.lchild(currentNode) == 0
        currentNode=KDTree.rchild(currentNode);
    else 
        axis = KDTree.axis(currentNode);
        if target(axis)<=KDTree.boundary(currentNode)
            currentNode=KDTree.lchild(currentNode);
        else
             currentNode=KDTree.rchild(currentNode);
        end
    end
end
node = currentNode;