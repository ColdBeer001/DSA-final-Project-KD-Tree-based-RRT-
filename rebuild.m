function KDTree = rebuild(KDTree, this, RebuildNode)
if  RebuildNode ==0
   KDTree=KDTree; 
end
if RebuildNode ~=0
    depth=KDTree.depth(RebuildNode);
    data = zeros(1, KDTree.size(RebuildNode));
    data = flat(KDTree, RebuildNode, data);
    SubTree = Drebuild (this,data, KDTree.axis(RebuildNode));
    if KDTree.father(RebuildNode)==0
        KDTree.rootindex=SubTree.root;
        for i = 1:length(data)
            KDTree.boundary(SubTree.node(i))=SubTree.boundary(i);
            KDTree.axis(SubTree.node(i))=SubTree.axis(i);
            KDTree.depth(SubTree.node(i))=SubTree.depth(i)-1+depth;
            KDTree.size(SubTree.node(i))=SubTree.size(i);
            KDTree.father(SubTree.node(i))=SubTree.father(i);
            KDTree.lchild(SubTree.node(i))=SubTree.lchild(i);
            KDTree.rchild(SubTree.node(i))=SubTree.rchild(i);
        end
    elseif KDTree.lchild(KDTree.father(RebuildNode))==RebuildNode
        father=KDTree.father(RebuildNode);
        for i = 1:length(data)
            KDTree.boundary(SubTree.node(i))=SubTree.boundary(i);
            KDTree.axis(SubTree.node(i))=SubTree.axis(i);
            KDTree.depth(SubTree.node(i))=SubTree.depth(i)-1+depth;
            KDTree.size(SubTree.node(i))=SubTree.size(i);
            KDTree.father(SubTree.node(i))=SubTree.father(i);
            KDTree.lchild(SubTree.node(i))=SubTree.lchild(i);
            KDTree.rchild(SubTree.node(i))=SubTree.rchild(i);
        end
        KDTree.lchild(father)=SubTree.root;
        KDTree.father(SubTree.root)=father;
        
    elseif  KDTree.rchild(KDTree.father(RebuildNode))==RebuildNode
        father=KDTree.father(RebuildNode);
        for i = 1:length(data)
            KDTree.boundary(SubTree.node(i))=SubTree.boundary(i);
            KDTree.axis(SubTree.node(i))=SubTree.axis(i);
            KDTree.depth(SubTree.node(i))=SubTree.depth(i)-1;
            KDTree.size(SubTree.node(i))=SubTree.size(i);
            KDTree.father(SubTree.node(i))=SubTree.father(i);
            KDTree.lchild(SubTree.node(i))=SubTree.lchild(i);
            KDTree.rchild(SubTree.node(i))=SubTree.rchild(i);
        end
        KDTree.rchild(father)=SubTree.root;
        KDTree.father(SubTree.root)=father;
    end
    
end