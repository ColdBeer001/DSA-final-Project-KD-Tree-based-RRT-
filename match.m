function [SubTree, i_n] =  match (TempNode, SubTree, i, i_n)

SubTree.node(i)=TempNode.index;
SubTree.boundary(i)=TempNode.boundary;
SubTree.axis(i)=TempNode.axis;

SubTree.size(i)=TempNode.size;

if TempNode.lchild.index ~=0 
    SubTree.lchild(i)=TempNode.lchild.index;
    i_n=i_n+1;
    SubTree.father(i_n)=TempNode.index;
    SubTree.depth(i_n)=SubTree.depth(i)+1;
    [SubTree, i_n] = match (TempNode.lchild, SubTree, i_n,i_n);
end
    
if TempNode.rchild.index ~=0
    SubTree.rchild(i)=TempNode.rchild.index;
    i_n=i_n+1;
    SubTree.father(i_n)=TempNode.index;
    SubTree.depth(i_n)=SubTree.depth(i)+1;
    [SubTree, i_n] = match (TempNode.rchild, SubTree, i_n,i_n);
end
