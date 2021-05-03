function SubTree=  Drebuild (this, data, axis)


TempNode =  Generatenode (this, data, axis);

SubTree.root=TempNode.index;

SubTree.node=zeros(1,length(data));
SubTree.boundary=zeros(1,length(data));
SubTree.axis=zeros(1,length(data));
SubTree.depth=zeros(1,length(data));
SubTree.depth(1)=1;

SubTree.size=zeros(1,length(data));

SubTree.father=zeros(1,length(data));   
SubTree.lchild=zeros(1,length(data));

SubTree.rchild=zeros(1,length(data));

[SubTree, ~] =  match (TempNode, SubTree, 1, 1);



