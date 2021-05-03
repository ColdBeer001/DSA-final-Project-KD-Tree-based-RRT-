function [KDTree, RebuildNode] = insert(KDTree, this)
target= this.node(:,this.last_id);
currentNode = KDTree.rootindex; 
RebuildNode=0;

KDTree.size(currentNode)=KDTree.size(currentNode)+1;

while (target(KDTree.axis(currentNode))<=KDTree.boundary(currentNode) && KDTree.lchild(currentNode)~=0) || (target(KDTree.axis(currentNode))>KDTree.boundary(currentNode) && KDTree.rchild(currentNode)~=0)
   if  target(KDTree.axis(currentNode))<=KDTree.boundary(currentNode) && KDTree.lchild(currentNode)~=0
   currentNode=  KDTree.lchild(currentNode);
   KDTree.size(currentNode)=KDTree.size(currentNode)+1;
        if KDTree.size(KDTree.father(currentNode))*KDTree.alpha <=  KDTree.size(currentNode)  && RebuildNode==0
            RebuildNode=KDTree.father(currentNode);
        end

    elseif  target(KDTree.axis(currentNode))> KDTree.boundary(currentNode) && KDTree.rchild(currentNode)~=0
        currentNode=  KDTree.rchild(currentNode);
        KDTree.size(currentNode)=KDTree.size(currentNode)+1;
            if KDTree.size(KDTree.father(currentNode))*KDTree.alpha <=  KDTree.size(currentNode) && RebuildNode==0
                RebuildNode=KDTree.father(currentNode);
            end
    end
end

if  target(KDTree.axis(currentNode))<=KDTree.boundary(currentNode) && KDTree.lchild(currentNode)==0
    KDTree.lchild(currentNode)=this.last_id;
    KDTree.axis (this.last_id) = mod(KDTree.axis (currentNode),3)+1;
    KDTree.boundary (this.last_id) = target(KDTree.axis (this.last_id) );
    KDTree.depth (this.last_id) = KDTree.depth (currentNode)+1;
    KDTree.size(this.last_id)= 1;
    KDTree.father(this.last_id)=currentNode;

elseif target(KDTree.axis(currentNode))>KDTree.boundary(currentNode) && KDTree.rchild(currentNode)==0
    KDTree.rchild(currentNode)=this.last_id;
    KDTree.axis (this.last_id) = mod(KDTree.axis (currentNode),3)+1;
    KDTree.boundary (this.last_id) = target(KDTree.axis (this.last_id) );
    KDTree.depth (this.last_id) = KDTree.depth (currentNode)+1;
    KDTree.size(this.last_id)= 1;
    KDTree.father(this.last_id)=currentNode;
end




    