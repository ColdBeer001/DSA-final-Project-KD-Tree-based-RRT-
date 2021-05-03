function [currentNearest, currentNearestDist,MUT_in] = kd_search(KDTree, this, target,K, MUT_in)

KDTree.visited=zeros(1,this.max_ite);  %Reset
currentNearest=zeros(1,K);

currentNode = KDTree.rootindex; %the id of the current node, now is the root
currentNode = search_down(KDTree,currentNode,target);% from currentNode search down until bottom
KDTree.visited(currentNode) = 1;

 
     
CurrentK=1; 
currentNearestDist(CurrentK) = sum((this.node(:,currentNode)-target).^2);   % Current smallest distance
MUT_in=MUT_in+3;
currentNearest(CurrentK) = currentNode;
Biggest=find(currentNearestDist==max(currentNearestDist));
   
while KDTree.rootindex ~= currentNode
    
    currentNode = KDTree.father(currentNode);
    if KDTree.visited(currentNode) == 0
        KDTree.visited(currentNode) = 1;%Marked as visited   
        temp = sum((this.node(:,currentNode)-target).^2);
        MUT_in=MUT_in+3;
        if  CurrentK < K
            CurrentK=CurrentK+1;
            currentNearest(CurrentK) = currentNode;
            currentNearestDist(CurrentK) = temp;
            Biggest=find(currentNearestDist==max(currentNearestDist));
        
        elseif temp<max(currentNearestDist) && CurrentK >= K
            Biggest=find(currentNearestDist==max(currentNearestDist));
            currentNearest(Biggest) = currentNode;
            currentNearestDist (Biggest) = temp;
        end
        
        dis = abs(KDTree.boundary(currentNode)-target(KDTree.axis(currentNode)));   %Distance between the boundary
        if dis <= currentNearestDist (Biggest)
       %if the distance is smaller than the smallest dist, then it's possible that there is potential closer point on the other side, jump to the other side
             if KDTree.lchild(currentNode) ~= 0 && KDTree.visited(KDTree.lchild(currentNode))==1 %?
                if KDTree.rchild(currentNode) ~= 0 % Current node's left child and if also has right child, search right child
                    currentNode = KDTree.rchild(currentNode);
                    currentNode = search_down(KDTree,currentNode,target); 
                    KDTree.visited(currentNode) = 1;%Mark as visited
                    
                    temp = sum((this.node(:,currentNode)-target).^2);
                    MUT_in=MUT_in+3;
                    if  CurrentK <K
                    CurrentK=CurrentK+1;
                    currentNearest(CurrentK) = currentNode;
                    currentNearestDist(CurrentK) = temp;
                    Biggest=find(currentNearestDist==max(currentNearestDist));
        
                    elseif temp<max(currentNearestDist) && CurrentK >= K
                    Biggest=find(currentNearestDist==max(currentNearestDist));
                    currentNearest(Biggest) = currentNode;
                    currentNearestDist (Biggest) = temp;
                    end
                end
             elseif KDTree.rchild(currentNode) ~= 0 && KDTree.visited(KDTree.rchild(currentNode))==1 %?
                if KDTree.lchild(currentNode) ~= 0 % Current node's left child and if also has right child, search right child
                currentNode = KDTree.lchild(currentNode);
                currentNode = search_down(KDTree,currentNode,target); 
                KDTree.visited(currentNode) = 1;%mark as visied
                    
                temp = sum((this.node(:,currentNode)-target).^2);
                MUT_in=MUT_in+3;
                if  CurrentK <K
                CurrentK=CurrentK+1;
                currentNearest(CurrentK) = currentNode;
                currentNearestDist(CurrentK) = temp;
        
                elseif temp<max(currentNearestDist) && CurrentK >= K
                Biggest=find(currentNearestDist==max(currentNearestDist));
                currentNearest(Biggest) = currentNode;
                currentNearestDist (Biggest) = temp;
                end
            end
        end
    end
    end
end
            