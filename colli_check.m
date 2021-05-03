      function free =colli_check(this,new_node,parent_index)
        % collision check function
        % return true if path is free, else, false
        % parent_index=int32(parent_index);
        free=true;
        for onum=1:this.obstacle.obs_num
           if(intersec(this.obstacle,onum, ...
               [new_node,this.node(:,parent_index)])==1)
               free=false;
               return;
           end
        end
      end