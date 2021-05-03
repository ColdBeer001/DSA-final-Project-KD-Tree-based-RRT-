clear all; close all; clc;clear;

%% this / Root/ Goal / Obstacle
    this=struct;                     
    this.root=[-20;-20;-20];         % root point
    this.x_bound=[-25,25];           % x_y_z boundary
    this.y_bound=[-25,25];
    this.z_bound=[-25,25];
    this.goal=[18;18;18];            % goal point
    this.goal_r=2;                   % goal region radius
    % obstacle structure: center point + width + length + height
    this.obstacle=struct;
    this.obstacle.obs_num=6;         
    cent=[-10,-10,-15;0,0,0;12,12,8;-15,15,5;15,-15,-15;15,-15,12];
    this.obstacle.center=cent;
    this.obstacle.width_x=[10,15,10,10,15,6];
    this.obstacle.length_y=[8,10,10,15,15,6];
    this.obstacle.height_z=[15,15,15,20,15,21];
    
    %% Basic Setting
    max_ite=5000;   % max iteration time (max node number)
    this.max_ite=max_ite;
    % Step
    max_cost=4;   % max cost value each step (Squared-Euclidean dist here)
    this.goal_r=2;                   % goal region radius
    %
    max_nbor=10;   % max neighborhood node number
    nearby_r=20;   % nearby searching radius square

    %???
    
    %% Tree Structure
        this.node=zeros(3,max_ite);
        this.node(:,1)=this.root;
        
        this.cost=zeros(1,max_ite);
        this.parent=zeros(1,max_ite);
        
        %Initial Setting
        this.last_id=1;
        this.rewire_n=0;
        this.best_cost=Inf;
        this.goal_idx=0;
        this.reached=false;
        this.cur_ite=0; % current iteration time 
        
%%   KDTree Initial
        KDTree.rootindex = 1;
        KDTree.alpha=0.75;
        
        %Node value
        %KDTree.node=zeros(1,max_ite);
        %KDTree.node(1)=1;
        
        KDTree.boundary=zeros(1,max_ite);
        KDTree.boundary(1)=-20;

        KDTree.axis=zeros(1,max_ite);
        KDTree.axis(1)=1;
        
        KDTree.depth=zeros(1,max_ite);
        KDTree.depth(1)=1;
        
        KDTree.size=zeros(1,max_ite);
        KDTree.size(1)=1;
        
        KDTree.father=zeros(1,max_ite);
        KDTree.lchild=zeros(1,max_ite);
        KDTree.rchild=zeros(1,max_ite);
        KDTree.visited=zeros(1,max_ite);
        
        
         tic
         MUTper500I=zeros(1,100);
         Oldper500I=zeros(1,100);
         check=1;
     MUTperI=zeros(1,max_ite);
     Old=zeros(1,max_ite);
     
 MUT=0;
 Oldone=0
for i=1:max_ite    
                    dont=0;
%% Random node
        new_node=zeros(3,1);
        new_node(1)=abs(this.x_bound(1)-this.x_bound(2))*rand+this.x_bound(1); %x
        new_node(2)=abs(this.y_bound(1)-this.y_bound(2))*rand+this.y_bound(1); %y
        new_node(3)=abs(this.z_bound(1)-this.z_bound(2))*rand+this.z_bound(1); %z
        
%% find nearest node
        [nearest_idx,nd, MUT] = kd_search(KDTree, this, new_node,1,MUT);

        nearest_node=this.node(:,nearest_idx);
%% New node
          dist=sum((nearest_node-new_node).^2);
          if(dist>=max_cost)
             direct=new_node-nearest_node;
             rate=abs(sqrt(max_cost/dist));
             steer_node=nearest_node + direct*rate;
          else
             steer_node=new_node;
          end
          this.cur_ite=this.cur_ite+1;


if(colli_check(this,steer_node,nearest_idx)==true) % check whether collison
%% Find neighbor
          
          [currentNearest, currentNearestDist, MUT]=kd_search(KDTree, this, steer_node,10,MUT);
          neighbor_idx=currentNearest(find(currentNearestDist<20&currentNearestDist>0));
          if(isempty(neighbor_idx))
              neighbor_idx=1;
          end   % For the first time
          
%% Find Best Parent
          min_cost=this.cost(nearest_idx)+sum((this.node(:,nearest_idx)-steer_node).^2); %改了 用的steer
          min_idx=nearest_idx;
          for j=1:numel(neighbor_idx)
             if(colli_check(this,steer_node,neighbor_idx(j))==true)
                 try_cost=this.cost(neighbor_idx(j))+ sum((this.node(:,neighbor_idx(j))-steer_node).^2);
                 if(try_cost<min_cost)
                    min_cost=try_cost;
                    min_idx=neighbor_idx(j);
                 end
             end
          end
          
          
%% Add the node to the tree
        % add a new node to RRT tree,
        Laode=this.last_id;
        Oldone=Oldone+Laode;
        this.last_id=this.last_id+1;
        this.node(:,this.last_id)=steer_node;
        this.parent(this.last_id)=min_idx;
        this.cost(this.last_id)=min_cost;

%% Rewire
          for k=1:numel(neighbor_idx)
              if(neighbor_idx(k)==min_idx)
                  continue;
              end
              origin_cost=this.cost(neighbor_idx(k));
              delta_cost=sum((this.node(:,neighbor_idx(k))-this.node(:,this.last_id)).^2);
              temp_cost=delta_cost+this.cost(this.last_id);
              if( temp_cost<origin_cost && colli_check(this,this.node(:,this.last_id),neighbor_idx(k))==true)
                 this.parent(neighbor_idx(k))=this.last_id; 
                 this.cost(neighbor_idx(k))=temp_cost;
                 this.rewire_n=this.rewire_n+1;
              end
          end
          
          [KDTree, RebuildNode] = insert(KDTree, this);
          KDTree = rebuild(KDTree, this, RebuildNode);
end   
       

%   if(mod(i,500)==0)
% 
%        
%        MUTper500I(check)=MUT/500;
%        MUT=0;
%        Oldper500I(check)=Oldone/500;
%        Oldone=0;
%        
%        check=check+1;
% 
%     end  
%  
% 
% 
%              MUTperI(i)=MUT;
%              Old(i)=(this.last_id-1)*6;
      if(mod(i,500)==0)
%          toc
%          disp(['Running time: ',num2str(toc)]);
         % check whether goal region is reached and choose the best path
         % to the goal region
         % this.reached=false;
         comp_temp=sum((this.node-this.goal).^2);
         comp_idx=comp_temp<this.goal_r^2;
         sort_idx=find(comp_idx);
         if(numel(sort_idx)~=0)
             this.reached=true;
         end
         for i=1:numel(sort_idx)
             if(this.cost(sort_idx(i))<this.best_cost)
                 this.best_cost=this.cost(sort_idx(i));
                 this.goal_idx=sort_idx(i);
             end
             % this.best_dis=min(opt_dis,this.cost(sort_idx(i)));
         end
         if(this.reached==true)
            fprintf('Feasible path found after %d-th iteration, min_cost =%d\n',this.cur_ite,this.best_cost);
         else
            fprintf('No feasible path after %d-th iteration\n', this.cur_ite);
         end
    end 
end

       purple=[171,154,192]/255;
        orange=[255,242,204]/255;
        ored=[183,71,42]/255;
        sky=[0,176,240]/255;
        grey=[189,189,189]/255;
        yel=[254,244,180]/255;
        
        obs_col=yel;
        path_col=sky;
        result_col=ored;
      % clear figure
        clf;

      % plot obstacle first
        vertex=zeros(3,8);
        o_num=this.obstacle.obs_num;
          for i=1:o_num
            % unfold cuboid obstacle
            obs_line=[this.obstacle.width_x(i);this.obstacle.length_y(i);this.obstacle.height_z(i)]/2;
            vertex(:,1)=this.obstacle.center(i,:)'+ obs_line.*[-1;-1;-1];
            vertex(:,5)=this.obstacle.center(i,:)'+ obs_line.*[-1;-1; 1];
            vertex(:,2)=this.obstacle.center(i,:)'+ obs_line.*[-1; 1;-1];
            vertex(:,6)=this.obstacle.center(i,:)'+ obs_line.*[-1; 1; 1];
            vertex(:,3)=this.obstacle.center(i,:)'+ obs_line.*[ 1; 1;-1];
            vertex(:,7)=this.obstacle.center(i,:)'+ obs_line.*[ 1; 1; 1];
            vertex(:,4)=this.obstacle.center(i,:)'+ obs_line.*[ 1;-1;-1];
            vertex(:,8)=this.obstacle.center(i,:)'+ obs_line.*[ 1;-1; 1];
            
            %{ 
                vertex(:,4)=this.obstacle.center(o_num,:)'+ ...
            [this.obstacle.width_x(o_num)/2;-this.obstacle.length_y(o_num)/2;-this.obstacle.height_z(o_num)/2];
                vertex(:,8)=this.obstacle.center(o_num,:)'+ ...
            [this.obstacle.width_x(o_num)/2;-this.obstacle.length_y(o_num)/2;this.obstacle.height_z(o_num)/2];
            %}
                % plot top and bottom
                plot3([vertex(1,1:4),vertex(1,1)],[vertex(2,1:4),vertex(2,1)], ...
                [vertex(3,1:4),vertex(3,1)],'k-','LineWidth',0.2);
                hold on;
                xlim([this.x_bound(1),this.x_bound(2)]);
                ylim([this.y_bound(1),this.y_bound(2)]);
                zlim([this.z_bound(1),this.z_bound(2)]);
                
                fill3(vertex(1,1:4),vertex(2,1:4),vertex(3,1:4),obs_col);
                
                plot3([vertex(1,5:8),vertex(1,5)],[vertex(2,5:8),vertex(2,5)], ...
                [vertex(3,5:8),vertex(3,5)],'k-','LineWidth',0.2);
                fill3(vertex(1,5:8),vertex(2,5:8),vertex(3,5:8),obs_col)
                % plot column
                for col=1:4
                    plot3([vertex(1,col),vertex(1,col+4)],[vertex(2,col),vertex(2,col+4)],...
                    [vertex(3,col),vertex(3,col+4)],'k-','LineWidth',0.25);
                end
                
                
                fill3([vertex(1,1),vertex(1,4),vertex(1,8),vertex(1,5)] ,...
                      [vertex(2,1),vertex(2,4),vertex(2,8),vertex(2,5)] ,...
                      [vertex(3,1),vertex(3,4),vertex(3,8),vertex(3,5)] ,...
                      obs_col);
                fill3([vertex(1,1),vertex(1,2),vertex(1,6),vertex(1,5)] ,...
                      [vertex(2,1),vertex(2,2),vertex(2,6),vertex(2,5)] ,...
                      [vertex(3,1),vertex(3,2),vertex(3,6),vertex(3,5)] ,...
                      obs_col);
                fill3([vertex(1,3),vertex(1,2),vertex(1,6),vertex(1,7)] ,...
                      [vertex(2,3),vertex(2,2),vertex(2,6),vertex(2,7)] ,...
                      [vertex(3,3),vertex(3,2),vertex(3,6),vertex(3,7)] ,...
                      obs_col);
                fill3([vertex(1,3),vertex(1,4),vertex(1,8),vertex(1,7)] ,...
                      [vertex(2,3),vertex(2,4),vertex(2,8),vertex(2,7)] ,...
                      [vertex(3,3),vertex(3,4),vertex(3,8),vertex(3,7)] ,...
                      obs_col);
                
          end
      % plot root/goal point
          root=this.node(:,1);
          root_p=plot3(root(1),root(2),root(3),'b*','LineWidth',3);
          go=this.goal;
          goal_p=plot3(go(1),go(2),go(3),'m*','LineWidth',3);
          

      % plot tree

          for i=2:this.last_id
          line_temp=[this.node(:,i),this.node(:,this.parent(i))];
          plot3(line_temp(1,:),line_temp(2,:),line_temp(3,:),'-', ...
          'Color', path_col,'LineWidth',0.3);
          hold on;
          end

      % plot the best path
          if this.reached==true
              cur_id=this.goal_idx;
              while(true)
                line_temp=[this.node(:,cur_id),this.node(:,this.parent(cur_id))];
                plot3(line_temp(1,:),line_temp(2,:),line_temp(3,:),'-+','LineWidth',1, ...
                    'Color',result_col);
                hold on;
                cur_id=this.parent(cur_id);
                if(cur_id==1)
                   break; 
                end
              end
          end
          
          
       % add legend£¬ title£¬ axis..
          maxi=num2str(max_ite);
          str=['RRT* atfer ', maxi, '-th iteration'];
          title(str);
          legend([root_p,goal_p],'root point','goal point'); 
          xlabel('x');
          ylabel('y');
          zlabel('z');


