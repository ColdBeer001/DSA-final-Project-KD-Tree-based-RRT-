function TempNode =  Generatenode (this, data, axis)
if data==0
    TempNode.index=0;
else
num = length(data);

for i = 1:num-1
    for j = 1:num-i
        if this.node(axis,data(j+1)) < this.node(axis,data(j))
            data([j,j+1]) = data([j+1,j]);
        end
    end
end

data_sorted = data;
mid=ceil(num/2);
if mid>1
   leftSet = data_sorted(1:ceil(num/2)-1);
else
   leftSet=0;
end
if mid<num
   rightSet = data_sorted(ceil(num/2)+1:end);
else
   rightSet= 0;
end

TempNode.index=data_sorted(mid);
TempNode.lchild=Generatenode (this, leftSet, mod(axis,3)+1);
TempNode.rchild=Generatenode (this, rightSet, mod(axis,3)+1);
TempNode.axis=axis;
TempNode.size=num;
TempNode.boundary=this.node(axis,data_sorted(mid));
end