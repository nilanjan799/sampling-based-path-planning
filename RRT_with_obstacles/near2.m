function n=near2(a,b,r,map)
% finds nearest nodes to new node in the free space
j=1;
l=size(a,1);
for i=1:l
   
    if dist_c(a(i,:),b)<=r && dist_c(a(i,:),b)>0 && scan_path(a(i,:),b,map)
        n(j,:)=a(i,:);
        j=j+1;
    end
  
end
end
        