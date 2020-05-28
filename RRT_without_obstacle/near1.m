function n=near1(a,b,r)
% finds nearest nodes
j=1;
l=size(a,1);
for i=1:l
    if dist_c(a(i,:),b)<=r
        n(j,:)=a(i,:);
        j=j+1;
    end
end
end
        