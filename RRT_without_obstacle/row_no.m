function l=row_no(a,b)
% computes the row index of a given row of a matrix
i=1;
t=[100 100];
while (t(:,1)~=b(:,1)) || (t(:,2)~=b(:,2))
    t=a(i,:);
    i=i+1;
end
l=i-1;
end