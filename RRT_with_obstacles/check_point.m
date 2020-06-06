function val=check_point(pt,map)
% checks whether a point lies in the free space
val=true;
if ~(pt(1)>1 && pt(1)<size(map,2) && pt(2)>1 && pt(2)<size(map,1) && map(pt(2),pt(1))==1)
    val=false;
end
end

