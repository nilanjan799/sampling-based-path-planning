function val=scan_path(pt1,pt2,map)
% checks whether  a line is in the free space
val=true;
for d=0:0.5:dist_c(pt1,pt2)
    pt3=((pt2-pt1).*d)./dist_c(pt1,pt2)+pt1;
    if ~(check_point(ceil(pt3),map) && check_point(floor(pt3),map) && ...
           check_point([ceil(pt3(1)),floor(pt3(2))],map) && check_point([floor(pt3(1)),ceil(pt3(2))],map))
       val=false;
         break
    end
end
end
