function pt=scan_path(pt1,pt2,map)
for d=0:dist_c(pt1,pt2)
    pt3=((pt2-pt1).*d)./dist_c(pt1,pt2)+pt1;
    if ~(check_point(ceil(pt3),map) && check_point(floor(pt3),map) && ...
           check_point([ceil(pt3(1)),floor(pt3(2))],map) && check_point([floor(pt3(1)),ceil(pt3(2))],map))
         break
    end
end
if d==dist_c(pt1,pt2)
    pt=pt2;
else
    d=d-1;
    pt=((pt2-pt1).*d)./dist_c(pt1,pt2)+pt1;
end
end
