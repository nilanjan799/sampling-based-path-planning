function []=RRT_star()
% author: nilanjan saha
% goal based bidirectional rrt* algorithm in presence of static obstacles

map=imresize(imread('map3.png'),.75);
map1=imbinarize(rgb2gray(map));
g(1,:)=[300,200]; p1(1,:)=g(1,:); c1(1)=0;   % source
f(1,:)=[430,330]; p2(1,:)=f(1,:); c2(1)=0;   % goal
nr1=[]; nr2=[]; nr3=[]; nr4=[];
s=15; r=35;            % stepsize and radius for nearest neighbours
imshow(map1)
axis on
hold on
set(gca,'Ydir','normal')
plot(g(1,1),g(1,2),'k.','markersize',30)
plot(f(1,1),f(1,2),'k.','markersize',30)
for i=2:2000  
    
    % adding node to the 1st tree
    for j1=1:100000
    k=rand(1,2).*[size(map1,2) size(map1,1)];   % random sample
    for j=1:i-1
        d1(j)=dist_c(g(j,:),k);
    end
    [~,idx]=min(d1);
     s1=g(idx,:);     % nearest node to the random sample
    n1=((k-s1).*s)./dist_c(k,s1)+s1;    % new node
    if ~(scan_path(s1,n1,map1))    % to ensure that the new node is in the free space
        continue;
    else
        break;
    end
    end
    g(i,:)=n1;
    p1(i,:)=s1;  % parent of the new node
    l=row_no(g,s1);
    c1(i)=c1(l)+dist_c(n1,s1);   % cost
    nr1=near2(g,n1,r,map1);
    i2=1; c3=[];
    
    % finding best parent
       if ~isempty(nr1)
        for i1=1:size(nr1,1)
            l1=row_no(g,nr1(i1,:));
            c3(i1)=c1(l1)+dist_c(n1,nr1(i1,:));      
            if c3(i1)<c1(i)
                nr2(i2,:)=nr1(i1,:);
                i2=i2+1;
            end   
        end
    end  
    [~,idx]=min(c3);
    if ~isempty(nr2)
        p1(i,:)=nr1(idx,:);  
        l=row_no(g,p1(i,:));
        c1(i)=c1(l)+dist_c(g(i,:),p1(i,:));
    end
    e1(i-1)=plot([g(i,1) p1(i,1)],[g(i,2) p1(i,2)],'m');
    pause(0.02)
    
    % rewiring
     if ~isempty(nr1)
        for i1=1:size(nr1,1)
            if nr1(i1,:)~=p1(i,:)
                l1=row_no(g,nr1(i1,:));
                if c1(l1)>c1(i)+dist_c(g(i,:),nr1(i1,:))
                    delete(e1(l1-1))
                    p1(l1,:)=g(i,:);     % changing parent
                    e1(l1-1)=plot([g(l1,1) p1(l1,1)],[g(l1,2) p1(l1,2)],'m');   %rewired edge
                    pause(0.02)
                end
            end
        end
     end
    
    % adding node to the 2nd tree towards the 1st tree
    for j=1:i-1
        d2(j)=dist_c(f(j,:),n1);
    end
    [~,idx]=min(d2);
    s2=f(idx,:);
    if dist_c(n1,s2)<s && scan_path(s2,n1,map1)
        plot([s2(:,1) n1(:,1)],[s2(:,2) n1(:,2)],'m')    % solution
        break;
    end
    n2=((n1-s2).*s)./dist_c(n1,s2)+s2;
    if ~(scan_path(s2,n2,map1))    % if node towards the other tree is not feasible
        f(i,:)=s2; p2(i,:)=s2;
        e2(i-1)=plot([f(i,1) p2(i,1)],[f(i,2) p2(i,2)],'m');
        pause(.02)
        l=row_no(f,s2);
        c2(i)=c2(l);
    else
        f(i,:)=n2; p2(i,:)=s2;
         l=row_no(f,s2);
         c2(i)=c2(l)+dist_c(n2,s2);
         nr3=near2(f,n2,r,map1);
         i2=1; c4=[];
         % finding best parent
           if ~isempty(nr3)
             for i1=1:size(nr3,1)
            l1=row_no(f,nr3(i1,:));
            c4(i1)=c2(l1)+dist_c(n2,nr3(i1,:));      
            if c4(i1)<c2(i)
                nr4(i2,:)=nr3(i1,:);
                i2=i2+1;
            end   
        end
    end  
      [~,idx]=min(c4);
      if ~isempty(nr4)
          p2(i,:)=nr3(idx,:);
          l=row_no(f,p2(i,:));
          c2(i)=c2(l)+dist_c(f(i,:),p2(i,:));
      end
      e2(i-1)=plot([f(i,1) p2(i,1)],[f(i,2) p2(i,2)],'m');
      pause(0.02)
     
    end
   % swapping the trees
    x1=g; g=f; f=x1;
    x2=p1; p1=p2; p2=x2;   
    x3=e1; e1=e2; e2=x3;
    x4=c1; c1=c2; c2=x4;    
end

    % finding and plotting the solution path
    if rem(i,2)==1
        x=g; g=f; f=x;
        z=p1; p1=p2; p2=z;
       l1=row_no(g,s2);
        while l1~=1
        plot([g(l1,1) p1(l1,1)],[g(l1,2) p1(l1,2)],'b','linewidth',2)
        hold on
        l1=row_no(g,p1(l1,:));
        end
        l3=i;
        while l3~=1
            plot([f(l3,1) p2(l3,1)],[f(l3,2) p2(l3,2)],'b','linewidth',2)
            hold on
          l3=row_no(f,p2(l3,:));
        end
          plot([s2(:,1) f(i,1)],[s2(:,2) f(i,2)],'b','linewidth',2)        
    end
      if rem(i,2)==0
           l1=row_no(f,s2);
            while l1~=1
                plot([f(l1,1) p2(l1,1)],[f(l1,2) p2(l1,2)],'b','linewidth',2)
                hold on
               l1=row_no(f,p2(l1,:));
            end
            l3=i;
            while l3~=1
                plot([g(l3,1) p1(l3,1)],[g(l3,2) p1(l3,2)],'b','linewidth',2)
                hold on
               l3=row_no(g,p1(l3,:));
            end
            plot([s2(:,1) g(i,1)],[s2(:,2) g(i,2)],'b','linewidth',2)
      end
    
end