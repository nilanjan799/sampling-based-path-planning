function []=RRT_star_basic()
% author: nilanjan saha
% goal based bidirectional RRT* algorithm without obstacles
b=100;
s=1.5; nr1=[]; nr2=[];
g(1,:)=[5 5]; c1(1)=0;
p1(1,:)=[5 5];
f(1,:)=[49 29]; c2(1)=0; nr3=[]; nr4=[];
p2(1,:)=[49 29];

for i=2:200
    k=b.*rand(1,2);
    for j=1:i-1
        d1(j)=dist_c(g(j,:),k);
    end
    s1=g(d1==min(d1),:);
    if dist_c(k,s1)<s
        n1=k;
    end
    n1=((k-s1).*s)./dist_c(k,s1)+s1;
    g(i,:)=n1; p1(i,:)=s1;
    l=row_no(g,s1);
    c1(i)=c1(l)+dist_c(n1,s1);
    nr1=near1(g,n1,2.5);
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
    e1(i-1)=plot([g(i,1) p1(i,1)],[g(i,2) p1(i,2)],'k');
    xlim([0 50])
    ylim([0 50])
    hold on
    pause(0.1)
    
    % rewiring the tree
    if ~isempty(nr1)
        for i1=1:size(nr1,1)
            if nr1(i1,:)~=p1(i,:)
                l1=row_no(g,nr1(i1,:));
                if c1(l1)>c1(i)+dist_c(g(i,:),nr1(i1,:))
                    delete(e1(l1-1))
                    p1(l1,:)=g(i,:);
                    e1(l1-1)=plot([g(l1,1) p1(l1,1)],[g(l1,2) p1(l1,2)],'r');
                    pause(0.1)
                end
            end
        end
    end
    
    for j=1:i-1
        d2(j)=dist_c(f(j,:),g(i,:));
    end
    s2=f(d2==min(d2),:);
    if dist_c(s2,g(i,:))<=s
        plot([g(i,1) s2(:,1)],[g(i,2) s2(:,2)],'k');
        break
    end
    n2=((g(i,:)-s2).*s)./dist_c(g(i,:),s2)+s2;
    f(i,:)=n2; p2(i,:)=s2;
    l=row_no(f,s2);
    c2(i)=c2(l)+dist_c(n2,s2);
    nr3=near1(f,n2,2.5);
    i2=1; c4=[];
    
    %finding best parent for the other tree
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
      e2(i-1)=plot([f(i,1) p2(i,1)],[f(i,2) p2(i,2)],'k');
      pause(0.1)
    
     x1=g; g=f; f=x1;
     x2=p1; p1=p2; p2=x2;
     x3=e1; e1=e2; e2=x3;
     x4=c1; c1=c2; c2=x4;
     
end
    
     % plotting the desired path
     % case1: if the no of iterations is odd
     if rem(i,2)==1
        x=g;
        g=f;
        f=x;
        z=p1;
        p1=p2;
        p2=z;
       l1=row_no(g,s2);
        while l1~=1
        plot([g(l1,1) p1(l1,1)],[g(l1,2) p1(l1,2)],'r','linewidth',2)
        hold on
        l1=row_no(g,p1(l1,:));
        end
        l3=i;
        while l3~=1
            plot([f(l3,1) p2(l3,1)],[f(l3,2) p2(l3,2)],'r','linewidth',2)
            hold on
          l3=row_no(f,p2(l3,:));
        end
          plot([s2(:,1) f(i,1)],[s2(:,2) f(i,2)],'r','linewidth',2)        
     end
    % case 2: if the no of iterations is even  
    if rem(i,2)==0
           l1=row_no(f,s2);
            while l1~=1
                plot([f(l1,1) p2(l1,1)],[f(l1,2) p2(l1,2)],'r','linewidth',2)
                hold on
               l1=row_no(f,p2(l1,:));
            end
            l3=i;
            while l3~=1
                plot([g(l3,1) p1(l3,1)],[g(l3,2) p1(l3,2)],'r','linewidth',2)
                hold on
               l3=row_no(g,p1(l3,:));
            end
            plot([s2(:,1) g(i,1)],[s2(:,2) g(i,2)],'r','linewidth',2)
    end
        
end