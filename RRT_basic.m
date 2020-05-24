function []=RRT_basic()
% goal based bidirectional RRT algorithm without obstacles
% pretty loooooong and naaaiiive code
% may be simplified later
b=50;
g(1,:)=[5 5];
p1(1,:)=[5 5];
f(1,:)=[45 25];
p2(1,:)=[45 25];
for i=2:200
    k=b.*rand(1,2);
    for j=1:i-1
        d1(j)=dist_c(g(j,:),k);
    end
    s1=g(d1==min(d1),:);
    n1=((k-s1).*2)./dist_c(k,s1)+s1;
    g(i,:)=n1;
    p1(i,:)=s1;
    plot([s1(:,1) n1(:,1)],[s1(:,2) n1(:,2)],'k')
    xlim([0 50])
    ylim([0 50])
    hold on
    pause (0.1)
    for k=1:i-1
        d2(k)=dist_c(f(k,:),n1);
    end
    s2=f(d2==min(d2),:);
    if dist_c(n1,s2)<=2
        plot([s2(:,1) n1(:,1)],[s2(:,2) n1(:,2)],'k')
        hold on
        break
    end
    n2=((n1-s2).*2)./dist_c(n1,s2)+s2;
    f(i,:)=n2;
    p2(i,:)=s2;
    plot([s2(:,1) n2(:,1)],[s2(:,2) n2(:,2)],'k')
    hold on
    a=g;
    g=f;
    f=a;
    y=p1;
    p1=p2;
    p2=y;
end
    if rem(i,2)==1
        x=g;
        g=f;
        f=x;
        z=p1;
        p1=p2;
        p2=z;
        l1=1;
        u=[0 0];
        while (u(:,1)~=s2(:,1)) || (u(:,2)~=s2(:,2))
            u=g(l1,:);
            l1=l1+1;
        end
        l1=l1-1;
        while l1~=1
        plot([g(l1,1) p1(l1,1)],[g(l1,2) p1(l1,2)],'r','linewidth',2)
        hold on
        l2=1; u1=[0 0];
        while (u1(:,1)~=p1(l1,1)) || (u1(:,2)~=p1(l1,2))
            u1=g(l2,:);
            l2=l2+1;
        end
        l1=l2-1;
        end
        l3=i;
        while l3~=1
            plot([f(l3,1) p2(l3,1)],[f(l3,2) p2(l3,2)],'r','linewidth',2)
            hold on
            l2=1; u1=[0 0];
            while (u1(:,1)~=p2(l3,1)) || (u1(:,2)~=p2(l3,2))
                u1=f(l2,:);
                l2=l2+1;
            end
            l3=l2-1;
        end
          plot([s2(:,1) f(i,1)],[s2(:,2) f(i,2)],'r','linewidth',2)      
            
    end
        if rem(i,2)==0
            l1=1; u=[0 0];
            while (u(:,1)~=s2(:,1)) || (u(:,2)~=s2(:,2))
                u=f(l1,:);
                l1=l1+1;
            end
            l1=l1-1;
            while l1~=1
                plot([f(l1,1) p2(l1,1)],[f(l1,2) p2(l1,2)],'r','linewidth',2)
                hold on
                l2=1; u1=[0 0];
                while (u1(:,1)~=p2(l1,1)) || (u1(:,2)~=p2(l1,2))
                    u1=f(l2,:);
                    l2=l2+1;
                end
                l1=l2-1;
            end
            l3=i;
            while l3~=1
                plot([g(l3,1) p1(l3,1)],[g(l3,2) p1(l3,2)],'r','linewidth',2)
                hold on
                l2=1; u1=[0 0];
                while (u1(:,1)~=p1(l3,1)) || (u1(:,2)~=p1(l3,2))
                    u1=g(l2,:);
                    l2=l2+1;
                end
                l3=l2-1;
            end
            plot([s2(:,1) g(i,1)],[s2(:,2) g(i,2)],'r','linewidth',2)
        end
           
    
   

end
    
        
    
    