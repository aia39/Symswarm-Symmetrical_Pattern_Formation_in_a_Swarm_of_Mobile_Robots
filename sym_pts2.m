function a=sym_pts2(n,pc,pi)    
%%%%THIS FUNCTION CALCULATE N-1 SYMMETRICAL POINTS (eqn 18 of paper)
%parameter
%n = number of robot
%pc = master robot position
%pi = desired center
xc=pi(1);
yc=pi(2);
%%part a
bc=1;
dl=0;
x = zeros(1,n);
y = zeros(1,n);
s = zeros(1,n);
x(1)=pc(1);
y(1)=pc(2);
s(1)= (yc-y(1))/(xc-x(1));
r=sqrt(((yc-y(1))*(yc-y(1)))+(xc-x(1))*(xc-x(1)));
%%part b
for i=1:n
    
    s(i+1)= (s(i)-tan((360/n)*(3.1416/180)))/(1+s(i)*(tan((360/n)*(3.1416/180))))
end
k = zeros(1,n);
l = zeros(1,n);
p = zeros(1,n);
q = zeros(1,n);
ca=abs(2*r*sin((180/n)*(3.1416/180)));
for i=1:n
    k(i+1)=xc+r/(sqrt(1+s(i+1)^2));
    l(i+1)=xc-r/(sqrt(1+s(i+1)^2));
    p(i+1)=yc+s(i+1)*(k(i+1)-xc);
    q(i+1)=yc+s(i+1)*(l(i+1)-xc);
    dl=sqrt((k(i+1)-x(i))^2 + (p(i+1)-y(i))^2);
    if (dl>=ca-0.2)&&(dl<=ca+0.2)
        x(i+1)=k(i+1);
        y(i+1)=p(i+1);
       
    else
        x(i+1)=l(i+1);
        y(i+1)=q(i+1);    
    end
    
end
a=[x;y];
a;
if n==4
    a(1,4)=xc*2-a(1,2);
    a(2,4)=yc*2-a(2,2);
    a(1,3)=xc*2-a(1,1);
    a(2,3)=yc*2-a(2,1);
end

a(:,n+1) =[];


end
