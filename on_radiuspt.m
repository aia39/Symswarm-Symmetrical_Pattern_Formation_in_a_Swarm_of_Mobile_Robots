function posd = on_radiuspt(posg, posc, r)
%parameters
%posg => current given position of a bot
%posc => desired center point
%r => desired radius

%%this function is for eqn 22, 23 of paper for special case section

distance = @(a,b) sqrt((a(1)-b(1))^2 + (a(2)-b(2))^2);  %calculate distance (inline function)

d = distance(posg,posc);

xd = posg(1) + ((r-d)/d)*(posg(1) - posc(1));
yd = posg(2) + ((r-d)/d)*(posg(2) - posc(2));

posd = [xd yd];


end