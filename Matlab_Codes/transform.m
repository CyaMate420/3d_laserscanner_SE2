function [x,y]=transform(winkel, abstand)

al=size(winkel);

x=zeros(al);
y=zeros(al);
for i=1:al(1)
    x(i)=abstand(i)*cos(winkel(i));
    y(i)=abstand(i)*sin(winkel(i));
end

% figure
% plot(x,y)
% hold on 
% axis image
% xlabel('x-koordinate')
% ylabel('y-koordinate')
% 
% fitcurve=fit(x,y,'gaus2')