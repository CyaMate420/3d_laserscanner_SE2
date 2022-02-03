function smooth(alpha, radius)

[zeile spalte] = size(radius)

for j=1:spalte

    p=0.9917895986747327;
    xxi=(0:2*pi/200:(2*pi-(2*pi/200)));
    g=csaps(alpha, radius(:,j), p, xxi);
    
    for i=1:200
        h(i)=g(i);
    end
    
    figure
    subplot(2,3,[1 2 3])
    plot(alpha,h,'r')
    hold on
    plot(xxi,radius(:,j),'g')
    
    subplot(2,3,4)
    polarplot(alpha,h)
    
    subplot(2,3,5)
    polarplot(alpha,radius(:,j))

    subplot(2,3,6)
    x == zeros(200,j)
    y == zeros(200,j)
    [x(:,j),y(:,j)]=transform(alpha,h)
    plot(x(:,j),y(:,j))
    axis image
end