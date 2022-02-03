% *****MAtrix teilen******

laenge_seriell=length(x);

changedIndexes = diff(z)~=0;
ebene_anzahl=find(changedIndexes == 1);
punkt_pro_ebene=ebene_anzahl(1)

ebene=laenge_seriell/punkt_pro_ebene;

for i=1:ebene;
for j=1:punkt_pro_ebene;
xx(j,i)=x((i-1)*punkt_pro_ebene + j);
yy(j,i)=y((i-1)*punkt_pro_ebene + j);
end
end

for i=1:ebene;
for j=1:punkt_pro_ebene;
h(j,i)=z((i-1)*punkt_pro_ebene + j);
end
end

% xxe und yye sind extended version von xx und yy
xxe=zeros(201,ebene);
yye=zeros(201,ebene);

% den letzen mit dem ersten gleich setzen
xxe(201,:)=xx(1,:); 
yye(201,:)=yy(1,:);

% Werte kopieren
xxe(1:punkt_pro_ebene,:)=xx(1:punkt_pro_ebene,:);
yye(1:punkt_pro_ebene,:)=yy(1:punkt_pro_ebene,:);

% Das gleiche mit der Höhe
he=zeros(201,ebene);
he(201,:)=h(1,:);
he(1:punkt_pro_ebene,:)=h(1:punkt_pro_ebene,:);

plot3(xxe(:,:),yye(:,:),he(:,:),'b')
axis image
grid on
hold on

clear A
clear B
A=zeros(201,ebene);
for i=1:ebene
    A(:,i)=xxe(:,i);
    B(:,i)=yye(:,i);
end

H=zeros(ebene,1);
for hh=1:ebene
    H(hh)=(hh-1)*2;
end

plot3(A,B,H,'b')