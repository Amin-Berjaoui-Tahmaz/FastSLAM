function p = measurement_model_map(possiblepose,Z,lambda,lidRanges,lidarangles,lidMax,lidSTD,map)

map_bw = im2bw(map,0.9);
map_bw = 1-map_bw;

% figure
% imshow(A)
% figure
% imshow(map_bw)

xc=(possiblepose(1)+10)*10;
yc=(possiblepose(2)+10)*10;

dbstop if error

[angles,ranges]=castraysmodified(xc,yc,possiblepose(3)*180/pi,map_bw,25,20,200,0);

pRay = zeros(25,1);
for i=1:25
    
    [~,j]=min(abs(angles(i)-lidarangles));
    
    ztk=lidRanges(j);
    if ztk>lidMax
        ztk=lidMax;
    end
    
    ztks=ranges(i);
    
    n=1/(1-exp(-lambda*ztks));
    N=(1/sqrt(2*pi*lidSTD^2))*exp(-0.5*((ztk-ztks)^2)/(lidSTD^2));
    
    if ztk < lidMax && ztk>0
        phit=n*N;
        prand=1/lidMax;
    else
        phit=0;
        prand=0;
    end
    if ztk == lidMax
        pmax=1;
    else
        pmax=0;
    end
    if ztk< ztks && ztk>0
        pshort=n*lambda*exp(-lambda*ztk);
    else
        pshort=0;
    end
    pRay(i)=(Z(1)*phit+Z(2)*pshort+Z(3)*pmax+Z(4)*prand)*100;
end

p=prod(pRay);
end

