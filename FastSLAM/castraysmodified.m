% Code to perform Raycasting on a Supplied Map to return simulated Lidar
% Range values.
% % Author- Shikhar Shrestha, IIT Bhubaneswar

% Input parameters are x-position, y-position, Binary image of map, number
% of rays to cast, Lidar range in cm.
% % Returned Range values are obtained on scanning clockwise. 
function [angle,range]=castraysmodified(xc,yc,thetac,map,n,lidarMin,lidarrange,DEBUG)
MatrixSize = size(map);
angle = [];
%xc =  MatrixSize(2) - xc; 
yc = MatrixSize(1) - yc;
if(xc<=0) 
    xc = 1;
end
if(yc<=0) 
    yc = 1 ;
end

% % Conversion factor from pixels to centimeters.
pixeltom=0.1;
if map(floor(yc),floor(xc))==1
    range=zeros(n,1);
    angle = zeros(n,1);
    return;
end

if DEBUG == 1
figure(2);
hold off;
imshow(map);
hold on;
plot(xc,yc,'b*');
end

% if nargin==3
%     n=20;
%     lidarrange=500;
% end
% 
% if nargin==4
% lidarrange=500;
% end

range=zeros(n,1);
angle = zeros(n,1);

thetastep=270/(n-1);

for i=1:n
%     pause(0.5)
theta=-thetastep*(i-1)-thetac+135;
r=linspace(lidarMin,lidarrange,2000);
x=xc+(r*cosd(theta));
y=yc+(r*sind(theta));
% Removing points out of map
temp=[];
for k=1:numel(x)
    if x(k)>MatrixSize(2) || y(k)>MatrixSize(1) || x(k)<=0 || y(k)<=0
        temp=[temp;k];
    end
end
x(temp)=[];
y(temp)=[];

if DEBUG == 1 
figure(2);
plot(x,y,'r')
end

% Computing Intersections
xint=round(x);
yint=round(y);
% Correcting zero map indexing
for l=1:numel(xint)
    if xint(l)==0
        xint(l)=1;
    end
    if yint(l)==0
        yint(l)=1;
    end
end

b=[];
for j=1:numel(xint)
b=[b;map(yint(j),xint(j))];
end
ind=find(b==1);

if ~isempty(ind)
xb=x(ind(1));
yb=y(ind(1));
% else
% xb=max(x);
% yb=max(y);
    
if DEBUG == 1
figure(2);
plot(xb,yb,'g*')
end

dist=sqrt((xc-xb).^2 + (yc-yb).^2);
range(i)=dist;  
angle(i)=-theta/360*2*pi;
else
    angle(i)=-theta/360*2*pi;
    dist=sqrt((xc).^2 + (yc).^2);
    range(i)=dist;  
end
%pause(0.000001);
end
% Converting to m from pixels.
range=range*1*pixeltom;
%range=flipud(range);
angle=angle';
hold off