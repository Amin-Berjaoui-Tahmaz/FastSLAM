function [newset] = Sample_motion_model(A,prevodom,newodom,oldset)

drot1 = atan2(newodom(2)-prevodom(2),newodom(2)-prevodom(2))-prevodom(3);
dtrans = sqrt((newodom(2)-prevodom(2))^2+(newodom(2)-prevodom(2))^2);
drot2 = newodom(3)-prevodom(3) - drot1;

newset=zeros(40,3);

for i=1:40
   drot1h = drot1-CalcSample(A(1)*drot1 + A(2)*dtrans);
   dtransh = dtrans-CalcSample(A(3)*dtrans +A(4)*(drot1+drot2));
   drot2h = drot2-CalcSample(A(1)*drot2 + A(2)*dtrans);
   
   xp = oldset(i,1)+dtransh*cos(oldset(i,3)+drot1h);
   yp = oldset(i,2)+dtransh*sin(oldset(i,3)+drot1h);
   thetap = oldset(i,3) + drot1h + drot2h;
   
   newset(i,:)=[xp,yp,thetap];
end

end

