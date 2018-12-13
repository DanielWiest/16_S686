x = [0,0,1];

grav = [0,0,-1.0];

rotMat = eul2rotm([0,-3*pi/4,-pi/4]);

plane = [1,0,0];

vec = x*rotMat;

figure(1)
theVec = [0,0,0;vec];
theGrav = [0,0,0;grav];
plot3(theVec(:,1),theVec(:,2),theVec(:,3),'.-b')
hold on
axis equal
plot3(theGrav(:,1),theGrav(:,2),theGrav(:,3),'.-r')
fill3([0,0,0,0],[5,-5,-5,5],[5,5,-5,-5],'g')
xlabel("X Axis")
ylabel("Y Axis")
zlabel("Z Axis")

figure(2)
projVecOntoPlane = (vec - plane*(dot(vec,plane)))
projGravOntoPlane = (grav - plane*(dot(grav,plane)))
theVec2 = [0,0,0;projVecOntoPlane];
theGrav2 = [0,0,0;projGravOntoPlane];
plot(theVec2(:,2),theVec2(:,3),'.-b')
hold on
plot(theGrav2(:,2),theGrav2(:,3),'.-r')
axis equal

magAB = norm(projVecOntoPlane)*norm(projGravOntoPlane)

thetaFromCos = acos( (dot(projVecOntoPlane,projGravOntoPlane))/(magAB));

sinTheta = (dot((cross(plane,projGravOntoPlane)),projVecOntoPlane)/magAB)

if sinTheta >= 0.0
   rad2deg(thetaFromCos - pi)
else
  rad2deg(pi-thetaFromCos)
end