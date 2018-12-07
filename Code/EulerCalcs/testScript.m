clc;close all;clear
load testData.mat

figure(1)
DATALOG1_REFORMATTED = deg2rad(DATALOG1);

% Of the form: Heading (yaw), roll, pitch

%YAW: [Z] angle between the magnetic north direction and the y-axis, around the z-axis
%PITCH: [X] Pitch, rotation around x-axis (-180 to 180), with positive values when the z-axis moves toward the y-axis.
%ROLL: Roll,[Y] rotation around the y-axis (-90 to 90) increasing as the device moves clockwise.

%newCoords = zeros(size(DATALOG1_REFORMATTED));
% 
%newCoords(:,1) = DATALOG1_REFORMATTED(:,3); %yaw
%newCoords(:,2) = DATALOG1_REFORMATTED(:,2); %roll
%newCoords(:,3) = DATALOG1_REFORMATTED(:,1); %pitch

x_0 = [0,0,0];
x1 = [0,0,1];
x2 = [0,1,0];
x3 = [1,0,0];

yawMat = @(yaw) [cos(yaw),-sin(yaw),0;
          sin(yaw), cos(yaw),0;
          0,0,1];
      
rollMat = @(roll) [cos(roll),0,sin(roll);
           0,1,0;
          -sin(roll),0, cos(roll)];
      
pitchMat = @(pitch)[1,0,0;
            0,cos(pitch),-sin(pitch);
            0,sin(pitch),cos(pitch)];
        
%                                 Z          Y           X
rotMat = @(yaw,roll,pitch) yawMat(yaw)*rollMat(roll)*pitchMat(pitch);

transform = @(vals) rotMat(vals(1),vals(2),vals(3));

plot(DATALOG1_REFORMATTED(:,1)) % Yaw (Z)
hold on
plot(DATALOG1_REFORMATTED(:,2)) % Roll (Y)
plot(DATALOG1_REFORMATTED(:,3)) % Pitch (X)

legend(["In1: ZRot (Yaw)","In2: YRot (Roll)","In3: XRot(Pitch)"])


figure(2)
for i = [1:size(newCoords,1)]
    curVals = newCoords(i,:);
    rotMat = transform(curVals);
    %rotMat = eul2rotm(curVals);
    
    rotated1 = x1*rotMat;
    rotated2 = x2*rotMat;
    rotated3 = x3*rotMat;
    
    result1 = [x_0;rotated1];
    result2 = [x_0;rotated2];
    result3 = [x_0;rotated3];
    plot3(result1(:,1),result1(:,2),result1(:,3),'.-r')
    hold on
    plot3(result2(:,1),result2(:,2),result2(:,3),'.-g')
    plot3(result3(:,1),result3(:,2),result3(:,3),'.-b')
    xlim([-1,1])
    ylim([-1,1])
    zlim([-1,1])
    xlabel("X Axis")
    ylabel("Y Axis")
    zlabel("Z Axis")
    drawnow
    
    
end