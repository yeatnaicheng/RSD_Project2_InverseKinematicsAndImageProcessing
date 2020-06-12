clc;    % Clear the command window.
close all;  % Close all figures (except those of imtool.)
imtool close all;  % Close all imtool figures if you have the Image Processing Toolbox.
clear;  % Erase all existing variables. Or clearvars if you want.

%Read Image
rgbImage = imread('Image2.jpg');

%Enhance light using dehazing algorithm
enhancedImage = imreducehaze(rgbImage);

%Red Bars
%Create mask for RGB value of red bars
redMask_R = enhancedImage(:,:,1) > 100;
redMask_G = enhancedImage(:,:,2) < 100;
redMask_B = enhancedImage(:,:,3) < 70;
redBarMask = uint8(redMask_R & redMask_G & redMask_B);
redBar = zeros(size(redBarMask), 'uint8');
%Eliminate elements other than red bars
redBar(:,:,1) = enhancedImage(:,:,1) .* redBarMask;
redBar(:,:,2) = enhancedImage(:,:,2) .* redBarMask;
redBar(:,:,3) = enhancedImage(:,:,3) .* redBarMask;

%Yellow Bars
%Create mask for RGB value of yellow bars
yellowMask_R = enhancedImage(:,:,1) > 100;
yellowMask_G = enhancedImage(:,:,2) > 100;
yellowMask_B = enhancedImage(:,:,3) < 70;
yellowBarMask = uint8(yellowMask_R & yellowMask_G & yellowMask_B);
yellowBar = zeros(size(yellowBarMask), 'uint8');
%Eliminate elements other than yellow bars
yellowBar(:,:,1) = enhancedImage(:,:,1) .* yellowBarMask;
yellowBar(:,:,2) = enhancedImage(:,:,2) .* yellowBarMask;
yellowBar(:,:,3) = enhancedImage(:,:,3) .* yellowBarMask;

%Blue Bars
%Create mask for RGB value of blue bars
blueMask_R = enhancedImage(:,:,1) < 80;
blueMask_G = enhancedImage(:,:,2) < 80;
blueMask_B = enhancedImage(:,:,3) > 100;
blueBarMask = uint8(blueMask_R & blueMask_G & blueMask_B);
blueBar = zeros(size(blueBarMask), 'uint8');
%Eliminate elements other than blue bars
blueBar(:,:,1) = enhancedImage(:,:,1) .* blueBarMask;
blueBar(:,:,2) = enhancedImage(:,:,2) .* blueBarMask;
blueBar(:,:,3) = enhancedImage(:,:,3) .* blueBarMask;

%Red Bars Centroid & Orientation
%Gray scale red bar image
grayImage_R = rgb2gray(redBar);
%Binarize image
binaryImage_R = imbinarize(grayImage_R,0.1);
%Remove region smaller than 1000 pixels
binaryImage_R = bwareaopen(binaryImage_R,1000);
%Find connected components
cc_R = bwconncomp(binaryImage_R);
%Get area for regions
areaRed = regionprops(cc_R, 'Area');
%Remove region larger than 4000 pixels
lm_R = labelmatrix(cc_R);
newBinaryImage_R = ismember(lm_R, find([areaRed.Area] < 4000));
redProps = regionprops('table',newBinaryImage_R, 'Centroid','Orientation');
%Store red bars' centroids and oreintation into array
centroid_R = cat(1,redProps.Centroid);
orient_R = cat(1,redProps.Orientation);

%Yellow Bars Centroid & Orientation
%Gray scale yellow bar image
grayImage_Y = rgb2gray(yellowBar);
%Binarize image
binaryImage_Y = imbinarize(grayImage_Y,0.1);
%Remove region smaller than 1000 pixels
binaryImage_Y = bwareaopen(binaryImage_Y,1000);
%Find connected components
cc_Y = bwconncomp(binaryImage_Y);
%Get area for regions
areaYellow = regionprops(cc_Y, 'Area');
%Remove region larger than 4000 pixels
lm_Y = labelmatrix(cc_Y);
newBinaryImage_Y = ismember(lm_Y, find([areaYellow.Area] < 4000));
yellowProps = regionprops('table',newBinaryImage_Y, 'Centroid','Orientation');
%Store yellow bars' centroids and oreintation into array
centroid_Y = cat(1,yellowProps.Centroid);
orient_Y = cat(1,yellowProps.Orientation);

%Blue Bars Centroid & Orientation
%Gray scale blue bar image
grayImage_B = rgb2gray(blueBar);
%Binarize image
binaryImage_B = imbinarize(grayImage_B,0.1);
%Remove region smaller than 1000 pixels
binaryImage_B = bwareaopen(binaryImage_B,1000);
%Find connected components
cc_B = bwconncomp(binaryImage_B);
%Get area for regions
areaBlue = regionprops(cc_B, 'Area');
%Remove region larger than 4000 pixels
lm_B = labelmatrix(cc_B);
newBinaryImage_B = ismember(lm_B, find([areaBlue.Area] < 4000));
blueProps = regionprops('table',newBinaryImage_B, 'Centroid','Orientation');
%Store blue bars' centroids and oreintation into array
centroid_B = cat(1,blueProps.Centroid)
orient_B = cat(1,blueProps.Orientation)


%%%%%%%%%%%%%%%%%%%%%%




%Create Variable for keep track on the current joint variables
Currenttheta1=0
Currenttheta2=0
Currentd3=0
Currenttheta4=0
CurrentJoint=[Currenttheta1 Currenttheta2 Currentd3 Currenttheta4]

%Create Variable for keep track on the current coordinate variables
%The initialised value is when joint angles and distance are all 0
%Pre-calculated
CurrentX=650
CurrentY=0
CurrentZ=185.2
CurrentRoll=0
CurrentCoordinate=[CurrentX CurrentY CurrentZ CurrentRoll]

%Define Robot Links
L1=Link([0 257.7 400 0])
L2=Link([0 0 250 pi])
L3=Link([0 0 0 0 1])
L3.qlim=[0 180] % Set the limit of the robot to 180 movement in verticle axis
L4=Link([0 72.5 0 0])
robot=SerialLink([L1 L2 L3 L4])


%%Start Calling Functiono to Do the calculation and plotting
%Plot the initial robot position and get the coordinate from blue bars
for i=1:1:length(centroid_B)
   x= centroid_B(i,1)
   y= centroid_B(i,2)
   z=25
   rollDegree=orient_B(i)
   CurrentCoordinate=PlotInitial(robot)
   CurrentCoordinate=PlotGoToObject(CurrentJoint,robot,CurrentCoordinate,x,y,z,rollDegree)
   if(i==length(centroid_B))
   h=questdlg('This is the last link','Notification','OK','OK');    
   else
   h=questdlg('Press OK to continue','Notification','OK','OK');
   end
end

function IKSolution=calculateInverseKinematic(x,y,z,roll)
%Constant D-H Parameter
d1=257.7
d4=72.5
l1=400
l2=250

%Convert the received roll angle from degree to radian
roll=degtorad(roll)

%Inverse Kinematic Calculation
theta2=acos((x*x+y*y-l1*l1-l2*l2)/(2*l1*l2))
theta1=atan((y*(l1+l2*cos(theta2))-x*l2*sin(theta2))/(x*(l1+l2*cos(theta2))+y*l2*sin(theta2)))
d3=d1-d4-z
theta4=theta1+theta2-roll
IKSolution=[theta1 theta2 d3 theta4]
end

function CurrentCoordinate=PlotInitial(robot)
robot.plot([0,0,0,0])
CurrentCoordinate=ForwardKinematic(0,0,0,0)
end

function CoordinateResult=ForwardKinematic(theta1,theta2,d3,theta4)
d1=257.7
d4=72.5
l1=400
l2=250

Px=l2*cos(theta1+theta2)+l1*cos(theta1)
Py=l2*sin(theta1+theta2)+l1*sin(theta1)
Pz=d1-d3-d4
roll=radtodeg(asin(cos(theta4)*sin(theta1+theta2)-sin(theta4)*cos(theta1+theta2)))
CoordinateResult=[Px Py Pz roll]
end



function CurrentCoordinate=PlotGoToObject(CurrentJoint,robot,CurrentCoordinate,x,y,z,roll)

pausetime=0.1


%Save the current coordinate separately in each variable for visual purpose
CurrentX=CurrentCoordinate(1)
CurrentY=CurrentCoordinate(2)
CurrentZ=CurrentCoordinate(3)
CurrentRoll=CurrentCoordinate(4)

%Move in X Axis
if x>CurrentX
    Change=5
else
    Change=-5
end

for a=CurrentX:Change:x
    CurrentX=a;
    IKSolution=calculateInverseKinematic(CurrentX,CurrentY,CurrentZ,CurrentRoll)
    robot.plot([IKSolution(1) IKSolution(2) IKSolution(3) IKSolution(4)])
    pause(pausetime)
end
%Move in Y Axis
if y>CurrentY
    Change=5
else
    Change=-5
end
for a=CurrentY:Change:y
    CurrentY=a;
    IKSolution=calculateInverseKinematic(CurrentX,CurrentY,CurrentZ,CurrentRoll)
    robot.plot([IKSolution(1) IKSolution(2) IKSolution(3) IKSolution(4)])
    pause(pausetime)
end

%Move in Roll
if roll>CurrentRoll
    Change=1
else
    Change=-1
end
for a=CurrentRoll:Change:roll
    CurrentRoll=a;
    IKSolution=calculateInverseKinematic(CurrentX,CurrentY,CurrentZ,CurrentRoll)
    robot.plot([IKSolution(1) IKSolution(2) IKSolution(3) IKSolution(4)])
    pause(pausetime)
end

%Move in Z Axis
if z>CurrentZ
    Change=5
else
    Change=-5
end
for a=CurrentZ:Change:z
    CurrentZ=a;
    IKSolution=calculateInverseKinematic(CurrentX,CurrentY,CurrentZ,CurrentRoll)
    robot.plot([IKSolution(1) IKSolution(2) IKSolution(3) IKSolution(4)])
    pause(pausetime)
end
%Return the new current coordinate
CurrentCoordinate=[CurrentX CurrentY CurrentZ CurrentRoll]
end

%%%%%%%%%%%%%%%%%%%%%%
%% For Plotting ONLY
%Show the images
% figure,
% subplot('Position', [0 0.375 0.25 0.25]), imshow(enhancedImage);
% title('RGB Image');
% subplot('Position', [0.3 0.7 0.25 0.25]), imshow(redBar);
% title('Red Bars');
% subplot('Position', [0.6 0.7 0.25 0.25]), imshow(newBinaryImage_R);
% hold on;
% plot(centroid_R(:,1),centroid_R(:,2),'b*');
% hold off;
% title('Red Bars binary image with centroids')
% subplot('Position', [0.3 0.375 0.25 0.25]), imshow(yellowBar);
% title('Yellow Bars');
% subplot('Position', [0.6 0.375 0.25 0.25]), imshow(newBinaryImage_Y);
% hold on;
% plot(centroid_Y(:,1),centroid_Y(:,2),'b*');
% hold off;
% title('Yellow Bars binary image with centroids')
% subplot('Position', [0.3 0.05 0.25 0.25]), imshow(blueBar);
% title('Blue Bars');
% subplot('Position', [0.6 0.05 0.25 0.25]), imshow(newBinaryImage_B);
% hold on;
% plot(centroid_B(:,1),centroid_B(:,2),'b*');
% hold off;
% title('Blue Bars binary image with centroids')
