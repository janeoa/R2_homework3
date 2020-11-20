# Asset Malik's R2 HW3
This project requires Image Processing Toolbox to be installed because it uses imrotate function.
## How to launch
The project is done using MATLAB app builder and is stored as *app1.mlapp* but it is also exported as single *app1_exported.m* file.
* open the file
* run

## How to use
Both task1 and task2 are in the same app. After using task1 just flip the switch to use task2.

## Examples
### task 1
![task1](/gifs/task1.gif)
As you can see there are two sliders that control the linear velocity of the wheel. The software uses ICC in order to calculate new direction and position of the robot. The calculations are done every time timer event is fired. The timer event is triggered by timer and its period can be controlled by FPS slider. In order to conserve the velocity of the robot, FPS slider also inversily affects the speed. However collider force is fixed and can misbihave or extreme FPSs.
#### Direct Kinematics:
```MATLAB
speedLeft = speed(1);
speedRight = speed(2);

x = pos(1); y = pos(2);
o = dir;

R = 52*(speedRight+speedLeft)/(speedRight-speedLeft+10^-10);
w = (speedRight - speedLeft+10^-10)/104;

ICC = [x-R*sin(o) y+R*cos(o)];
A = [cos(w*dt) -sin(w*dt) 0; sin(w*dt) cos(w*dt) 0; 0 0 1];

new = A*[x-ICC(1); y-ICC(2); o] + [ICC(1) ICC(2) w*dt]';
```
### task 2
![task2](/gifs/task2.gif)
a small video (mp4) showing that the robot can perform the task from 3 different initial positions: https://youtu.be/e8_igD4qstM
#### Light sensors
The sensors calculate distance by
```MATLAB
RotSen = [cosd(theta) -sind(theta); sind(theta) cosd(theta)];
LL = RotSen*[-35;65]+app.robot_position_vector';
LL = (1-sqrt((LL(1)-app.light_position(1))^2+(LL(2)-app.light_position(2))^2)/sqrt(app.dimX^2+app.dimY^2))^2;
```
Here `[-35:65]` is position of left light sensor in relation to the robot's center. Which is rotated by matrix rotation. The distance from sensor to the light source is normalized by deviding the distance over scene diagonal. But this value is inversally proportional that is why the value is then inversed by `1-value`. The physics tells us that Light Intensity decays by `r^2`, that is why the value is squared.
#### contact sensor
Contacts sensors just check for collision of the robot with obstacles (but ignore the wall, because it was not required)

#### Control
The ANN is pretty simple and is a replica of ANN from presentation.
```MATLAB
ML = 0.6*dRL-0.15*CR+0.25;
MR = 0.6*dLL-0.15*CL+0.25;
```
Right light sensor feeds its data to the left motor and has a bias of 0.25. In case of collision speed drops a bit.
same goes to the second motor.
