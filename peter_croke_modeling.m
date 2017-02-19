
%%%% MOdeling using Peter Corke toolbox %%%% 
function peter_croke_modeling()
close all
clear all

nk=100;
%% Robot Modeling 
rd=0.1;            %odometry RMS error in distance
rh=4*pi/180;      %odometry RMS error in heading
W=diag([rd,rh].^2); %Odometry error covariance matrix
veh=Vehicle(W);     % an object from vehicle class
veh.init([5 5 .1]);

%% Map Modeling
slength=10;
nlmarks=4;             %Number of the landmarks in the map
map=Map(nlmarks,slength);
figure
grid on
hold on
map.plot();
xlim([-slength slength]);
ylim([-slength slength]);
%% Showing the initial robot pose in the map
veh.plot();

%% Sensor Modeling
rr=0.1;             %RMS range sensor error
rb=5*pi/180;          % RMS bearing sensor error
V=diag([rr,rb].^2); % Sensor error Covariance Matrix
sensor=RangeBearingSensor(veh,map,V);

%% Measurment accusition
z=zeros(nlmarks,2);
for n=1:nlmarks
z(n,:)=sensor.h(veh.x()',n);
end

%% Driving the robot around 
veh.add_driver(RandomPath(map.dim));    %Driving the robot randomly within the map space
% veh.run()

% ekf=EKF(veh,W,diag([0.05, 0.005, 0.001].^2),sensor,V,map);

% ekf.run(1000);

Q=diag([0.1, 0.1, 5*pi/180]).^2;

L=diag([0.1 0.1]);

pf= ParticleFilter(veh,sensor,Q,L,1000);

pf.run(nk);

figure
plot(1:nk,pf.std(:,1),'r',1:nk,pf.std(:,2),'b',1:nk,pf.std(:,3),'k');
title('RMSE Estimation')
legend('RMSE in x','RMSE in y','RMSE in theta');
grid on;

end

