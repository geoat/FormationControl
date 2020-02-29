pathPlanner = csvread('/home/geo/Dropbox/Thesis/C++/pathplanner_output_SL.Csv');
virtualDrone = csvread('/home/geo/Dropbox/Thesis/C++/virtualdrone_output_SL.Csv');

realx = 100:5:1100;
realy = 0:5:1000;

figure(1);
clf;
hold on
plot(pathPlanner(:,1),pathPlanner(:,2),'b');
plot(virtualDrone(:,1),virtualDrone(:,2),'g');
plot(realx,realy,'--r');
plot(100,0,'*k');
title('Path Planner Node for Straight Line Mission');
xlabel('x (meters)');
ylabel('y (meters)');

legend('Path Planner UAV', 'Reference Dyamics','Mission','Line Origin','Location','southeast');
print -depsc SLMission.eps


%% for orbit
pathPlanner = csvread('/home/geo/Dropbox/Thesis/C++/pathplanner_output_O.Csv');
virtualDrone = csvread('/home/geo/Dropbox/Thesis/C++/virtualdrone_output_O.Csv');


centre = [0,0];
radius = 50;

R = radius; x_c = centre(1); y_c = centre(2);thetas = 0:pi/512:pi*2;
xs = x_c + R*cos(thetas); ys = y_c + R*sin(thetas);

figure(2);
clf;
hold on
axis([-150 75 -75 450]);
%viscircles(centre, radius,'LineStyle','--','LineWidth',1.5);

plot(pathPlanner(:,1),pathPlanner(:,2),'b');
plot(virtualDrone(:,1),virtualDrone(:,2),'g');
plotCircle(centre,radius,0,pi*2);

plot(centre(1),centre(2),'*k');

title('Path Planner Node for Loitering Mission');
xlabel('x (meters)');
ylabel('y (meters)');

legend('Path Planner UAV','Reference Dyamics','Mission','Orbit Centre');
print -depsc OMission.eps

%% for waypoint
pathPlanner = csvread('/home/geo/Dropbox/Thesis/C++/pathplanner_output_W.Csv');
virtualDrone = csvread('/home/geo/Dropbox/Thesis/C++/virtualdrone_output_W.Csv');

realx = 100:5:1100;
realy = 0:5:1000;
waypoints=[0,0;500,500;500,0;0,500;0,0;500,500];
f = figure(3);
%f.WindowState = 'maximized';
clf;
hold on
axis([-150 510 -10 640]);
plot(pathPlanner(:,1),pathPlanner(:,2),'b');
plot(virtualDrone(:,1),virtualDrone(:,2),'g');
plot(waypoints(:,1),waypoints(:,2),'--r');
for i = 1:1:length(waypoints(:,1))
    plot(waypoints(i,1),waypoints(i,2),'*k');
end

title('Path Planner Node for Waypoints Mission');
xlabel('x (meters)');
ylabel('y (meters)');

legend('Path Planner UAV', 'Reference Dyamics','Mission','Waypoints','Location','northeast');
print -depsc WMission.eps



function plotCircle(centre,radius,startAngle, endAngle)
R = radius; x_c = centre(1); y_c = centre(2);thetas = startAngle:pi/512:pi*endAngle;
xs = x_c + R*cos(thetas); ys = y_c + R*sin(thetas);
plot(xs,ys,'--r');
end