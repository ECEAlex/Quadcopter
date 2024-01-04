clear all, close all, clc
%% Continuous time model
% State x = Inertia moments, rotor inertia, rotor axis to copter center
% distance.
Ixx = 0.0086;
Iyy = 0.0086;
Izz = 0.0172;
Jr = 3.7404e-5;
m = 0.65;
g = 9.81;
l = 0.225;
kF = 3.13e-5;
kM = 9e-7;

a1 = (Iyy - Izz)/Ixx;
a2 = Jr/Ixx;
a3 = (Izz - Ixx)/Iyy;
a4 = (Jr/Iyy);
a5 = (Ixx-Iyy)/Izz;
b1 = l/Ixx;
b2 = l/Iyy;
b3 = l/Izz;

% Simulation setup
t0 = 0;
tf = 20;
dt = 0.001;
T = t0:dt:tf;
x0 = [0; 0; 0; 0; 0; 0; 2; 0; 0; 0; 0; 0]; % Observe the initial values
xd = [0; 0; 0; 0; 0; 0; 3; 0; 0; 0; 0; 0]; % Desired values

% Set up state value vector
x = zeros(12,length(T));
x(:,1) = x0;

% Set up place holder state values
y = x0;
yd = xd;

% Set up control vectors
u = zeros(4,length(T)-1);       % Input control vector
u_pd = zeros(4,length(T)-1);    % PD control vector
u_p = zeros(2,length(T)-1);     % P control vector

% Set up PD error vectors
e_p = zeros(4,length(T)-1);
ep = zeros(4,length(T)-1);      % P error vector
ed = zeros(4,length(T)-1);      % D error vector

ohm = zeros(4,1);               % Set up ohm/motor vector
hover = m*g/4;                  % hover value

kp = [1.5 0.8 0.8 0.8];
kd = [1 0.2 0.2 0.2];

kp2 = [5 5];
kd2 = [10 10];


wp = [2 0; 2 2];

count = 1;


% Solve for control gain and simulate – online
for t=1:length(T)-1

    if(count < length(wp))
        yd(9) = wp(count,1);
        yd(11) = wp(count,2);
    end

    % Waypoint following
    e_p(1,t) = yd(9)-y(9);      % x position error
    e_p(2,t) = yd(10)-y(10);    % x velocity error
    e_p(3,t) = yd(11)-y(11);    % y position error
    e_p(4,t) = yd(12)-y(12);    % y velocity error
    
    u_p(1,t) = kp2(1)*e_p(1,t)+kd2(1)*e_p(2,t); % X-direction Control
    u_p(2,t) = kp2(2)*e_p(3,t)+kd2(2)*e_p(4,t); % Y-direction Control

    if(count >= length(wp))
        yd(9) = 1;
        yd(11) = 2;
    end

    if(abs(yd(5)-y(5)) > 0.01)
        u_p(1,t) = 0;
        u_p(2,t) = 0;
    end

    if(abs(yd(7)-y(7)) > 0.01)
        u_p(1,t) = 0;
        u_p(2,t) = 0;
    end

    if(e_p(1,t) >= 0.1)
        % Facing x direction
        yd(1) = deg2rad(u_p(2,t));
        yd(3) = deg2rad(u_p(1,t));
    elseif(e_p(1,t) < 0.1)
        if(e_p(3,t) >= 0.1)
            yd(5) = atan2(yd(11),0);
            count = count + 1;
        elseif(e_p(3,t) < 0.1)
            yd(5) = - atan2(yd(11),0);
            count = count + 1;
        end
    end
    if(e_p(3,t) >= 0.1)
        % Facing y direction
        yd(1) = deg2rad(u_p(1,t));
        yd(3) = deg2rad(u_p(2,t));
    elseif(e_p(3,t) < 0.1)
        if(e_p(1,t) >= 0.1)
            yd(5) = y(5) + atan2(0,yd(9));
        elseif(e_p(1,t) < 0.1)
            yd(5) = y(5)-atan2(0,yd(9));
        end
    end

    if(rad2deg(y(5)) > 360)
        y(5) = deg2rad(0);
    end

    % Saturation Functions
    v_max = deg2rad(5);
    if(abs(yd(1)) > v_max)
        if(yd(1) < 0)
            yd(1) = -1*v_max;
        elseif(yd(1) >= 0)
            yd(1) = 1*v_max;
        end
    end
    if(abs(yd(3)) > v_max)
        if(yd(3) < 0)
            yd(3) = -1*v_max;
        elseif(yd(3) >= 0)
            yd(3) = 1*v_max;
        end
    end

    % Proportional control error
    ep(1,t) = (yd(7) - y(7));   % Altitude P error
    ep(2,t) = (yd(1) - y(1));   % Roll P error
    ep(3,t) = (yd(3) - y(3));   % Pitch P error
    ep(4,t) = (yd(5) - y(5));   % Yaw P error

    % Derivative control error
    ed(1,t) = (yd(8) - y(8));   % Altitude D error
    ed(2,t) = (yd(2) - y(2));   % Roll D error
    ed(3,t) = (yd(4) - y(4));   % Pitch D error
    ed(4,t) = (yd(6) - y(6));   % Yaw D error

    % Control
    u_pd(1,t) = kp(1)*ep(1,t) + kd(1)*ed(1,t);  % Altitude PD control
    u_pd(2,t) = kp(2)*ep(2,t) + kd(2)*ed(2,t);  % Roll PD control
    u_pd(3,t) = kp(3)*ep(3,t) + kd(3)*ed(3,t);  % Pitch PD control
    u_pd(4,t) = kp(4)*ep(4,t) + kd(4)*ed(4,t);  % Yaw PD control


    % Rotor Speed Calculations
    ohm(1) = hover + u_pd(1,t) + u_pd(3,t) + u_pd(4,t);
    ohm(2) = hover + u_pd(1,t) - u_pd(2,t) - u_pd(4,t);
    ohm(3) = hover + u_pd(1,t) - u_pd(3,t) + u_pd(4,t);
    ohm(4) = hover + u_pd(1,t) + u_pd(2,t) - u_pd(4,t);
    ohmr = ohm(1) - ohm(2) + ohm(3) - ohm(4);


    % Input Controls 
    u(1,t) = -(ohm(1)+ohm(2)+ohm(3)+ohm(4));
    u(2,t) = (0-ohm(2)+0+ohm(4));
    u(3,t) = (ohm(1)+0-ohm(3)+0);
    u(4,t) = (ohm(1)-ohm(2)+ohm(3)-ohm(4)); 
    
    % Simulate quadcoptor
    dy(1,1) = y(2);
    dy(2,1) = y(4)*y(6)*a1+y(4)*ohmr*a2+b1*u(2,t);
    dy(3,1) = y(4);
    dy(4,1) = y(2)*y(6)*a3+y(2)*ohmr*a4+b2*u(3,t);
    dy(5,1) = y(6);
    dy(6,1) = y(2)*y(4)*a5+b3*u(4,t);
    dy(7,1) = y(8);
    dy(8,1) = -g-(u(1,t)/m)*cos(y(1))*cos(y(3));
    dy(9,1) = y(10);
    dy(10,1) = (-u(1,t)/m)*(sin(y(1))*sin(y(5)) + cos(y(1))*sin(y(3))*cos(y(5)));
    dy(11,1) = y(12);
    dy(12,1) = (u(1,t)/m)*(sin(y(1))*cos(y(5)) - cos(y(1))*sin(y(3))*sin(y(5)));
    
    for i=1:12
        y(i,1) = y(i,1) + dy(i,1)*dt;
    end
    x(:,t+1) = y(:, 1);

end

%Define total width, length and height of flight arena (metres)
spaceDim = 10;
spaceLimits = [-spaceDim/2 spaceDim/2 -spaceDim/2 spaceDim/2 0 spaceDim/2];


%figure to display drone simulation
f1 = figure;
ax1 = gca;
view(ax1, 3);
xlabel('X')
ylabel('Y')
zlabel('Z')
axis equal;
axis(spaceLimits)
grid ON
grid MINOR
caxis(ax1, [0 spaceDim]);
hold(ax1,'on')
axis vis3d

an = animatedline;

for k=1:100:(length(T)-1)
    addpoints(an,x(9,k),x(11,k),x(7,k))
    drawnow
end




