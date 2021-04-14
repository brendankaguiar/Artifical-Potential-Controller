% CPE470 Project 2: Artificial Potential Field Path Planning
%By Brendan Aguiar

%matlab notes
% x = y(i,:) assigns x the ith row of y
% x = y(:,i) assigns x the ith col of y
% Code by Dr. Jim La
clc, clear
close all
n = 2; % Number of dimensions
delta_t = .05; % Set time step
t = 0:delta_t:10;% Set total simulation time
lambda = 8.5; % Set scaling factor of attractive potential field
pr_max = 1.2; % Set maximum of robot velocity
error = zeros (length(t) - 1, 1); % Set tracking error

%=============Set VIRTUAL TARGET=======================================
qv = zeros (length(t), n); % Initial positions of virtual target
pv = 1.2; %Set velocity of virtual target
theta_v = zeros (length(t), 1); % Initial heading of the virtual target

%=============Set ROBOT================================================
%Set intial state of robot
qr = zeros (length(t), n); % Initial position of robot
pr = zeros (length(t), 1); % Initial velocity of robot. Also v_rd...
theta_r = zeros (length(t), 1); %Initial heading of the robot

%=============Set relative states between robot and TARGET=============
qrv = zeros (length(t), n); % Save relative positions 
%between robot and virtual target
prv = zeros (length(t), n); % Save relative velocities
% between robot and virtual target
%=============Compute initial relative states between robot and TARGET=
qrv(1,:) = qv(1,:) - qr(1,:); % Compute the initial relative position
%Compute the initial relative velocity
a = pv*cos(theta_v(1)) - pr(1)*cos(theta_r(1));
b = pv*sin(theta_v(1)) - pr(1)*sin(theta_r(1));
prv(1,:) = [a, b];

%=============Loop Assignments=========================================
qt_diff = zeros(length(t), n);
phi = zeros(length(t), 1);
nested = 0;
%=============Set noise mean and std. dev.=============================
noise_mean = .5;
noise_std = 0.5;% = 0.2;

%=============MAIN PROGRAM=============================================
for i = 2:length(t)
    % Code by Brendan Aguiar
    %+++++++++SINUSOIDAL TRAJECTORY+++++++++++++++++++++++++++++
    qv_x = t(i);%parametric equation 1
    qv_y = sin(t(i));%parametric equation 2
    
    %W/ noise
    %qv_x = t(i) + noise_std * randn + noise_mean;
    %qv_y = 4*sin(t(i) * 3) + 10 + noise_std * randn + noise_mean;
    %+++++++++LINEAR TRAJECTORY+++++++++++++++++++++++++++++++++
    %qv_x = t(i);%parametric equation 1
    %qv_y = t(i);%parametric equation 2
    
    %W/ noise
    %qv_x = t(i)+ noise_std * randn + noise_mean;
    %qv_y = 4*t(i) + 10 + noise_std * randn + noise_mean;
    % Code by Dr. Jim La
    %+++++++++CIRCULAR TRAJECTORY+++++++++++++++++++++++++++++++
    %W/O noise
    %qv_x = 60 - 15 * cos(t(i));%parametric equation 1
    %qv_y = 30 + 15 * sin(t(i));%parametric equation 2
    
    %W/ noise
    %qv_x = 60 - 15 * cos(t(i)) + noise_std * randn + noise_mean;
    %qv_y = 30 + sin(t(i))  + noise_std * randn + noise_mean;
    % Code by Brendan Aguiar
    %+++++++++SPIRAL TRAJECTORY++++++++++++++++++++++++++++++++++
    %qv_x = t(i) * cos(t(i));
    %qv_y = t(i) * sin(t(i));
    %+++++++++Lissajous TRAJECTORY+++++++++++++++++++++++++++++++
    %qv_x = 4* sin(3*t(i) + 5);
    %qv_y = 6*sin(t(i));
    %+++++++++Hypotrochoid TRAJECTORY++++++++++++++++++++++++++++
    %qv_x = 5 * cos(t(i)) + 5 * cos(5*t(i));
    %qv_y = 5 * sin(t(i)) - 5 * sin(5*t(i));
    %+++++++++Hypocycloid TRAJECTORY++++++++++++++++++++++++++++
    %qv_x = 5 * cos(t(i)) +  cos(5*t(i));
    %qv_y = 5 * sin(t(i)) -  sin(5*t(i));
    qv(i,:) = [qv_x, qv_y]; % Compute position of target
    qt_diff(i,:) = qv(i,:) - qv(i - 1,:);
    theta_v(i) = atan2(qt_diff(i,2),qt_diff(i,1));
    
    
    %+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % Code by Brendan Aguiar
    phi(i) = atan2(qrv(i - 1,2), qrv(i - 1,1));% Get phi
    %modeling robot velocity
    term1 = pv^2;
    term2 = 2*lambda*norm(qrv(i - 1,:))*pv*abs(cos(theta_v(i)-phi(i)));
    term3 = (norm(qrv(i - 1,:))*lambda)^2;
    pr(i) = sqrt(term1 + term2 + term3);
    if pr(i) >= pr_max
        pr(i) = pr_max;%robot velocity constraint
    end
    %modeling robot heading
    nested = (pv*sin(theta_v(i) - phi(i))/pr(i));
    theta_r(i) = phi(i) + asin(nested);
    
    % Code by Dr. Jim La here%
    c = cos(theta_r(i-1));
    d = sin(theta_r(i-1));
    qr(i,:) = qr(i - 1, :) + pr(i)*delta_t*[c, d];% Get next pos. of robot
    qrv(i,:) = qv(i,:) - qr(i,:);% Update pos. between robot and target
    e = pv*cos(theta_v(i) - pr(i)*cos(theta_r(i)));
    f = pv*sin(theta_v(i) - pr(i)*sin(theta_r(i)));
    prv(i,:) = [e,f];
    error(i) = norm(qv(i,:) - qr(i,:)); % Get dist. between robot & target
    % Plot positions qv of virtual target
    plot (qv(:,1),qv(:,2), 'r>')
    hold on
    % Plot positions qv of robot
    plot(qr(:,1),qr(:,2), 'g>')
   
    M = getframe(gca); % Find out what this does.
    %Getframe serves to capture the axes at each time in "t".
    %In other words, the chart is reloading with each iteration similar to
    %how a movie reel would cycle through frames (Mathworks, website).
    %mov = addframe(mov, M); % Find out what this does
    %Addframe will add current frame to the avi file (Northwestern, website).
end

%============PLOT RESULTS==============================================
figure(2), plot(error(2:length(t)), 'b.')
legend('Distance error between robot and virtual target')
figure(3), plot(pr, 'b')
figure(4), plot(theta_r, '--b')
hold on
plot(theta_v, '-.r')
hold on
plot (phi, 'k')
legend('Robot orientation', 'Target orientation', 'Relative Orientation')
%https://www.mathworks.com/help/matlab/ref/getframe.html
%http://www.ece.northwestern.edu/local-apps/matlabhelp/techdoc/ref/addframe.html