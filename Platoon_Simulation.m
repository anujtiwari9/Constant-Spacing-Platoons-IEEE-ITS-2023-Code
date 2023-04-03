%% Code for 
%Lin, Y., Tiwari, A., Fabien, B. and Devasia, S., 2023. Constant-Spacing Connected Platoons With Robustness to Communication Delays. IEEE Transactions on Intelligent Transportation Systems, 24(3), pp.3370-3382.

%% Description: Run this script to simulate the vehicle network with DSR and non-DSR

%% Initialization
clc;clear all;close all;

% Simulation parameters
simStep = 0.01;
T = 50;
t = 0:simStep:T;

% Platoon informaiton 
N = 5;                                      %  Number of followers, N > 1
alpha = 0.4;                                %  Time constant (s^{-1}), Ts = 4/alpha;
d_0 = 6;                                    %  Steady target distance (m)
v_d = 20;                                   %  Target speed of the traffic (m/s)

% Communication parameters
commStatus = ones(1,N);                     %  1 x N boolean vector of communication status of all vehicles
tau_c = 2.5;                                %  The communication delay (secs)

% Local sensing delay 
tau_l = 0.1;

% lowpass freq 
w_f = 40*alpha; Kv = 10*alpha;

% blended DSR control parameter
beta = 1; tau_d = 0.1; gamma = 0.83;

% Parameter
para = [N,T,simStep,d_0,tau_l,tau_c,tau_d,alpha,beta,gamma,w_f,Kv,v_d];

% vehType: 1 x N vector, define the type of vehicles in a platoon, for each entry:   
%-      0:  Standard PLF autonomous vehicle
%-      1:  DSR based autonomous vehicle
vehType = ones(1,N);

%% Simulation to plot Fig.6
nFig = 0;
for row = [1,0]
    % Set the communication status
    commStatus = row*ones(1,N);
    
    for col = [1,0]
        % Select the vehicle type
        vehType = col*ones(1,N);
        
        % Run the simulation
        [X,V,A,S] = vehSysSim(para,commStatus,vehType);
        E = S - d_0;

        % plot the spacing error (Fig.1), velocity (Fig.2) and Acceleration (Fig.3)
        nFig = nFig + 1;
        for num = 1:N
            if num ~= N
                splot(1,nFig,t,E(:,num+1),"\delta_" + num2str(num) + "(t)","spacing error (m)",row,col,gamma);
            end  
            splot(2,nFig,t,V(:,num),"v_" + num2str(num) + "(t)","velocity (m/s)",row,col,gamma);
            splot(3,nFig,t,A(:,num),"a_" + num2str(num) + "(t)","acceleration (m/s^2)",row,col,gamma);
        end
        
        % Set the same y limit scale for comparing 
        if col == 0
            figure(1)
            lim = [min(E(:,2:end),[],'all'),max(E(:,2:end),[],'all')];
            ylim(lim)
            subplot(2,2,nFig - 1)
            ylim(lim)
            figure(2)
            lim = [min(V,[],'all'),max(V,[],'all')];
            ylim(lim)
            subplot(2,2,nFig - 1)
            ylim(lim)
            figure(3)
            lim = [min(A,[],'all'),max(A,[],'all')];
            ylim(lim)
            subplot(2,2,nFig - 1)
            ylim(lim)
        end
    end
end

%%
%%---- Utility functions ----%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% desireTraj: construct the desired velocity vector 
%-  Input:
%-      v_d: scalar, the target speed of the traffic
%-      t: 1xL vector, the time vector of the simulation
%-  Output: 
%-      X_d: 1xL vector, the desired position trajectory
function [V_d, X_d] = desireTraj(v_d,t)
    V_d = v_d*ones(1,length(t));
    X_d = v_d*t;                                    %  Integrate target speed to get desired distance
end

% vehSysSim: the simulator of the whole vehicle network
%-  Input: 
%-      para: 1D vector, includes
%-          -N: the number of vehicles 
%-          -T: the simulation end time
%-          -simStep: the simulation step
%-          -d_0: the target constant spacing 
%-          -tau_l: the local sensing delay 
%-          -tau_c: the buffer communication delay 
%-          -tau_d: the DSR delay for reinforcement 
%-          -alpha; the target settling time 
%-          -beta: the DSR gain 
%-          -gamma: the blending gain 
%-          -w_f: the low pass frequency 
%-          -Kv: the placement pole for stable p-z cancellation 
%-          -v_d: target velocity
%-      commStatus: 1xN vector, 0: not connected; 1: connected
%-      vehType: 1xN vector, 0: non DSR AV; 1: DSR AV 
%-  Output:
%-      X: L x N matrix, the position of the vehicle network with N vehicles and L time steps
%-      V: L x N matrix, the velocity of the vehicle network with N vehicles and L time steps
%-      A: L x N matrix, the acceleration of the vehicle network with N vehicles and L time steps
%-      S: L x N matrix, the relative spacing of the vehicle network with N vehicles and L time steps
function [X,V,A,S] = vehSysSim(para,commStatus,vehType)

%   Unpack parameters
N = para(1);T = para(2); simStep = para(3); d_0 = para(4); tau_l = para(5);
tau_c = para(6); tau_d = para(7); alpha = para(8); beta = para(9);gamma = para(10);
w_f = para(11); Kv = para(12); v_d = para(13);

%   Initialization
t = 0:simStep:T; 
L = length(t);
V = zeros(L,N);                             %  LxN matrix which stores the velocity for all agents for all time 
X = zeros(L,N);                             %  LxN matrix which stores the position for all time
U = zeros(L,N);                             %  LxN matrix which stores the control command for all time
A = zeros(L,N);                             %  LxN matrix which stores the acceleration for all time
D_0 = [0,d_0*ones(1,N-1)];                  %  1 x N vector of target spacing (m)
X0 = d_0*(0:-1:1-N);                        %  1xN vector , the initial position
S = zeros(L,N);                             %  LxN matrix, stores the relative spacing for all time
[V_d,X_d] = desireTraj(v_d,t);              %  Construct the desired trajectory with settling time 4/alpha
S0 = [X_d(1),X0(1,1:N-1)] - X0(1,:);        %  1xN vector, compute the initial spacing from the position

% Simulation
for i = 1:L                                 %  Time-axis iteration
    for j = 1:N                             %  Agent iteration
        
        % Update the initial conditions for the current step
        V_prev = initl(V,0,i,j);
        X_prev = initl(X,X0(j),i,j);
        U_prev = initl(U,0,i,j);
        
        % Add delay to communication info and local sensing
        if j == 1
            x_d = delayed(X_d, t, 0, tau_l, i,simStep);                     % local delay for comm info for the leader
            x_n_TauC = delayed(X(:,j), t, X0(j), tau_l, i, simStep);        % buffer comm delay for odemetry detection to match x_d
        else
            x_d = delayed(X_d, t, 0, tau_c, i,simStep);                     % comm delay info for the followers
            x_n_TauC = delayed(X(:,j), t, X0(j), tau_c, i, simStep);        % buffer comm delay for odemetry detection to match x_d
        end
        s_n = delayed(S(:,j), t, S0(j), tau_l, i,simStep);                  % local sensing delay for distance detection
        x_n = delayed(X(:,j), t, X0(j), tau_l, i,simStep);                  % local sensing delay for odometry detection
        
        % Compute the trajectory in discrete steps, depending on the types of the vehicle
        switch vehType(j) 
            % Standard PLF
            case 0
                U(i,j) = SPLF(j, x_d, x_n_TauC, s_n, commStatus(j), alpha, D_0(j));
                
            % DSR-based vehicles
            case 1
                % Achieve the delayed info
                s_n_TauD = delayed(S(:,j), t, S0(j), tau_d + tau_l, i,simStep);
                x_n_TauD = delayed(X(:,j), t, X0(j), tau_d + tau_l, i,simStep);
                
                % Update the speed 
                U(i,j) = DSR(j,s_n,s_n_TauD,x_n, x_n_TauD, x_n_TauC, alpha, beta, tau_d, D_0(j),commStatus(j),gamma,x_d);           
        end
        
        % Pass through vehicle dynamics to update the position
        U(i,j) = lowPass(U_prev,U(i,j),w_f,simStep);
        A(i,j) = Cff(U(i,j),U_prev,Kv,1,simStep) - Kv*V_prev;
        [X(i,j),V(i,j)] = vehDyn(A(i,j),X_prev,V_prev,simStep);
    end

    % Update the relative speed, position and relative spacing from speed
    S(i,:) = [X_d(i),X(i,1:N-1)] - X(i,:); 
end
end

% SPLF: This function updates the velocity of the AVs/CAVs with standard PLF control
%-  Input:
%-      j: 1x1 scalar. the order of the agent
%-      x_d: scalar. the target position
%-      x_n_TauC: scalar, the delayed odometry position
%-      s_n: scalar, the relative spacing detection
%-      connStat: scalar. The type of AV, 0: pure AV; 1:CAV
%-      alpha: scalar, the time constant for tracking
%-      d_0: scalar, the standatill target spacing.
%-  Output: 
%-      v:  the command of the next step
function v = SPLF(j, x_d, x_n_TauC, s_n, connStat,alpha,d_0)
    if connStat == 0 || j == 1  % without communication or leader
        v = alpha*(s_n - d_0);
	else                        % with communication
        v = alpha*(s_n - d_0) + alpha*(x_d  - (j-1)*d_0 - x_n_TauC);
    end
end

% DSR: This funciton updates the velocity of the AVs with DSR control
%-  Input:
%-      j: 1x1 scalar. The order of the agent
%-      s_n: scalar, the relative spacing detection
%-      s_n_TauD: scalar, the delayed relative spaicng detection for augmentation
%-      x_n: scalar, the delayed odometry position
%-      x_n_TauD: scalar, the delayed odometry position for augmentation
%-      x_n_TauC: scalar, the delayed odometry position for communication delay buffer
%-      alpha: scalar, the time constant for tracking
%-      beta: scalar, the DSR gain
%-      tau_d: the DSR delay
%-      d_0: scalar, the standstill target spacing.
%-      commStatus: scalar. The type of AV, 0: pure AV; 1:CAV
%-      gamma: scalar, the blending gain 
%-      x_d: scalar. the target position
%-  Output: 
%-      v:  the command of the next step
function v = DSR(j, s_n, s_n_TauD, x_n, x_n_TauD, x_n_TauC, alpha, beta, tau_d, d_0,commStatus,gamma,x_d)
    err = s_n - d_0; 
    err_prevTauD = s_n_TauD - d_0;
    if j == 1   % If the vehicle is a leader
        u_dsr = alpha * beta * err + (1 - beta)*(x_n - x_n_TauD)/tau_d;
    else
        u_dsr = alpha * beta * err + beta * (err - err_prevTauD)/tau_d + (x_n - x_n_TauD)/tau_d;
    end
    
    if commStatus == 1
        x_dn = x_d - (j-1)*d_0;
        v = gamma*u_dsr + alpha*(1-gamma)*(x_dn - x_n_TauC);
    else
        v = gamma*u_dsr;
    end
end

% delayed: Achieve the delayed information given certain delay
%-  Input:
%-      y: 1xL vector, the vector of the data to be delayed
%-      t: 1xL vector, the corresponding time vector of y
%-      y_init: scalar, the initial value of the data
%-      delayTime: scalar, time for applying delay
%-      currentStep: scalar, the current time stamp
%-      simStep: scalar, the simulation step
%-  Output: 
%-      y_del: scalar, the delayed value at current step
function y_del = delayed(y, t, y_init, delayTime, currentStep, simStep)
    if t(currentStep) < delayTime + simStep
        y_del = y_init;
    else
        ind = floor(currentStep - delayTime/simStep - 1);
        y_del = y(ind);
    end
end

%-  initl: to initialize the states for each current step
%-  Input:
%-      u: 2 x N matrix, the vector of state to be initialized
%-      y0: scalar, the initial value of the state
%-      i: scalar, the time stamp
%-      j: scalar, the platoon index of the state
%-  Output:
%-      y: the initialized state
function y = initl(u,y0,i,j)
    if i == 1
        y = y0;
    else
        y = u(i-1,j);
    end
end

% vehDyn: provide one step vehicle dynamics calculation using Euler method
%-  Input:
%-      u: scalar, the acceleration command
%-      x_p: scalar, the position at previous step
%-      v_p: scalar, the velocity at previous step
%-      simStep: scalar, the simulation step
%-  Output: 
%-      v: scalar, the updated velocity
%-      x: scalar, the updated position 
function [x,v] = vehDyn(u,x_p,v_p,simStep)
    v = v_p + u*simStep;
    x = x_p + v*simStep;
end

% lowPass: low pass filtering (the DSR command) with T(s) = w_f/(s+w_f)
%-  Input:
%-      y_prev: scalar, the previous step of the input signal
%-      u: scalar, the updated input signal
%-      w_f scalar, the filtering frequency
%-      simStep: scalar, the simulation step
%-  Output: 
%-      y: scalar, the filtered signal
function y = lowPass(y_prev,u,w_f,simStep)
    k = 1 - exp(-w_f*simStep);
    y = y_prev*(1 - k) + k*u;
end

% Cff: the feedback control Cff for stable pole zero cancellation
%-  Input: 
%-      u: scalar, the current step input 
%-      u_prev, scalar, the previous step input
%-      Kp: scalar, the proportional control gain
%-      Kd: scalar, the derivative control gain
%-      simStep: scalar, the simulation step
%-  Output: 
%-      y: the output signal for current step
function y = Cff(u,u_prev,Kp,Kd,simStep)
y = Kd*(u - u_prev)/simStep + Kp*u;
end

% splot: make a subplot for a given state of vehicle
%-  Input:
%-      i: scalar, the figure index
%-      nFig: scalar, the subplot index
%-      t: 1 x L vector, the time vector
%-      data: 1 x L vector, the data to be plot 
%-      lname: string, the name of the legend 
%-      ylabl: string, the label of the y axis
%-      row: scalar, the row index of the plot
%-      col: scalar, the column index of the plot
%-      gamma: scalar, the blending gain of DSR
function splot(i,nFig,t,data,lname,ylabl,row,col,gamma)
    figure(i)
    subplot(2,2,nFig)
    plot(t,data,'linewidth',2,'DisplayName',lname);
    hold on
    xlabel("("+char(96+nFig)+")"+"time (s)")
    ylabel(ylabl)
    addtitle(row,col,gamma)
    grid on 
    legend
    xlim([0,max(t)]);
end

% addtitle: add title onto the sub-plots 
% Input: 
%-      row: scalar, the row of the sub-plots
%-      col: scalar, the column of the sub-plots
%-      gamma: scalar, the blending gain of DSR method
function addtitle(row,col,gamma)
if row == 1
    titlestr = "PLF";
else
	titlestr = "PF";
end
        
if col == 1
	titlestr = titlestr + " with DSR" + " (\gamma = " + num2str(gamma) + ")";
else
	titlestr = titlestr + " without DSR";
end
title(titlestr)
end
