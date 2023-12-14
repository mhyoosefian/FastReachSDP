% This is a general code for fast Reach-SDP.
% Author: Mohammad Hussein Yoosefian Nooshabadi,
% yoosefiannooshabad.m@northeastern.edu
% Last edited: November 5, 2023.

% HOW TO USE THE CODE: Either choose from two predefined systems (quadrotor
% and double integrator), or define your own system at the end of "INPUTS"
% section.

clc
close all
clear all %#ok<*CLALL> 

%% CHOOSE SYSTEM
doubleInt = 1;      %gets 1 or 0. set to 1 to regenerate the result for double integrator system
quadrotor = 0;      %gets 1 or 0. set to 1 to regenerate the result for quadrotor system

% NOTE: To define your own system, set all previous variables to zero, and see line 45.

%% INPUTS
if doubleInt == 0 && quadrotor == 1
    load quadRotorv2.0.mat
    A = AMatrix;
    B = BMatrix;
    x0 = [4.70; 4.70; 3.00; 0.95; 0; 0];
    epsInit = [0.01; 0.01; 0.05; 0.05; 0.025; 0.025; 0.0001; 0.0001; 0.0001; 0.0001; 0.0001; 0.0001];
    numNeurons = [32 32];
    weights = net.weights;
    biases = net.biases;
    N = 10;
    solver = 'mosek';
    verbose = 0;
elseif doubleInt == 1 && quadrotor == 0
    load nnmpc_nets_di_1.mat
    A = [1 1; 0 1];
    B = [0.5; 1];
    x0 = [1; 1];
    N = 5;
    epsInit = 0.25*ones(4, 1);
    numNeurons = [15 10];
    solver = 'mosek';
    verbose = 0;
end

%%%%%%%%%% To define your own system, uncomment and use the following %%%%%%%%%%

% numNeurons = 0;         %the number of neurons for each hidden layer: [n1 n2 n3 ...]
% weights = 0;            %a cell of size 1 by length(numNeurons)+1
% biases = 0;             %a cell of size 1 by length(numNeurons)+1
% N = 0;                  %simulation time
% A = 0;                  %system's A matrix
% B = 0;                  %system's B matrix
% x0 = 0;                 %initial condition
% epsInit = 0;            %size of the hypercube for the initial condition (a single column)
% solver = 'mosek';       %highly recommended to use 'mosek'
% verbose = 0;

%% GENERAL SETTINGS
nx = size(A, 1);
nu = size(B, 2);
numHiddenLayers = length(numNeurons);
totalNumNeurons = sum(numNeurons);
Hin = zeros(2*nx, nx);      %Hin*x(t) <= hin
for i=1:nx
    Hin(2*i-1, i) = 1;
    Hin(2*i, i) = -1;
end
Hout = Hin;     %Hout*x(t+1) <= hout

Ein = [eye(nx) zeros(nx, totalNumNeurons+1)
       zeros(1, nx + totalNumNeurons) 1];

Enn = cell(numHiddenLayers, 1);
Enn{1} = zeros(nx+numNeurons(1)+1 , nx+totalNumNeurons+1);
Enn{1}(1:nx, 1:nx) = eye(nx);
Enn{1}(nx+1 : numNeurons(1)+nx , nx+1 : numNeurons(1)+nx) = eye(numNeurons(1));
Enn{1}(end) = 1;
ctr = 0;    %a counter for columns of Enn
for i=2:numHiddenLayers
    Enn{i} = zeros(numNeurons(i-1)+numNeurons(i)+1 , nx+totalNumNeurons+1);
    Enn{i}(1 : numNeurons(i-1) , ctr+nx+1 : ctr+numNeurons(i-1)+nx) = eye(numNeurons(i-1));
    ctr = ctr + numNeurons(i-1);
    Enn{i}(numNeurons(i-1)+1 : numNeurons(i-1)+numNeurons(i) , ctr+nx+1 : ctr+numNeurons(i)+nx) = eye(numNeurons(i));
    Enn{i}(end) = 1;
end

eps = zeros(2*nx, N+1);
eps(:, 1) = epsInit;
x = cell(N+1,1);
x{1} = x0;

%% MAIN CODE
for t=1:N
    disp(t);

    % Compute the control action and apply it to the system
    layerInputs = cell(1, numHiddenLayers+1);
    layerOutputs = cell(1, numHiddenLayers+1);
    layerInputs{1} = x{t};
    for j = 1:numHiddenLayers
        layerOutputs{j} = poslin(weights{j}*layerInputs{j} + biases{j});
        layerInputs{j+1} = layerOutputs{j};
    end
    layerOutputs{end} = weights{end}*layerInputs{end} + biases{end};
    u{t} = layerOutputs{end};
    x{t+1} =  A*x{t} + B*u{t};

    %%%%%%%%%% Step 1 %%%%%%%%%%

    % QC for the input (Min)
    hin = zeros(2*nx, 1);
    for i=1:2*nx
        hin(i) = Hin(i, :)*x{t} + eps(i, t);
    end
    Gamma = sdpvar(2*nx, 2*nx);
    constraints = [Gamma(:) >= 0, diag(Gamma) == 0];
    R = [Hin'*Gamma*Hin -Hin'*Gamma*hin
         -hin'*Gamma*Hin hin'*Gamma*hin];
    Min = Ein'*R*Ein;

    % QC for the hidden layers of neural network (Mnn)
    lambda = cell(numHiddenLayers, 1);
    gamma = cell(numHiddenLayers, 1);
    nu = cell(numHiddenLayers, 1);
    eta = cell(numHiddenLayers, 1);
    Mnn = cell(numHiddenLayers, 1);
    for k=1:numHiddenLayers
        n = size(weights{k}, 1);
        lambda{k} = triu(sdpvar(n-1, n), 1);
        gamma{k} = sdpvar(n, 1);
        nu{k} = sdpvar(n, 1);
        eta{k} = sdpvar(n, 1);
        constraints = [constraints, lambda{k}(:) >= 0, nu{k}(:) >= 0, eta{k}(:) >= 0]; %#ok<*AGROW> 
        Tnn = zeros(n, n);
        for ii=1:n
            ei = zeros(n, 1);
            ei(ii) = 1;
            Tnn = Tnn + gamma{k}(ii)*(ei*ei');
        end
        for ii=1:n-1
            ei = zeros(n, 1);
            ei(ii) = 1;
            for jj=ii+1:n
                ej = zeros(n, 1);
                ej(jj) = 1;
                Tnn = Tnn + lambda{k}(ii, jj)*((ei - ej)*(ei - ej)');
            end
        end
        Q11 = zeros(n, n);
        Q12 = Tnn;
        Q13 = -nu{k};
        Q22 = -2*Tnn;
        Q23 = nu{k} + eta{k}; 
        Q33 = 0;
        Q = [Q11 Q12 Q13; Q12.' Q22 Q23; Q13.' Q23.' Q33];
        W = weights{k};
        b = biases{k};
        tmp = [W zeros(size(W, 1), n) b
               zeros(n, size(W, 2)) eye(n) zeros(n, 1)
               zeros(1, size(W, 2) + n ) 1];
        Mnn{k} = Enn{k}'*tmp.'*Q*tmp*Enn{k};
    end
    
    % QC for output (an SDP for each face) (Mout)
    epsilon = sdpvar(2*nx, 1);
    for i=1:2*nx
        hout(i, 1) = Hout(i, :)*x{t+1} + epsilon(i);
    end
    Eout = [A zeros(nx, totalNumNeurons - numNeurons(end)) B*weights{end} B*biases{end};
    zeros(1, nx + totalNumNeurons) 1];
    constraintsEachFace = cell(2*nx, 1);
    P = cell(2*nx, 1);
    Mout = cell(2*nx, 1);
    for i=1:2*nx
        constraintsEachFace{i} = [constraints, epsilon(i) >= 0]; %if we don't add this constraint, the corner of hout will not be fixed at z(t+1)
        P{i} = [zeros(nx) Hout(i, :)'
                 Hout(i, :) -2*hout(i)];
        Mout{i} = Eout'*P{i}*Eout;

        % Solve the SDP
        ops = sdpsettings('solver', solver, 'verbose', verbose, 'debug', 1);
        sumMnns = zeros(size(Mout{i}));
        for k=1:numHiddenLayers
            sumMnns = sumMnns + Mnn{k};
        end
        constraintsEachFace{i} = [constraintsEachFace{i}, Mout{i} + sumMnns + Min <= 0];
        objective = epsilon(i);
        out = optimize(constraintsEachFace{i}, objective, ops);
        disp(out.info);
        eps(i, t+1) = value(epsilon(i));
        if eps(i, t+1) <= 1e-10     %to avoid numerical problems!
            eps(i, t+1) = 0;
        end
    end
end

%% PLOTS
% We only polt the first two components of the state vector!
for t=1:length(x)
    posX(t, 1) = x{t}(1); %#ok<*SAGROW> 
    posY(t, 1) = x{t}(2);
end
if doubleInt == 1
    xInitTemp = linspace(posX(1) - eps(1, 1), posX(1) + eps(2, 1), 10);
    yInitTemp = linspace(posY(1) - eps(3, 1), posY(1) + eps(4, 1), 10);
    [X, Y] = meshgrid(xInitTemp, yInitTemp);
    
    for i=1:size(X, 2)
        for j=1:size(X, 1)
            points{1}(2*i-1:2*i, j) = [X(j, i); Y(j, i)];
            for t=1:N
                layerInputs = cell(1, numHiddenLayers+1);
                layerOutputs = cell(1, numHiddenLayers+1);
                layerInputs{1} = points{t}(2*i-1:2*i, j);
                for jj = 1:numHiddenLayers
                    layerOutputs{jj} = poslin(weights{jj}*layerInputs{jj} + biases{jj});
                    layerInputs{jj+1} = layerOutputs{jj};
                end
                layerOutputs{end} = weights{end}*layerInputs{end} + biases{end};
                u{t} = layerOutputs{end};
                points{t+1}(2*i-1:2*i, j) =  A*points{t}(2*i-1:2*i, j) + B*u{t};
            end
        end
    end       
    figure
    hold on
    ylabel('\boldmath${y}$', 'Fontsize', 15, 'Interpreter', 'latex');
    xlabel('\boldmath${x}$', 'Fontsize', 15, 'Interpreter', 'latex');
    set(gca,'FontSize',12, 'TickLabelInterpreter', 'latex');
    grid on;
    box on;
    axis equal;
    rectangle("EdgeColor", 'green', 'Position', [posX(1)-eps(2, 1), posY(1)-eps(4, 1), ...
            eps(1, 1)+eps(2, 1), eps(3, 1)+eps(4, 1)], 'LineWidth', 2);
    rectangle("EdgeColor", 'black', 'Position', [posX(2)-eps(2, 2), posY(2)-eps(4, 2), ...
            eps(1, 2)+eps(2, 2), eps(3, 2)+eps(4, 2)], 'LineWidth', 2);
    h1 = plot(NaN, NaN, 'red', 'linewidth', 2, 'DisplayName', 'Initial box');
    h2 = plot(NaN, NaN, 'black', 'linewidth', 2, 'DisplayName', 'Fast Reach-SDP');
    for j=1:size(points{1}, 1)/2
        scatter(points{1}(2*j-1, :), points{1}(2*j, :), 'magenta', 'filled');
        scatter(points{2}(2*j-1, :), points{2}(2*j, :), 'magenta', 'filled');
    end
    for i=3:N+1
        rectangle("EdgeColor", 'black', 'Position', [posX(i)-eps(2, i), posY(i)-eps(4, i), ...
            eps(1, i)+eps(2, i), eps(3, i)+eps(4, i)], 'LineWidth', 2);
        for j=1:size(points{1}, 1)/2
            scatter(points{i}(2*j-1, :), points{i}(2*j, :), 'magenta', 'filled');
        end
    end
    legend([h1 h2], 'Interpreter', 'latex', 'FontSize', 15);
else
    figure;
    plot(posX, posY, 'r-*', 'linewidth', 2);
    hold on
    legend('trajectory', 'Fontsize', 17,'Interpreter', 'latex');
    ylabel('\boldmath${p_y}$', 'Fontsize', 15, 'Interpreter', 'latex');
    xlabel('\boldmath${p_x}$', 'Fontsize', 15, 'Interpreter', 'latex');
    set(gca,'FontSize',12, 'TickLabelInterpreter', 'latex')
    grid on;
    axis equal;
    for i=1:N+1
        rectangle("EdgeColor", 'blue', 'Position', [posX(i)-eps(2, i), posY(i)-eps(4, i), eps(1, i)+eps(2, i), eps(3, i)+eps(4, i)], 'LineWidth', 2);
    end
end