% Start 
format short
clear all;
clc;

% Parameters Initialization
D=2;            % Dimensions
Lb=[-5 -5];     % Lower Bound
Ub=[5 5];       % Upper Bound
N=25;           % Population Size
alpha=1.0;      % Randomness strength 
beta0=1.0;      % Attraction Constant
gamma=0.01;     % Absorption coefficient
theta=0.97;     % Randomness reduction factor = 10^(-5/tMax)
iter_max=10000;   % Maximum no of iteration

% Generate Initial Population Randomly
for i=1:N
    for j=1:D
        pop(i,j)=Lb(:,j)+rand.*(Ub(:,j)-Lb(:,j));
    end
end

% Evaluate Objective Function
fx = fun(pop);

alpha = alpha*theta; % Reduce alpha by a factor theta
scale = abs(Ub-Lb);  % Scale of the problem

% Firefly Algorithm MAIN LOOP START
for iter = 1:iter_max
    for i=1:N
        for j=1:N
            fx(i)=fun(pop(i,:));
            if fx(i) < fx(j)
                pop(i,:)=pop(i,:);
            elseif fx(i) > fx(j)
                Xi = pop(i,:);
                Xj = pop(j,:);
                r = sqrt(sum(Xi-Xj).^2);
                beta = beta0*exp(-gamma*r.^2);
                steps=alpha.*(rand(1,D)-0.5).*scale;
                Xnew=Xi+beta*(Xj-Xi)+steps;
                % Check bounds
                for k=1:size(Xnew,2)
                    if Xnew(k)>Ub(k)
                        Xnew(k)=Ub(k);
                    elseif Xnew(k)<Lb(k)
                        Xnew(k)=Lb(k);
                    end
                end
                fnew=fun(Xnew);
                if fnew<fx(i)
                    fx(i)=fnew;
                    pop(i,:)=Xnew;
                end
            end
        end
    end
end


