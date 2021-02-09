clear; 
load('experiments.mat');

IO = struct('input', 'input (V)', 'output', 'load position (m)');      % define what the method has to consider as input and output signals
Gnp = nonpar_ident(experiment{:}, IO, 'time2frf');                     % calculate the nonparametric transfer function using the 'time2frf' method
bode(Gnp);      

experiment_1period = clip(experiment{1},'firstnper',1);           % clip the first period of experiment 1
Gnp_1period = nonpar_ident(experiment_1period, IO, 'time2frf');   % calculate the new measured frequency response, based on a single period
bode(Gnp,Gnp_1period);                                            % draw a Bode diagram and compare with using all data
legend('averaged','1 measurement, 1 period');                     % add a legendexperiment1_3periods = clip(experiment{1},'firstnper',3);   % clip the first three periods of experiment 1

experiment1_3periods = clip(experiment{1},'firstnper',3);                                  % clip the first three periods of experiment 4
experiment4_3periods = clip(experiment{3},'firstnper',3);                                  % clip the first three periods of experiment 4
Gnp_3periods = nonpar_ident(experiment1_3periods, experiment4_3periods, IO, 'time2frf');   % calculate the new measured frequency response, based the three periods of experiment 1 and 4
bode(Gnp,Gnp_3periods);                                                                    % draw a Bode diagram and compare with using all data
legend('averaged','2 measurements, 3 periods');                                            % add a legend

s = struct('numh', 1, 'numl', 0, 'denh', 3, 'denl', 1);                    % define the model structure 
Gp = param_ident('data', Gnp, 'method', 'nllsfdi', 'settings', s);         % fit a model with the structure we defined
opts = bodeoptions();                                                      % enhances the phase plot of the Bode diagram
opts.PhaseMatching = 'on';                                                 % enhances the phase plot of the Bode diagram
bodeplot(Gnp,Gp,opts);                                                     % make a Bode diagram 
legend('nonparametric FRF', 'fitted model');                               % add a legend

s_otherstructure = struct('numh', 1, 'numl', 0, 'denh', 5, 'denl', 1);                                          % let's for example change the highest denominator model to 4
Gp_otherstructure = param_ident('data', Gnp, 'method', 'nllsfdi', 'settings', s_otherstructure);                % fit a model with the structure we defined
bodeplot(Gnp,Gp,Gp_otherstructure,opts);                                                                        % make a Bode diagram 
legend('nonparametric FRF', 'fitted model with theoretical structure', 'fitted model with adapted structure');  % add a legend

% define our two main systems
G = IOSystem(1,2); G.add([Gp; Gp_angle]);
K = IOSystem(1,1);

% define some helper signals (aliases)
y = G.out(1); 
a = G.out(2);
u = G.in();
r = Signal(1);
e = r - y;

% define the connections
connections = [K.in == e ; K.out == u];

% define the closed-loop configuration
CL = IOSystem(G,K,connections);

% define the performance transfers
S = Channel(e/r, 'Sensitivity');
T = Channel(y/r, 'Complementary sensitivity (reference to load position)');
U = Channel(u/r, 'Control sensitivity'); 
A = Channel(a/r, 'Complementary sensitivity (reference to angle)');

% define the specifications
sensitivity_peak_db = 8;                 % we want |S| < 8 dB for all frequencies
MS = Weight.DC(sensitivity_peak_db);
WS = Weight.LF(0.1, 2, -40);             % pushes S down at low frequencies with 2nd order roll-off -> optimizes the bandwidth
WU = Weight.HF(3, 2, -40);               % pushes U down at high frequencies with 2nd order roll-off -> makes the controller roll off to suppress noise

objective = WS*S;                        % define the objective
constraint = [MS*S <= 1;                 % define the constraints
              WU*U <= 1];
          
[~,~,info] = CL.solve(objective, constraint, K);        % solve the problem
showall(info);   

opts = stepDataOptions('StepAmplitude', 0.4);
figure; step(CL(T),opts); title('Step response: absolute load position'); xlabel('time'); ylabel('load position (m)');     % step response of the absolute load position
figure; step(CL(A),opts); title('Step response: angle'); xlabel('time'); ylabel('angle (°)');                              % response of the angle to a step reference on the absolute load position

sensitivity_peak_db = 4;                 % lower this value to enforce more damping or the swinging motion
MS = Weight.DC(sensitivity_peak_db);     

objective = WS*S;                        % the design remains the same, but with other values for MS
constraint = [MS*S <= 1;
              WU*U <= 1];
[~,~,info] = CL.solve(objective, constraint, K); 

figure; step(CL(T),opts); title('Step response: absolute load position'); xlabel('time'); ylabel('load position (m)'); 
figure; step(CL(A),opts); title('Step response: angle'); xlabel('time'); ylabel('angle (°)'); 
