%% Compressor Stages Design
clear
clc
%% Cycle Parameters (from Gasturb)

% LPC inlet, outlet
state2 = initState(314.3, 80.68*1e3, 29.42); % T0 (K), p0 (Pa), mdot (kg/s)
state21 = initState(499, 306*1e3, 24.22); % mdot dec. to bypass
% HPC
state25 = initState(499, 303*1e3, 24.22);
state3 = initState(800, 1419*1e3, 23.5); % mdot cooling bleed

%% Air properties
air.gamma = 1.4;
air.cp = 1010; % J/kg.K
air.R = 287; % J/kg.K

% T=298;
% T_LPC_avg = (state2.T + state21.T)/2
% T_HPC_avg = (state25.T + state3.T)/2

% % Polynomial fit for cp
% p = [1.9327e-10, -7.9999e-07, 1.1407e-03, -4.4890e-01, 1057.5];
% % Evaluate the polynomial
% Cp = polyval(p, 1000)
% Cp = polyval(p, 800)

%% Constraints
phiMin = 0.4; phiMax = 0.75; % Vx/Um, Flow Coefficient
psiMin = 0.25; psiMax = 0.5; % dh0/Um^2, Work Coefiicient

%% Fan/LPC
air.cp = 1014; % J/kg.K (at avg temp 400K)
gamma  = air.cp / (air.cp - air.R)

% Design nums
Mt_rel = 1.1; % Reltive tip Mach number. 1.1 (Check CUM). 1.3 (old).
HTR = 0.65; % 0.35
state2.M = 0.4; % pure axial flow

% Find fan area 
%mbar = 0.85*nonDimMdot(air,1); % assume 0.5*choked mdot
mbar = nonDimMdot(air, state2.M);
Afan = areaFromMbar(air, state2, mbar);
rt = areaCalc(Afan, HTR);
rh = rt*HTR;
rm = (rh+rt)/2;
% Relative Mach calc.
Mt = sqrt(Mt_rel^2 - state2.M^2);
state2.T = totalToStaticTemp(air, state2.T0, state2.M);
a2 = spdSound(air, state2.T);
LPC.Ut = a2*Mt;
omega = (a2*Mt)/rt/2/pi;
fprintf('___FAN___\nSpeed (rpm): %.2f\n', omega*60); 

% Assign params
Vx = a2*state2.M; % Vx assumed constant through stage
LPC.omega = omega; LPC.rm = rm;  LPC.Vx =Vx;

% Exit conditions
[LPC, state21] = exitDimensions(state21, LPC,air);

% Check FC
LPC.Um = LPC.omega*2*pi*rm;
LPC.phi =  Vx/LPC.Um;
fprintf('Flow coeff, (Vx/Um): %.2f\n', LPC.phi); 
if (LPC.phi>phiMax)||(LPC.phi<phiMin)
    fprintf('Bad flow coeff!\n'); 
end
fprintf('Exit blade height (mm): %.2f\n', LPC.hexit*1e3)

%disp('LPC params: '); disp(LPC)

%% FAN/LPC Stage design

LPC = sizeCompressorStages(state2,state21,LPC,air);
%% FAN Blade design
%clc
% Limits
CLim.ARmin = 0.75; CLim.ARmin = 2.5; % Aspect Ratio for fan can be 4.
CLim.DFmax = 0.5; % Diffusion Factor
CLim.deltaBeta = 45; % degrees Flow deflection.
CLim.Rmin = 0.5; CLim.Rmax = 0.8; % Reaction

% Function inputs
alphaIn = 10; % deg
alphaIGV = 0;
AR = 0.75;
PCR = 0.7;
% Stators
AR_stator=0.75; arArr = repelem(AR_stator,LPC.N);
PCR_stator=0.75; PCRArr =repelem(PCR_stator,LPC.N);

% Display
dispBlade=1;
dispTipHub=0;

if dispBlade
    disp('Next')
    disp('___Mean___')
    meanLPC = bladeDesignCompressor(LPC, CLim,alphaIn,alphaIGV, AR, PCR,PCR_stator,AR_stator,'mean');
    %meanLPCTab = table(meanLPC.R',meanLPC.DF',meanLPC.Nb',meanLPC.deltaBeta','VariableNames',{'Reaction (0.5-0.8)','Diffusion (max 0.5)','N blade','Delta Beta (max 45)'});
    %disp(meanLPCTab)
    meanLPCTab = table(meanLPC.alphaS(:,1),meanLPC.alphaS(:,2),meanLPC.alphaS(:,3),meanLPC.betaS(:,1),meanLPC.betaS(:,2),meanLPC.Nb',meanLPC.DF',meanLPC.R',meanLPC.deltaBeta','VariableNames',{'alpha1','alpha2','alpha3','beta1','beta2','Nb','DF','R','dBeta'});
    disp(meanLPCTab)
    disp('Stators')
    meanLPCStator = table(arArr',PCRArr',meanLPC.Nbstator', meanLPC.Rstator',meanLPC.DFstator','VariableNames',{'AR','PCR','Ns','R (0.5-0.8)','DF (0.5 max)'});
    disp(meanLPCStator)

    disp('Fan Tip Blades:')
    tipLPC = bladeDesignCompressor(LPC, CLim,alphaIn,alphaIGV, AR, PCR,PCR_stator,AR_stator,'tip');
    tipLPCTab = table(tipLPC.R',tipLPC.DF',tipLPC.Nb',tipLPC.deltaBeta','VariableNames',{'Reaction','Diffusion','N blade','Delta Beta'});
        
    disp('Fan Hub Blades:')
    hubLPC = bladeDesignCompressor(LPC, CLim,alphaIn,alphaIGV, AR, PCR,PCR_stator,AR_stator,'hub');
    hubLPCTab = table(hubLPC.R',hubLPC.DF',hubLPC.Nb',hubLPC.deltaBeta','VariableNames',{'Reaction','Diffusion','N blade','Delta Beta'});
    if dispTipHub
        disp(tipLPCTab)
        disp(hubLPCTab)
    end
    %meanLPCTab = table(meanLPC.alphaS(:,1),meanLPC.alphaS(:,2),meanLPC.betaS(:,1),meanLPC.betaS(:,2),'VariableNames',{'alpha1','alpha2','beta1','Beta2'});

end
%hubLPC.V_thetaS
%meanLPC.V_thetaS
%tipLPC.V_thetaS
%LPC.betaS
%% HPC
% Iterate to decide suitable Vx
air.cp = 1058; % J/kg.K (at avg temp 600K)
gamma  = air.cp / (air.cp - air.R);

% Inital assumptions
HPC.Vx = LPC.Vx;
HTR_HPC = 0.8; % 0.5
phi = 0.5; % Vx/Um

% Start iteration
converged = 0; n=0;
while converged==0
    state25.T = totalToStaticTempVx(air,state25.T0,HPC.Vx);
    a25 = spdSound(air, state25.T);
    state25.M = HPC.Vx/a25; % assume mostly axial flow
    
    mbarHPC = nonDimMdot(air,state25.M);
    AHPC = areaFromMbar(air,state25,mbarHPC);
    rtHPC = areaCalc(AHPC, HTR_HPC);
    rhHPC = rtHPC*HTR_HPC;
    rmHPC = (rtHPC+rhHPC)/2;
    
    % Find tip velocity
    Vt_rel_max = Mt_rel*a25;
    Ut = Vt_rel_max/sqrt(1 + (phi * (1+HTR_HPC)/2)^2 );
    wHPC = Ut/(2*pi*rtHPC);
    
    UmHPC = wHPC*2*pi*rmHPC;
    phiGuess = HPC.Vx/UmHPC;
    VxNew = phi*UmHPC; % Updated value
    fracErr = abs((VxNew - HPC.Vx)/HPC.Vx);
    if  fracErr*100< 0.1
        converged=1;
        HPC.Vx = VxNew; HPC.omega = wHPC; HPC.rm = rmHPC; HPC.Ut=Ut;
    else
        HPC.Vx = VxNew;
        n = n+1;
    end
end

% Exit conditions
[HPC, state3] = exitDimensions(state3, HPC,air);

fprintf('\n___HPC___\nSpool Speed (rpm): %.2f\nNum iterations: %d\n', HPC.omega*60, n); 
fprintf('Exit blade height (mm): %.2f\n', HPC.hexit*1e3)

%% HPC Stage Design

HPC.Um = UmHPC; HPC.rm = rmHPC;
% Repeat the FAN/LPC process
HPC = sizeCompressorStages(state25,state3,HPC,air);

%% HPC Blade design
% Function
%clc
alphaIn =15; % deg. alpha1
alphaIGV = 15; % deg. alpha1 (for first stage)
AR = 0.8;
PCR = 0.66;
% Stator params
AR_stator=0.75; arArr = repelem(AR_stator,HPC.N);
PCR_stator=0.8; PCRArr =repelem(PCR_stator,HPC.N);

%
dispBlade=1;
dispTipHub=1;

if dispBlade
    disp('___Mean___')
    meanHPC = bladeDesignCompressor(HPC, CLim,alphaIn,alphaIGV, AR, PCR,PCR_stator,AR_stator,'mean');
    meanHPCTab = table(meanHPC.R',meanHPC.DF',meanHPC.Nb',meanHPC.deltaBeta',meanHPC.PTC_new','VariableNames',{'Reaction (0.5-0.8)','Diffusion (max 0.5)','N blade','dBeta (max 45)','PTC'});
    disp(meanHPCTab)

    meanHPCTab = table(meanHPC.alphaS(:,1),meanHPC.alphaS(:,2),meanHPC.alphaS(:,3),meanHPC.betaS(:,1),meanHPC.betaS(:,2),meanHPC.Nb',meanHPC.DF',meanHPC.R',meanHPC.deltaBeta','VariableNames',{'alpha1','alpha2','alpha3','beta1','beta2','Nb','DF','R','dBeta'});
    % disp(meanHPCTab)
    disp('Stators')
    meanHPCStator = table(arArr',PCRArr',meanHPC.Nbstator', meanHPC.Rstator',meanHPC.DFstator','VariableNames',{'AR','PCR','Ns','R (0.5-0.8)','DF (0.5 max)'});
    disp(meanHPCStator)

    if dispTipHub
        disp('___Tip___')
        tipHPC = bladeDesignCompressor(HPC, CLim,alphaIn, alphaIGV,AR, PCR,PCR_stator,AR_stator,'tip');
        tipHPCTab = table(tipHPC.R',tipHPC.DF',tipHPC.Nb',tipHPC.deltaBeta',tipHPC.PTC_new','VariableNames',{'Reaction','Diffusion','N blade','Delta Beta','PTC'});
        tipHPCTab = table(tipHPC.alphaS(:,1),tipHPC.alphaS(:,2),tipHPC.betaS(:,1),tipHPC.betaS(:,2),'VariableNames',{'alpha1','alpha2','beta1','Beta2'});
        disp(tipHPCTab)
        
        disp('___Hub___')
        hubHPC = bladeDesignCompressor(HPC, CLim,alphaIn,alphaIGV, AR, PCR,PCR_stator,AR_stator,'hub');
        hubHPCTab = table(hubHPC.R',hubHPC.DF',hubHPC.Nb',hubHPC.deltaBeta',hubHPC.PTC_new','VariableNames',{'Reaction','Diffusion','N blade','Delta Beta','PTC (0.6 min)'});
        disp(hubHPCTab)
        hubHPCTab = table(hubHPC.alphaS(:,1),hubHPC.alphaS(:,2),hubHPC.betaS(:,1),hubHPC.betaS(:,2),'VariableNames',{'alpha1','alpha2','beta1','Beta2'});
    end
end

% Final row anal

finalRow = final(hubHPC,meanHPC, tipHPC)

%% Turbine design.
%% General gas constant
R = 287;                    

%% Engine cycle values from Gasturb
% HPT - station 41 to 43
mdot_HPT       = 23.129;            % Mass flow [kg/s]
T0_in_HPT      = 1990;           % HPT inlet total temp [K] (Fixed to 2000)
P0_in_HPT      = 1376025;           % HPT inlet total pressure [Pa]
delta_T0_HPT   = 1990 - 1876.30; % Drop in total temp across the stage (Fixed to 2000)
eta_p_HPT      = 0.8934;            % HPT Polytropic Efficiency

% LPT - station 45 to 49
mdot_LPT       = 24.34;             % Mass flow [kg/s]
T0_in_LPT      = 1829.12;           % LPT inlet total temp [K]
P0_in_LPT      = 699233;            % LPT inlet total pressure [Pa]
delta_T0_LPT   = 1829.12 - 1655.3;  % Drop in total temp across the stage
eta_p_LPT      = 0.9053;            % LPT Polytropic Efficiency

%% Dynamic Cp & Gamma Calculations
Cp_HPT_in  = 1160; 
Cp_HPT_out = 1120; 
Cp_HPT_avg = (Cp_HPT_in + Cp_HPT_out) / 2;
gamma_HPT  = Cp_HPT_avg / (Cp_HPT_avg - R);

Cp_LPT_in  = 1120; 
Cp_LPT_out = 1090; 
Cp_LPT_avg = (Cp_LPT_in + Cp_LPT_out) / 2;
gamma_LPT  = Cp_LPT_avg / (Cp_LPT_avg - R);

%% Other values
N_HPT           = HPC.omega*60; % Spool matching
N_LPT           = LPC.omega*60; % Spool matching
rm_HPT          = 0.17;         
rm_LPT          = 0.22;         
phi             = 0.9;          
psi_max         = 1.3;            
M_rel_tip_limit = 1.0;          
PR_max_stage    = 2.2;          % Added max pressure ratio limit
max_rm_change   = 0.05;         % Limit for mean radius change between HPT & LPT [m]

%% Turbine Geometry Calculations
fprintf('\n--- Turbine Output Values ---\n')
% HPT calculations
[T0_out_HPT, P0_out_HPT, Vels_HPT, HPT] = sizeTurbine('High Pressure Turbine', ...
    mdot_HPT, T0_in_HPT, P0_in_HPT, delta_T0_HPT, N_HPT, rm_HPT, ...
    phi, psi_max, gamma_HPT, Cp_HPT_avg, R, eta_p_HPT, M_rel_tip_limit, PR_max_stage);

% LPT calculations
[~, ~, Vels_LPT, LPT] = sizeTurbine('Low Pressure Turbine', ...
    mdot_LPT, T0_in_LPT, P0_in_LPT, delta_T0_LPT, N_LPT, rm_LPT, ...
    phi, psi_max, gamma_LPT, Cp_LPT_avg, R, eta_p_LPT, M_rel_tip_limit, PR_max_stage);

%% System Level Checks
fprintf('\n====================================\n');
fprintf('--- Turbine System Level Checks ---\n');
fprintf('====================================\n');
rm_diff = abs(rm_HPT - rm_LPT);
fprintf('Mean Radius Change (HPT to LPT): %.3f m\n', rm_diff);
if rm_diff > max_rm_change
    fprintf('*** WARNING: Mean radius change exceeds limit of %.2f m! Swan-neck duct losses will be high. ***\n\n', max_rm_change);
else
    fprintf('Mean radius change is acceptable.\n\n');
end

%% Save
name = 'CompTurbstages';
%save(name,'LPC','HPC','HPT','LPT')
%% Overall plot
wBurn = 4;
figure; hold on
plotBlades(LPC,'#4287f5',0,'LPC') % #8FD9FB
plotBlades(HPC,'blue',LPC.N+2,'HPC') %#69C930
wBurn = 4; hBurn = 0.04;
rectangle('Position', [LPC.N+HPC.N+4.5, HPC.rm - hBurn/2, wBurn-1, hBurn], 'EdgeColor', 'red', 'LineWidth', 2,'Curvature', [0.3, 0.3],'LineStyle','-');
hDummy = plot(nan, nan, 's', 'MarkerEdgeColor', 'r', 'LineWidth', 2);
hDummy.DisplayName = 'Combustor';
plotBlades(HPT,'#FF5F1F',LPC.N+HPC.N+4+wBurn,'HPT')
plotBlades(LPT,'#EFBF04',LPC.N+HPC.N+HPT.N+6+wBurn,'LPT')

xticklabels({})
box on;
ylabel('\it Radius (m)')
title('Stage Design')
yMax=0.4;
ylim([0,yMax])
yticks(0:0.1:yMax) 
%xlim([0,LPC.N+HPC.N+5])
legend('show',  'Location','best')

function plotBlades(stage,col,iStart,Name)
    for i=1:stage.N
        if i==1
            plot([i+iStart,i+iStart],[stage.rh(i),stage.rt(i)],"Color",col,"LineWidth",2,DisplayName=Name); %
        else
            plot([i+iStart,i+iStart],[stage.rh(i),stage.rt(i)],"Color",col,"LineWidth",2,"HandleVisibility", "off");
        end
    end
    iEnd = i+iStart+1;
    plot([iEnd,iEnd],[stage.rhexit,stage.rtexit],"Color",'k',"LineWidth",2,"HandleVisibility", "off")
    plot([iStart+0.5,iEnd+0.5],[stage.rm,stage.rm],'k--',"LineWidth",0.8,"HandleVisibility", "off")
    %legend(p,{Name});
end


%% Funcitons

%spdSound(air,298)
%totalToStaticTemp(air,200,0.8)
%mbar_M = nonDimMdot(air,0.55)
function state=initState(T0,p0,mdot)
    state.T0 = T0;
    state.p0 = p0;
    state.mdot=mdot;
end

function a = spdSound(gas,Tstatic)
    a = (gas.gamma*gas.R*Tstatic)^0.5;
end

function Tstatic = totalToStaticTemp(gas,Ttotal,M)
    Tstatic = Ttotal/(1 + ((gas.gamma-1)/2)*(M^2) );
end

function Tstatic = totalToStaticTempVx(gas,Ttotal,Vx)
    Tstatic = Ttotal - Vx^2/(2*gas.cp);
end

function mbar = nonDimMdot(gas,M)
    gamma = gas.gamma;
    mbar = (M*(gamma)/sqrt(gamma-1)) *((1 + (M^2)*(gamma-1)/2))^(- (gamma+1)/2/(gamma-1));
end

function A = areaFromMbar(gas, state,mbar)
    A = state.mdot*sqrt(gas.cp*state.T0)/mbar/state.p0;
end

function rt = areaCalc(A, HTR)
    rt = sqrt(A/(pi*(1 - HTR^2)));
end

% Calucaltion of final stage exit conditions (LPC and HPC)
function [stage,exitState] = exitDimensions(exitState, stage,air)
    
    exitState.T = totalToStaticTempVx(air,exitState.T0,stage.Vx);
    aExit = spdSound(air, exitState.T);
    exitState.M = stage.Vx/aExit;

    % Find flow area and blade dimensions. At LPC/HPC exit
    mbar = nonDimMdot(air,exitState.M);
    exitState.A = areaFromMbar(air,exitState,mbar);
    stage.hexit = exitState.A/2/pi/stage.rm;

    h_lim = 10*1e-3; % 10 mm
    if stage.hexit<h_lim
        fprintf('Exit blade height too low, h (mm): %.2f\n', stage.hexit*1e3)
    end
    stage.rtexit = stage.rm + stage.hexit/2;
    stage.rhexit = stage.rm - stage.hexit/2;

    % Mass flow check - no non-dimensionalisation
    exitState.pStatic = exitState.p0/(1 + ((air.gamma-1)/2)*exitState.M^2)^(air.gamma/(air.gamma-1));
    exitState.rho = exitState.pStatic/(air.R * exitState.T);
    exitState.Acheck = exitState.mdot/(exitState.rho*stage.Vx);
end

function t = final(hub,mean,tip)
    PTC = [hub.PTC_new(end), mean.PTC_new(end), tip.PTC_new(end)]';
    DF = [hub.DF(end), mean.DF(end), tip.DF(end)]';
    R = [hub.R(end), mean.R(end), tip.R(end)]';
    dB = [hub.deltaBeta(end), mean.deltaBeta(end), tip.deltaBeta(end)]';
    t = table(PTC,DF,R,dB,'VariableNames',{'PCR','DF','R','dBeta'});
end

