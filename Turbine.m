%% Script to calculate the geometry values for the turbine stage based
%% on the cycle values from Gasturb - assume equal work done per stage
clear; clc; close all;

%% Define general gas constant
R = 287;                    

%% Engine cycle values from Gasturb
% HPT - station 41 to 43
mdot_HPT       = 48.297;            % Mass flow [kg/s]
T0_in_HPT      = 2000.00;           % HPT inlet total temp [K] (Updated)
P0_in_HPT      = 1376025;           % HPT inlet total pressure [Pa]
delta_T0_HPT   = 2000.00 - 1881.30; % Drop in total temp across the stage
eta_p_HPT      = 0.8934;            % HPT Polytropic Efficiency

% LPT - station 45 to 49
mdot_LPT       = 50.826;            % Mass flow [kg/s]
T0_in_LPT      = 1833.91;           % LPT inlet total temp [K]
P0_in_LPT      = 699233;            % LPT inlet total pressure [Pa]
delta_T0_LPT   = 1833.91 - 1660.18; % Drop in total temp across the stage
eta_p_LPT      = 0.9053;            % LPT Polytropic Efficiency

%% Dynamic Cp & Gamma Calculations (Averaged across the turbine)
Cp_HPT_in  = 1160; % J/kgK (Approx for ~2000K)
Cp_HPT_out = 1120; % J/kgK (Approx for ~1800K)
Cp_HPT_avg = (Cp_HPT_in + Cp_HPT_out) / 2;
gamma_HPT  = Cp_HPT_avg / (Cp_HPT_avg - R); % Thermodynamic consistency fix

Cp_LPT_in  = 1120; % J/kgK (Approx for ~1800K)
Cp_LPT_out = 1090; % J/kgK (Approx for ~1600K)
Cp_LPT_avg = (Cp_LPT_in + Cp_LPT_out) / 2;
gamma_LPT  = Cp_LPT_avg / (Cp_LPT_avg - R); % Thermodynamic consistency fix

%% Other values - Limits and Constraints
N_HPT           = 35041.47;     % HPT Nominal Speed [RPM] (Gasturb)
N_LPT           = 13827.79;     % LPT Nominal Speed [RPM] (Gasturb)
rm_HPT          = 0.14;         % HPT Mean Radius [m] 
rm_LPT          = 0.21;         % LPT Mean Radius [m] 
phi             = 0.7;          % Flow Coefficient (Ca/U) 
psi_max         = 1.3;          % Max Stage Loading Coefficient limit  
M_rel_tip_limit = 1.0;          % Max allowed Relative Tip Mach Number 
PR_max_stage    = 2.5;          % Max allowed pressure ratio per stage
max_rm_change   = 0.05;         % Max allowed change in mean radius between turbines [m]

%% Geometry Calculations
fprintf('Output Values\n')
% HPT calculations
[T0_out_HPT, P0_out_HPT, Vels_HPT, HPT] = sizeTurbine('High Pressure Turbine', ...
    mdot_HPT, T0_in_HPT, P0_in_HPT, delta_T0_HPT, N_HPT, rm_HPT, ...
    phi, psi_max, gamma_HPT, Cp_HPT_avg, R, eta_p_HPT, M_rel_tip_limit, PR_max_stage);

% LPT calculations
[~, ~, Vels_LPT, LPT] = sizeTurbine('Low Pressure Turbine', ...
    mdot_LPT, T0_in_LPT, P0_in_LPT, delta_T0_LPT, N_LPT, rm_LPT, ...
    phi, psi_max, gamma_LPT, Cp_LPT_avg, R, eta_p_LPT, M_rel_tip_limit, PR_max_stage);

% --- System Checks ---
fprintf('\n====================================\n');
fprintf('--- System Level Checks ---\n');
fprintf('====================================\n');
rm_diff = abs(rm_HPT - rm_LPT);
fprintf('Mean Radius Change (HPT to LPT): %.3f m\n', rm_diff);
if rm_diff > max_rm_change
    fprintf('*** WARNING: Mean radius change exceeds limit of %.2f m! Swan-neck duct losses will be high. ***\n\n', max_rm_change);
else
    fprintf('Mean radius change is acceptable.\n\n');
end