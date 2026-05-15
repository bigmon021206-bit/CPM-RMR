%% plotDOR_fixedCPM.m
% Compute and plot DOR only.
% DOR(d,dt) = NV2X(d,dt) / nveh(d,dt)
% NV2X counts duplicated object detections received via successful CPMs.

clear; clc; close all;

%% ===== Data load =====
dataFile = '/Users/bigmo/OneDrive/바탕 화면/WiLabV2Xsim-main/highway_1000m_time1250_veh200_2025-04-21.mat';
data = load(dataFile);
mydata = data.mydata;

targetVehList = 1:200;
d_list = 10:25:700;

tStart = 21;
tEnd   = 25;

%% ===== Experiment parameters =====
% Use scalar or vector. Both are supported.
distanceParams  = 25;
frequencyParams = 5;
dynamicPParams  = 1.9;
dynamicSParam   = 0.5;

includeNoRMR = true;

%% ===== Fixed CPM No RMR DOR =====
DOR_NoRMR = [];
if includeNoRMR && isfile('EAR_log_fixedCPM_none_th0.mat') && isfile('RMRPolicy_fixedCPM_none_th0.mat')
    noLog = load('EAR_log_fixedCPM_none_th0.mat');
    noPol = load('RMRPolicy_fixedCPM_none_th0.mat');

    DOR_NoRMR = computeDOR_from_RMRMsg( ...
        noLog.EAR_log, noPol.RMRPolicy, mydata, ...
        targetVehList, d_list, tStart, tEnd);
else
    includeNoRMR = false;
end

%% ===== Frequency RMR DOR =====
DOR_frequency = nan(length(frequencyParams), length(d_list));

for i = 1:length(frequencyParams)
    N = frequencyParams(i);

    logFile = sprintf('EAR_log_fixedCPM_frequency_th%d.mat', N);
    polFile = sprintf('RMRPolicy_fixedCPM_frequency_th%d.mat', N);

    if ~isfile(logFile) || ~isfile(polFile)
        warning('Missing file for frequency RMR: N=%d', N);
        continue;
    end

    logData = load(logFile);
    polData = load(polFile);

    DOR_frequency(i,:) = computeDOR_from_RMRMsg( ...
        logData.EAR_log, polData.RMRPolicy, mydata, ...
        targetVehList, d_list, tStart, tEnd);
end

figure;
hold on;
markers = {'-s','-^','-d','-x','-p','-*','-v'};
legendEntries = {};

if includeNoRMR
    plot(d_list, DOR_NoRMR, 'k-o', 'LineWidth', 2);
    legendEntries{end+1} = 'No RMR'; %#ok<SAGROW>
end

for i = 1:length(frequencyParams)
    if all(isnan(DOR_frequency(i,:)))
        continue;
    end
    plot(d_list, DOR_frequency(i,:), markers{mod(i-1,length(markers))+1}, 'LineWidth', 2);
    legendEntries{end+1} = sprintf('N=%d', frequencyParams(i)); %#ok<SAGROW>
end

grid on;
xlabel('Distance [m]');
ylabel('Average DOR');
title('Frequency-based RMR: Fixed CPM, 200 Vehicles Average DOR');
legend(legendEntries, 'Location', 'best');

%% ===== Distance RMR DOR =====
DOR_distance = nan(length(distanceParams), length(d_list));

for i = 1:length(distanceParams)
    R = distanceParams(i);

    logFile = sprintf('EAR_log_fixedCPM_distance_th%d.mat', R);
    polFile = sprintf('RMRPolicy_fixedCPM_distance_th%d.mat', R);

    if ~isfile(logFile) || ~isfile(polFile)
        warning('Missing file for distance RMR: R=%d', R);
        continue;
    end

    logData = load(logFile);
    polData = load(polFile);

    DOR_distance(i,:) = computeDOR_from_RMRMsg( ...
        logData.EAR_log, polData.RMRPolicy, mydata, ...
        targetVehList, d_list, tStart, tEnd);
end

figure;
hold on;
markers = {'-s','-^','-d','-x','-p','-*','-v'};
legendEntries = {};

if includeNoRMR
    plot(d_list, DOR_NoRMR, 'k-o', 'LineWidth', 2);
    legendEntries{end+1} = 'No RMR'; %#ok<SAGROW>
end

for i = 1:length(distanceParams)
    if all(isnan(DOR_distance(i,:)))
        continue;
    end
    plot(d_list, DOR_distance(i,:), markers{mod(i-1,length(markers))+1}, 'LineWidth', 2);
    legendEntries{end+1} = sprintf('R=%dm', distanceParams(i)); %#ok<SAGROW>
end

grid on;
xlabel('Distance [m]');
ylabel('Average DOR');
title('Distance-based RMR: Fixed CPM, 200 Vehicles Average DOR');
legend(legendEntries, 'Location', 'best');


%% ===== Dynamics RMR DOR =====
DOR_dynamics = nan(length(dynamicPParams), length(d_list));

for i = 1:length(dynamicPParams)
    P = dynamicPParams(i);
    S = dynamicSParam;
    thStr = sprintf('P%s_S%s', num2str(P), num2str(S));

    logFile = sprintf('EAR_log_fixedCPM_dynamics_th%s.mat', thStr);
    polFile = sprintf('RMRPolicy_fixedCPM_dynamics_th%s.mat', thStr);

    if ~isfile(logFile) || ~isfile(polFile)
        warning('Missing file for dynamics RMR: %s', thStr);
        continue;
    end

    logData = load(logFile);
    polData = load(polFile);

    DOR_dynamics(i,:) = computeDOR_from_RMRMsg( ...
        logData.EAR_log, polData.RMRPolicy, mydata, ...
        targetVehList, d_list, tStart, tEnd);
end

figure;
hold on;
markers = {'-s','-^','-d','-x','-p','-*','-v'};
legendEntries = {};

if includeNoRMR
    plot(d_list, DOR_NoRMR, 'k-o', 'LineWidth', 2);
    legendEntries{end+1} = 'No RMR'; %#ok<SAGROW>
end

for i = 1:length(dynamicPParams)
    if all(isnan(DOR_dynamics(i,:)))
        continue;
    end
    plot(d_list, DOR_dynamics(i,:), markers{mod(i-1,length(markers))+1}, 'LineWidth', 2);
    legendEntries{end+1} = sprintf('P=%.1fm, S=%.1fm/s', dynamicPParams(i), dynamicSParam); %#ok<SAGROW>
end

grid on;
xlabel('Distance [m]');
ylabel('Average DOR');
title('Dynamics-based RMR: Fixed CPM, 200 Vehicles Average DOR');
legend(legendEntries, 'Location', 'best');

%% ===== Local functions =====
function DOR_curve = computeDOR_from_RMRMsg(EAR_log, RMRPolicy, mydata, targetVehList, d_list, tStart, tEnd)

DOR_curve = nan(size(d_list));

for dIdx = 1:length(d_list)
    d = d_list(dIdx);
    DOR_eachVeh = nan(length(targetVehList),1);

    for vIdx = 1:length(targetVehList)
        rxVeh = targetVehList(vIdx);

        nveh = compute_nveh(mydata, rxVeh, d, tStart, tEnd);
        if nveh == 0
            continue;
        end

        NV2X = compute_NV2X_from_RMRMsg( ...
            EAR_log, RMRPolicy, mydata, rxVeh, d, tStart, tEnd);

        % DOR can be larger than 1 because duplicated detections are counted.
        DOR_eachVeh(vIdx) = NV2X / nveh;
    end

    DOR_curve(dIdx) = mean(DOR_eachVeh, 'omitnan');
end

end

function nveh = compute_nveh(mydata, rxVeh, d, tStart, tEnd)

vehSet = [];

for t = tStart:tEnd
    posX = squeeze(mydata(:,2,t));
    posY = squeeze(mydata(:,3,t));

    distVec = hypot(posX - posX(rxVeh), posY - posY(rxVeh));

    ids = find(distVec <= d);
    ids(ids == rxVeh) = [];

    vehSet = union(vehSet, ids);
end

nveh = length(vehSet);

end

function NV2X = compute_NV2X_from_RMRMsg(EAR_log, RMRPolicy, mydata, rxVeh, d, tStart, tEnd)

NV2X = 0;

if isempty(EAR_log)
    return;
end

% EAR_log columns assumed:
% [timeStep, txID, rxID, objectID, distance, success]
timeCol = 1;
txCol   = 2;
rxCol   = 3;
succCol = 6;

idx = (EAR_log(:,timeCol) >= tStart) & ...
      (EAR_log(:,timeCol) <= tEnd) & ...
      (EAR_log(:,rxCol) == rxVeh) & ...
      (EAR_log(:,succCol) == 1);

rxLogs = EAR_log(idx,:);

if isempty(rxLogs)
    return;
end

% If EAR_log is stored per object, the same packet may appear multiple times.
% Therefore, count each successfully received CPM packet once.
rxPackets = unique(rxLogs(:, [timeCol, txCol]), 'rows');

for r = 1:size(rxPackets,1)
    t  = rxPackets(r,1);
    tx = rxPackets(r,2);

    if t < 1 || t > size(RMRPolicy.RMRMsg,4)
        continue;
    end

    msgObjects = squeeze(RMRPolicy.RMRMsg(tx,:,:,t));

    if all(isnan(msgObjects(:)))
        continue;
    end

    objectIDs = msgObjects(:,1);
    objectIDs = objectIDs(~isnan(objectIDs));

    if isempty(objectIDs)
        continue;
    end

    posX = squeeze(mydata(:,2,t));
    posY = squeeze(mydata(:,3,t));

    distFromRx = hypot(posX(objectIDs) - posX(rxVeh), ...
                       posY(objectIDs) - posY(rxVeh));

    validObjects = objectIDs(distFromRx <= d);
    validObjects(validObjects == rxVeh) = [];

    % DOR numerator: total received object detections, including duplicates.
    NV2X = NV2X + length(validObjects);
end

end
