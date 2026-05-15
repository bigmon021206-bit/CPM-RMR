%% plotEAR_fixedCPM_20veh_corrected.m
clear; clc; close all;

dataFile = '/Users/bigmo/OneDrive/바탕 화면/WiLabV2Xsim-main/highway_1000m_time1250_veh200_2025-04-21.mat';
data = load(dataFile);
mydata = data.mydata;

targetVehList = 1:200;
d_list = 10:10:700;

tStart = 21;
tEnd   = 25;

distanceParams  = [];
frequencyParams = [100];

%% ===== Fixed CPM No RMR =====
noLog = load('EAR_log_fixedCPM_none_th0.mat');
noPol = load('RMRPolicy_fixedCPM_none_th0.mat');

%EAR_NoRMR = computeEAR_from_RMRMsg( ...
    %noLog.EAR_log, noPol.RMRPolicy, mydata, ...
    %targetVehList, d_list, tStart, tEnd);



%% ===== Frequency RMR =====
EAR_frequency = nan(length(frequencyParams), length(d_list));

for i = 1:length(frequencyParams)

    N = frequencyParams(i);

    logData = load(sprintf('EAR_log_fixedCPM_frequency_th%d.mat', N));
    polData = load(sprintf('RMRPolicy_fixedCPM_frequency_th%d.mat', N));

    EAR_frequency(i,:) = computeEAR_from_RMRMsg( ...
        logData.EAR_log, polData.RMRPolicy, mydata, ...
        targetVehList, d_list, tStart, tEnd);
end

figure;
%plot(d_list, EAR_NoRMR, 'k-o', 'LineWidth', 2); hold on;
plot(d_list, EAR_frequency(1,:), '-s', 'LineWidth', 2);
%plot(d_list, EAR_frequency(2,:), '-^', 'LineWidth', 2);
%plot(d_list, EAR_frequency(3,:), '-d', 'LineWidth', 2);
%plot(d_list, EAR_frequency(4,:), '-x', 'LineWidth', 2);
%plot(d_list, EAR_frequency(5,:), '-p', 'LineWidth', 2);

grid on;
xlabel('Distance [m]');
ylabel('Average EAR');
title('Frequency-based RMR: Fixed CPM, 200 Vehicles Average EAR');
ylim([0.6 1]);

legend({'N=100'}, ...
    'Location', 'best');

%% =====================================================
% Local functions
%% =====================================================
function EAR_curve = computeEAR_from_RMRMsg(EAR_log, RMRPolicy, mydata, targetVehList, d_list, tStart, tEnd)

EAR_curve = nan(size(d_list));

for dIdx = 1:length(d_list)

    d = d_list(dIdx);
    EAR_eachVeh = nan(length(targetVehList),1);

    for vIdx = 1:length(targetVehList)

        rxVeh = targetVehList(vIdx);

        nveh = compute_nveh(mydata, rxVeh, d, tStart, tEnd);

        if nveh == 0
            continue;
        end

        nV2X = compute_nV2X_from_RMRMsg( ...
            EAR_log, RMRPolicy, mydata, rxVeh, d, tStart, tEnd);

        EAR_eachVeh(vIdx) = min(nV2X / nveh, 1);
    end

    EAR_curve(dIdx) = mean(EAR_eachVeh, 'omitnan');

    fprintf('Distance %d m processed\n', d);
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

function nV2X = compute_nV2X_from_RMRMsg(EAR_log, RMRPolicy, mydata, rxVeh, d, tStart, tEnd)

vehSet = [];

if isempty(EAR_log)
    nV2X = 0;
    return;
end

% EAR_log column 가정:
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

for r = 1:size(rxLogs,1)

    t = rxLogs(r,timeCol);
    tx = rxLogs(r,txCol);

    if t < 1 || t > size(RMRPolicy.RMRMsg,4)
        continue;
    end

    msgObjects = squeeze(RMRPolicy.RMRMsg(tx,:,:,t));

    if all(isnan(msgObjects(:)))
        continue;
    end

    objectIDs = msgObjects(:,1);
    objectIDs = objectIDs(~isnan(objectIDs));

    posX = squeeze(mydata(:,2,t));
    posY = squeeze(mydata(:,3,t));

    distFromRx = hypot(posX(objectIDs) - posX(rxVeh), ...
                       posY(objectIDs) - posY(rxVeh));

    validObjects = objectIDs(distFromRx <= d);
    validObjects(validObjects == rxVeh) = [];

    vehSet = union(vehSet, validObjects);
end

nV2X = length(vehSet);

end

%% ===== Distance RMR =====
EAR_distance = nan(length(distanceParams), length(d_list));

for i = 1:length(distanceParams)

    R = distanceParams(i);

    logData = load(sprintf('EAR_log_fixedCPM_distance_th%d.mat', R));
    polData = load(sprintf('RMRPolicy_fixedCPM_distance_th%d.mat', R));

    EAR_distance(i,:) = computeEAR_from_RMRMsg( ...
        logData.EAR_log, polData.RMRPolicy, mydata, ...
        targetVehList, d_list, tStart, tEnd);
end

figure;
%plot(d_list, EAR_NoRMR, 'k-o', 'LineWidth', 2); hold on;
%plot(d_list, EAR_distance(1,:), '-s', 'LineWidth', 2);
%plot(d_list, EAR_distance(2,:), '-^', 'LineWidth', 2);
%plot(d_list, EAR_distance(3,:), '-d', 'LineWidth', 2);
%plot(d_list, EAR_distance(4,:), '-x', 'LineWidth', 2);

grid on;
xlabel('Distance [m]');
ylabel('Average EAR');
title('Distance-based RMR: Fixed CPM, 200 Vehicles Average EAR');
ylim([0.6 1]);

legend({'No RMR', 'R=5m'}, ...
    'Location', 'best');
