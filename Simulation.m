close all
clear
clc

packetSize=681;
nTransm=1;
sizeSubchannel=10;
Raw = [50, 150, 300];
speed=70;
speedStDev=7;
SCS=15;
pKeep=0.4;
sensingThreshold=-126;

configFile = 'Highway3GPP.cfg';

%% ===== 데이터 로드 =====
dataFile = '/Users/bigmo/OneDrive/바탕 화면/WiLabV2Xsim-main/highway_1000m_time1250_veh200_2025-04-21.mat';
data = load(dataFile);
mydata = data.mydata;

[numVeh, numFeat, numSteps] = size(mydata);
testSteps = 40;

%% ===== LOS / NLOS analysis using mydata =====
%vehLength = 4.5;
%vehWidth  = 1.8;
%sensorRange = 150;

%LOSMat = false(numVeh, numVeh, testSteps);
%inRangeMat = false(numVeh, numVeh, testSteps);
%headingAll = zeros(numVeh, numSteps);

%disp('Starting LOS/NLOS analysis ...');

%for t = 2:testSteps
    %posX  = squeeze(mydata(:,2,t));
    %posY  = squeeze(mydata(:,3,t));

    %posX_prev = squeeze(mydata(:,2,t-1));
    %posY_prev = squeeze(mydata(:,3,t-1));

    %heading = atan2(posY - posY_prev, posX - posX_prev);
    %headingAll(:,t) = heading;

    %for i = 1:numVeh
        %for j = 1:numVeh
            %if j == i
                %continue;
            %end

            %dx = posX(i) - posX(j);
            %dy = posY(i) - posY(j);
            %dist = sqrt(dx^2 + dy^2);

            %if dist > sensorRange
                %continue;
            %end

            %inRangeMat(i,j,t) = true;
            %los = isLOS_pair(i, j, posX, posY, heading, vehLength, vehWidth);
            %LOSMat(i,j,t) = los;
        %end
    %end

    %if mod(t,10) == 0 || t == testSteps
        %fprintf('Processed t = %d / %d\n', t, testSteps);
    %end
%end

%save('los_result.mat', 'LOSMat', 'inRangeMat', 'headingAll', ...
     %'vehLength', 'vehWidth', 'sensorRange', '-v7.3');

%disp('LOS/NLOS calculation completed and saved to los_result.mat');



%% ===== LOS 데이터 글로벌 등록 =====
global LOSMat
tmp = load('los_result.mat','LOSMat');
LOSMat = tmp.LOSMat;

%% === =====
BandMHz = 10;
if BandMHz == 10
    MCS = 7;
elseif BandMHz == 20
    MCS = 5;
end

rho = 100;
simTime = 4;




%% =====================================================
% Fixed-size CPM RMR
%% =====================================================

RMRExperiments = {
    
    'frequency', 100;
    
};

global RMRPolicyGlobal
global useRMRScheduling
global EAR_log

for eIdx = 1:size(RMRExperiments,1)

    RMRMode   = RMRExperiments{eIdx,1};
    threshold = RMRExperiments{eIdx,2};

    fprintf('\n==============================\n');
    fprintf('Running Fixed CPM RMR: %s, threshold = %g\n', RMRMode, threshold);
    fprintf('==============================\n');

    RMRPolicy = buildFixedCPM_ObjectRMRPolicy(mydata, LOSMat, RMRMode, threshold);

    RMRPolicyGlobal = RMRPolicy;
    useRMRScheduling = true;

    EAR_log = zeros(0,6);

    outputFolder = sprintf('Output/FixedCPM_%s_th%s', RMRMode, num2str(threshold));

    WiLabV2Xsim(configFile, ...
        'outputFolder', outputFolder, ...
        'Technology', '5G-V2X', ...
        'MCS_NR', MCS, ...
        'SCS_NR', SCS, ...
        'beaconSizeBytes', packetSize, ...
        'simulationTime', simTime, ...
        'rho', rho, ...
        'probResKeep', pKeep, ...
        'BwMHz', BandMHz, ...
        'vMean', speed, ...
        'vStDev', speedStDev, ...
        'cv2xNumberOfReplicasMax', nTransm, ...
        'allocationPeriod', 0.1, ...
        'sizeSubchannel', sizeSubchannel, ...
        'powerThresholdAutonomous', sensingThreshold, ...
        'Raw', Raw, ...
        'FixedPdensity', false, ...
        'dcc_active', false, ...
        'cbrActive', true);

    save(sprintf('EAR_log_fixedCPM_%s_th%s.mat', RMRMode, num2str(threshold)), ...
        'EAR_log', '-v7.3');
end


%% =====================================================
% Fixed-size CPM No RMR
%% =====================================================

global useRMRScheduling
global RMRPolicyGlobal
global EAR_log

fprintf('\n==============================\n');
fprintf('Running Fixed-size CPM No RMR\n');
fprintf('==============================\n');

RMRPolicy = buildFixedCPM_ObjectRMRPolicy(mydata, LOSMat, 'none', 0);

RMRPolicyGlobal = RMRPolicy;
useRMRScheduling = true;

EAR_log = zeros(0,6);

outputFolder = 'Output/FixedCPM_NoRMR';

WiLabV2Xsim(configFile, ...
    'outputFolder', outputFolder, ...
    'Technology', '5G-V2X', ...
    'MCS_NR', MCS, ...
    'SCS_NR', SCS, ...
    'beaconSizeBytes', packetSize, ...
    'simulationTime', simTime, ...
    'rho', rho, ...
    'probResKeep', pKeep, ...
    'BwMHz', BandMHz, ...
    'vMean', speed, ...
    'vStDev', speedStDev, ...
    'cv2xNumberOfReplicasMax', nTransm, ...
    'allocationPeriod', 0.1, ...
    'sizeSubchannel', sizeSubchannel, ...
    'powerThresholdAutonomous', sensingThreshold, ...
    'Raw', Raw, ...
    'FixedPdensity', false, ...
    'dcc_active', false, ...
    'cbrActive', true);

save('EAR_log_fixedCPM_none_th0.mat', 'EAR_log', '-v7.3');


% %% ===== 100ms 간격 LOS/NLOS 리스트 출력 =====
% targetVeh = 1;          
% stepDurationMs = 100;   % 1 step = 100ms
% 
% stepList = 2:10;        % 예시: 2~10 step 출력
% timeMsList = (stepList - 1) * stepDurationMs;
% 
% fprintf('\n===== Vehicle %d LOS/NLOS over time =====\n', targetVeh);
% 
% for idx = 1:length(stepList)
% 
%     tCheck = stepList(idx);
% 
%     if tCheck > numSteps
%         warning('Step %d exceeds numSteps. Skipping.', tCheck);
%         continue;
%     end
% 
%     losList  = find(inRangeMat(tCheck,:) & LOSMat(tCheck,:));
%     nlosList = find(inRangeMat(tCheck,:) & ~LOSMat(tCheck,:));
%     outRangeList = find(~inRangeMat(tCheck,:));
% 
%     losList(losList == targetVeh) = [];
%     nlosList(nlosList == targetVeh) = [];
%     outRangeList(outRangeList == targetVeh) = [];
% 
%     fprintf('\n--- %d ms (step = %d) ---\n', timeMsList(idx), tCheck);
% 
%     fprintf('LOS (%d): ', length(losList));
%     if isempty(losList)
%         fprintf('(none)');
%     else
%         fprintf('%d ', losList);
%     end
%     fprintf('\n');
% 
%     fprintf('NLOS (%d): ', length(nlosList));
%     if isempty(nlosList)
%         fprintf('(none)');
%     else
%         fprintf('%d ', nlosList);
%     end
%     fprintf('\n');
% 
%     fprintf('Out-of-range (%d): ', length(outRangeList));
%     if isempty(outRangeList)
%         fprintf('(none)');
%     else
%         fprintf('%d ', outRangeList);
%     end
%     fprintf('\n');
% end
% 
% %% ===== 전체 차량 body + 차량 1번 센서 range 시각화 =====
% tPlot = 100;      % 보고 싶은 step
% targetVeh = 1;    % 기준 차량
% 
% if tPlot < 2 || tPlot > numSteps
%     error('tPlot must be between 2 and numSteps.');
% end
% 
% if targetVeh > numVeh
%     error('targetVeh exceeds numVeh.');
% end
% 
% % 현재/이전 step 위치
% posX = squeeze(mydata(:,2,tPlot));
% posY = squeeze(mydata(:,3,tPlot));
% posX_prev = squeeze(mydata(:,2,tPlot-1));
% posY_prev = squeeze(mydata(:,3,tPlot-1));
% 
% % heading 계산
% heading = atan2(posY - posY_prev, posX - posX_prev);
% 
% % 차량 1번 기준 LOS / NLOS
% losList  = find(inRangeMat(targetVeh,:,tPlot) & LOSMat(targetVeh,:,tPlot));
% nlosList = find(inRangeMat(targetVeh,:,tPlot) & ~LOSMat(targetVeh,:,tPlot));
% outRangeList = find(~inRangeMat(targetVeh,:,tPlot));
% 
% % 자기 자신 제외
% losList(losList == targetVeh) = [];
% nlosList(nlosList == targetVeh) = [];
% outRangeList(outRangeList == targetVeh) = [];
% 
% figure; hold on; axis equal;
% title(sprintf('Vehicle Bodies + Sensor Range of Vehicle %d at step = %d', targetVeh, tPlot));
% xlabel('X');
% ylabel('Y');
% 
% % 전체 차량 body 그리기
% for i = 1:numVeh
%     rect = getVehicleRectangle(posX(i), posY(i), heading(i), vehLength, vehWidth);
% 
%     if i == targetVeh
%         % 기준 차량: 빨강
%         patch(rect(:,1), rect(:,2), 'red', 'FaceAlpha', 0.6, 'EdgeColor', 'r');
%     elseif ismember(i, losList)
%         % LOS 차량: 초록
%         patch(rect(:,1), rect(:,2), 'green', 'FaceAlpha', 0.4, 'EdgeColor', 'g');
%     elseif ismember(i, nlosList)
%         % NLOS 차량: 노랑
%         patch(rect(:,1), rect(:,2), 'yellow', 'FaceAlpha', 0.4, 'EdgeColor', 'y');
%     else
%         % 범위 밖: 하늘색
%         patch(rect(:,1), rect(:,2), 'cyan', 'FaceAlpha', 0.2, 'EdgeColor', 'b');
%     end
% 
%     plot(posX(i), posY(i), 'k.', 'MarkerSize', 8);
%     text(posX(i), posY(i), num2str(i), 'FontSize', 7, 'Color', 'k');
% end
% 
% % 차량 1번의 센서 range 원 그리기
% theta = linspace(0, 2*pi, 300);
% xCircle = posX(targetVeh) + sensorRange * cos(theta);
% yCircle = posY(targetVeh) + sensorRange * sin(theta);
% plot(xCircle, yCircle, 'r--', 'LineWidth', 1.5);
% 
% % 차량 1번 중심 강조
% plot(posX(targetVeh), posY(targetVeh), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
% 
% legend({'Target vehicle','LOS','NLOS','Out-of-range','Sensor range'}, ...
%        'Location', 'bestoutside');
% 
% hold off;
