function RMRPolicy = buildFixedCPM_ObjectRMRPolicy(mydata, LOSMat, RMRMode, threshold)

dt = 0.1;
historySteps = 10;      % 1초 window
bufferTimeout = 10;     % buffer도 1초 지나면 제거
sensorRange = 150;
maxPOC = 15;

numVeh = size(mydata,1);
numSteps = 40;

RMRGenerateFlag = zeros(numVeh, numSteps);
RMRMsg = nan(numVeh, maxPOC, 5, numSteps);

objectBuffer = cell(numVeh,1);
bufferTime   = cell(numVeh,1);

T_GenCpmMax = 1.0;
maxWaitSteps = round(T_GenCpmMax / dt);

lastTxStep = zeros(numVeh,1);

for tx = 1:numVeh
    objectBuffer{tx} = [];
    bufferTime{tx} = [];
end

% rxHistory{rx} = [objectID, senderID, timeStep]
rxHistory = cell(numVeh,1);

for rx = 1:numVeh
    rxHistory{rx} = zeros(0,3);
end

commRange = 300;   % 수신 가능 거리. 일단 Raw 최대값 300m 기준

for t = 1:numSteps

    posX  = squeeze(mydata(:,2,t));
    posY  = squeeze(mydata(:,3,t));
    speed = squeeze(mydata(:,5,t));

    % 각 차량이 "수신한 CPM history"를 최근 1초만 유지
    for rx = 1:numVeh
        rxHistory{rx} = rxHistory{rx}(rxHistory{rx}(:,3) >= t-historySteps, :);
    end

    % 이번 step에서 생성된 CPM 목록
    % [objectID, senderID, timeStep]
    txThisStep = [];

    for tx = 1:numVeh

        %% 1. 송신 차량 기준 LOS 객체 추출
        distFromTx = hypot(posX - posX(tx), posY - posY(tx));

        losIDs = find(LOSMat(tx,:,t));
        losIDs(losIDs == tx) = [];
        losIDs = losIDs(distFromTx(losIDs) <= sensorRange);

        %% 2. 현재 LOS가 아닌 buffer 객체 제거
        [objectBuffer{tx}, bufferTime{tx}] = removeInvalidBuffer( ...
            objectBuffer{tx}, bufferTime{tx}, losIDs, t, bufferTimeout);

        %% 3. buffer에 없는 LOS 객체만 RMR 적용
        newIDs = setdiff(losIDs, objectBuffer{tx}, 'stable');
        passedIDs = [];

        for k = 1:length(newIDs)
            objID = newIDs(k);

            keepObj = applyObjectRMR_ReceivedBased( ...
                objID, tx, posX, posY, rxHistory{tx}, RMRMode, threshold);

            if keepObj
                passedIDs(end+1) = objID; %#ok<AGROW>
            end
        end

        %% 4. 통과 객체 buffer에 추가
        objectBuffer{tx} = [objectBuffer{tx}, passedIDs];
        bufferTime{tx}   = [bufferTime{tx}, t * ones(1,length(passedIDs))];

        %% 5. 전송 조건 판단
        enoughObjects = length(objectBuffer{tx}) >= maxPOC;

        if lastTxStep(tx) == 0
            waitedTooLong = (t >= maxWaitSteps);
        else
            waitedTooLong = (t - lastTxStep(tx)) >= maxWaitSteps;
        end

        if enoughObjects || waitedTooLong

            bufIDs = objectBuffer{tx};

            %% 1초가 지났는데 15대 미만이면 RMR 미적용 LOS 후보로 부족분 채우기
            if length(bufIDs) < maxPOC

                shortage = maxPOC - length(bufIDs);

                % 현재 시점 LOS 후보 전체에서 buffer에 없는 차량 선택
                fillCandidates = setdiff(losIDs, bufIDs, 'stable');

                % 가까운 순서로 정렬
                distFill = hypot(posX(fillCandidates) - posX(tx), ...
                                posY(fillCandidates) - posY(tx));

                [~, fillOrder] = sort(distFill, 'ascend');

                numFill = min(shortage, length(fillCandidates));

                if numFill > 0
                    fillIDs = fillCandidates(fillOrder(1:numFill));
                    bufIDs = [bufIDs, fillIDs];
                end
            end

            %% 그래도 객체가 하나도 없으면 전송하지 않음
            if isempty(bufIDs)
                continue;
            end

            %% 최종적으로 최대 15개 선택
            distBuf = hypot(posX(bufIDs) - posX(tx), ...
                            posY(bufIDs) - posY(tx));

            [~, order] = sort(distBuf, 'ascend');

            numSelect = min(maxPOC, length(bufIDs));
            selectedIDs = bufIDs(order(1:numSelect));

            msgObjects = nan(maxPOC,5);

            for s = 1:numSelect
                obj = selectedIDs(s);

                msgObjects(s,:) = [ ...
                    obj, ...
                    posX(obj), ...
                    posY(obj), ...
                    speed(obj), ...
                    hypot(posX(obj)-posX(tx), posY(obj)-posY(tx)) ...
                ];
            end

            RMRMsg(tx,:,:,t) = msgObjects;
            RMRGenerateFlag(tx,t) = 1;

            % 전송 history 저장
            txThisStep = [txThisStep; ...
                [selectedIDs(:), tx*ones(numSelect,1), t*ones(numSelect,1)]]; %#ok<AGROW>

            % 전송한 객체는 buffer에서 제거
            removeMask = ismember(objectBuffer{tx}, selectedIDs);
            objectBuffer{tx}(removeMask) = [];
            bufferTime{tx}(removeMask) = [];

            % 마지막 전송 step 갱신
            lastTxStep(tx) = t;
        end
    end
    % 이번 step에서 전송된 CPM을 주변 차량들이 수신했다고 가정하고 rxHistory에 저장
    % 실제 PHY 성공 여부까지 반영하려면 mainV2X 수신 성공 이후에 업데이트해야 하지만,
    % policy pre-build 방식에서는 거리 기반 수신 가능 범위로 근사함.
    for row = 1:size(txThisStep,1)

        objID    = txThisStep(row,1);
        senderID = txThisStep(row,2);
        timeStep = txThisStep(row,3);

        distFromSender = hypot(posX - posX(senderID), posY - posY(senderID));

        receivers = find(distFromSender <= commRange);
        receivers(receivers == senderID) = [];

        for rr = 1:length(receivers)
            rx = receivers(rr);
            rxHistory{rx} = [rxHistory{rx}; objID, senderID, timeStep]; %#ok<AGROW>
        end
    end

    fprintf('[%s th=%g] step %d / %d\n', RMRMode, threshold, t, numSteps);
end

RMRPolicy.RMRGenerateFlag = RMRGenerateFlag;
RMRPolicy.RMRMsg = RMRMsg;
RMRPolicy.RMRMode = RMRMode;
RMRPolicy.threshold = threshold;
RMRPolicy.maxPOC = maxPOC;
RMRPolicy.historySteps = historySteps;

save(sprintf('RMRPolicy_fixedCPM_%s_th%s.mat', RMRMode, num2str(threshold)), ...
    'RMRPolicy', '-v7.3');

end

function [bufIDs, bufTime] = removeInvalidBuffer(bufIDs, bufTime, losIDs, t, bufferTimeout)

if isempty(bufIDs)
    return;
end

stillLOS = ismember(bufIDs, losIDs);
notOld = (t - bufTime) <= bufferTimeout;

keepMask = stillLOS & notOld;

bufIDs = bufIDs(keepMask);
bufTime = bufTime(keepMask);

end

function keepObj = applyObjectRMR_ReceivedBased(objID, tx, posX, posY, rxHistTx, RMRMode, threshold)

keepObj = true;

% 현재 송신 차량(tx)이 최근에 수신한 CPM 중,
% 같은 objectID가 포함된 이력만 추출
objHist = rxHistTx(rxHistTx(:,1) == objID, :);

switch lower(RMRMode)

    case 'none'
        keepObj = true;

    case 'distance'
        % Distance-based RMR:
        % tx가 최근에 받은 CPM 중 같은 object를 보낸 sender가
        % 현재 tx와 R_Redundancy 이내에 있으면 redundant로 판단
        for h = 1:size(objHist,1)

            senderID = objHist(h,2);

            distTxSender = hypot(posX(tx) - posX(senderID), ...
                                 posY(tx) - posY(senderID));

            if distTxSender <= threshold
                keepObj = false;
                return;
            end
        end

    case 'frequency'
        % Frequency-based RMR:
        % tx가 최근 time window 동안 같은 object update를
        % threshold번 이상 수신했으면 redundant로 판단
        receivedUpdateCount = size(objHist,1);

        if receivedUpdateCount >= threshold
            keepObj = false;
            return;
        end

    otherwise
        error('Unknown RMRMode');
end

end
