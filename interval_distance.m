clear; clc;

%% ===== 파일 설정 =====
files = {
    'RMRPolicy_fixedCPM_none_th0.mat',      'No RMR';
    'RMRPolicy_fixedCPM_distance_th7.mat',  'R=7m';
    'RMRPolicy_fixedCPM_distance_th15.mat',  'R=15m';
    'RMRPolicy_fixedCPM_distance_th25.mat', 'R=25m'
};

dt = 0.1;

minStep = 1;   % 0.1s
maxStep = 10;  % 1.0s

intervalSteps = minStep:maxStep;
intervalSec = intervalSteps * dt;

ratioMat = zeros(length(files), length(intervalSteps));

%% ===== 각 케이스별 반복 =====
for fIdx = 1:size(files,1)

    load(files{fIdx,1});  % RMRPolicy 로드
    flag = RMRPolicy.RMRGenerateFlag;

    allIntervals = [];

    for tx = 1:size(flag,1)
        txSteps = find(flag(tx,:) == 1);

        if numel(txSteps) >= 2
            intervals = diff(txSteps(:)); % step 단위
            allIntervals = [allIntervals; intervals];
        end
    end

    %% ===== 0.1~1.0초만 필터 =====
    allIntervals = allIntervals( ...
        allIntervals >= minStep & ...
        allIntervals <= maxStep);

    %% ===== count & ratio =====
    count = zeros(size(intervalSteps));

    for i = 1:length(intervalSteps)
        count(i) = sum(allIntervals == intervalSteps(i));
    end

    ratio = count / sum(count);

    ratioMat(fIdx,:) = ratio;
end

%% ===== Plot =====
figure;
bar(intervalSec*1000, ratioMat' * 100);  % ms, %

grid on;
xlabel('CPM Generation Interval [ms]');
ylabel('Ratio [%]');
title('CPM Generation Interval Ratio Comparison');

xticks(100:100:1000);
xlim([50 1050]);
ylim([0 100]);

legend(files(:,2), 'Location', 'best');
