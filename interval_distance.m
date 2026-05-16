clear;
%% 명령창에 붙여넣어서 패킷 생성 주기의 분포를 볼 수 있는 코드
%% 원하는 파일로 변경
%load('RMRPolicy_fixedCPM_frequency_th5.mat');
load('RMRPolicy_fixedCPM_distance_th5.mat');

dt = 0.1;
flag = RMRPolicy.RMRGenerateFlag;

allIntervals = [];

for tx = 1:size(flag,1)
    txSteps = find(flag(tx,:) == 1);

    if numel(txSteps) >= 2
        intervals = diff(txSteps(:)) * dt;
        allIntervals = [allIntervals; intervals];
    end
end

allIntervals = round(allIntervals, 2);

%% 0.1~1.0초 전체 보기
figure;

edges = 0.05:0.1:1.05;   % 1.0초가 마지막 bin 중앙에 들어가도록 설정

histogram(allIntervals, ...
    'BinEdges', edges, ...
    'FaceAlpha', 0.85, ...
    'LineWidth', 1.5);

grid on;
xlabel('Transmission Interval [s]');
ylabel('Count');
title('Transmission Interval Distribution');

xlim([0.05 1.05]);
xticks(0.1:0.1:1.0);

fprintf('\n===== Interval Statistics =====\n');
fprintf('Total interval samples = %d\n', length(allIntervals));
fprintf('Mean interval   = %.4f s\n', mean(allIntervals));
fprintf('Median interval = %.4f s\n', median(allIntervals));
fprintf('Min interval    = %.4f s\n', min(allIntervals));
fprintf('Max interval    = %.4f s\n', max(allIntervals));

disp('Unique intervals:');
disp(unique(allIntervals));
