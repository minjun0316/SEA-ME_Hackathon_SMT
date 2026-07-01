function plot_logs(csv_path)
% PLOT_LOGS  Stage 5: 주행 로그(CSV)로부터 제어 성능 분석 그래프 생성.
%
%   plot_logs('sim_log.csv')
%
% 입력 CSV는 sim/logger.py 의 고정 스키마를 따른다:
%   time, x, y, yaw, target_x, target_y, lookahead, curvature,
%   cross_track_error, heading_error, steering_cmd, speed_cmd,
%   battery_voltage, mission_state
%
% 시뮬 로그와 실차 로그(rosbag→CSV 변환)를 동일하게 분석할 수 있다.

    if nargin < 1
        csv_path = 'sim_log.csv';
    end

    T = readtable(csv_path);

    figure('Name', 'D-Racer Controller Analysis', 'Position', [100 100 1200 900]);

    % 1) Reference Path vs Vehicle Path
    subplot(4,2,1);
    plot(T.x, T.y, 'b-', 'LineWidth', 1.5); hold on;
    plot(T.target_x, T.target_y, 'r.', 'MarkerSize', 4);
    axis equal; grid on;
    title('Reference vs Vehicle Path'); xlabel('x [m]'); ylabel('y [m]');
    legend('Vehicle', 'Lookahead target', 'Location', 'best');

    % 2) Cross Track Error
    subplot(4,2,2);
    plot(T.time, T.cross_track_error, 'LineWidth', 1.2); grid on;
    yline(0, 'k-'); title('Cross Track Error'); xlabel('time [s]'); ylabel('CTE [m]');

    % 3) Heading Error
    subplot(4,2,3);
    plot(T.time, T.heading_error, 'LineWidth', 1.2); grid on;
    title('Heading Error'); xlabel('time [s]'); ylabel('[rad]');

    % 4) Steering
    subplot(4,2,4);
    plot(T.time, T.steering_cmd, 'LineWidth', 1.2); grid on;
    title('Steering Command'); xlabel('time [s]'); ylabel('[-1, 1]');

    % 5) Speed
    subplot(4,2,5);
    plot(T.time, T.speed_cmd, 'LineWidth', 1.2); grid on;
    title('Speed Command'); xlabel('time [s]'); ylabel('speed');

    % 6) Lookahead
    subplot(4,2,6);
    plot(T.time, T.lookahead, 'LineWidth', 1.2); grid on;
    title('Lookahead'); xlabel('time [s]'); ylabel('[m]');

    % 7) Curvature
    subplot(4,2,7);
    plot(T.time, T.curvature, 'LineWidth', 1.2); grid on;
    title('Curvature'); xlabel('time [s]'); ylabel('[1/m]');

    % 8) Battery Voltage
    subplot(4,2,8);
    plot(T.time, T.battery_voltage, 'LineWidth', 1.2); grid on;
    title('Battery Voltage'); xlabel('time [s]'); ylabel('[V]');

    sgtitle('D-Racer Controller Analysis (Stage 5)');

    % 요약 지표 출력
    fprintf('RMS CTE : %.2f cm\n', rms(T.cross_track_error) * 100);
    fprintf('MAX CTE : %.2f cm\n', max(abs(T.cross_track_error)) * 100);
end
