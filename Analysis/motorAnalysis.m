voltage     = 12;   % volts
stallTorque = 110;  % oz-in
noLoadSpeed = 350;  % rpm
noLoadCur   = .3;   % mA
stallCur    = 5;    % A


speed  = 0:.5:noLoadSpeed;
torque = stallTorque - (stallTorque * speed /noLoadSpeed);
power  = (torque .* speed) / 1352.2943390876; % power in watts

% plotting
figure(1);
hold on;
fontsize = 24;
[ax, T, P] = plotyy(speed, torque,speed, power);
set(ax, 'Fontsize', fontsize);
T.LineWidth = 5;
P.LineWidth = 5;
ylabel(ax(1), "Torque (oz-in)", 'fontsize', fontsize);
ylabel(ax(2), "Output Power (W)", 'fontsize', fontsize);
xlabel("Speed (RPM)", 'fontsize', fontsize);
title("Output Torque and Power vs Speed, 30:1 Pololu Motor", 'fontsize', fontsize);

