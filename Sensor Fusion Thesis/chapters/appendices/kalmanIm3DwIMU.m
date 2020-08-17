    % function kalman(duration, dt)
    %
    % Kalman filter simulation for a drone's 3D position and velocity with roll, pitch and yaw.
    %
    % State [posx(cm); posy(cm); posz(cm); velx(cm/s); vely(cm/s); velz(cm/s); roll(o); pitch(o); yaw(o)]
    %
    % E = covariance
    %
    close all;

    %%%%TESTING PURPOSES%%%%
    % INITIALISE DATA GATHERING
    TotalPosxErrorExt = 0;   % cumulative position extrapolation error
    TotalPosyErrorExt = 0;   % cumulative position extrapolation error
    TotalPoszErrorExt = 0;   % cumulative position extrapolation error
    TotalPosxErrorMeas = 0;  % cumulative position measurement error
    TotalPosyErrorMeas = 0;  % cumulative position measurement error  
    TotalPoszErrorMeas = 0;  % cumulative position measurement error 
    TotalPosxErrorFil = 0;   % cumulative position error  
    TotalPosyErrorFil = 0;   % cumulative position error
    TotalPoszErrorFil = 0;   % cumulative position error
    TotalVelxErrorExt = 0;   % cumulative velocity extrapolation error
    TotalVelyErrorExt = 0;   % cumulative velocity extrapolation error
    TotalVelzErrorExt = 0;   % cumulative velocity extrapolation error
    TotalVelxErrorMeas = 0;  % cumulative velocity measurement error
    TotalVelyErrorMeas = 0;  % cumulative velocity measurement error  
    TotalVelzErrorMeas = 0;  % cumulative velocity measurement error 
    TotalVelxErrorFil = 0;   % cumulative velocity error  
    TotalVelyErrorFil = 0;   % cumulative velocity error
    TotalVelzErrorFil = 0;   % cumulative velocity error
    TotalRollErrorExt = 0;   % cumulative position extrapolation error
    TotalPitchErrorExt = 0;  % cumulative position extrapolation error
    TotalYawErrorExt = 0;    % cumulative position extrapolation error
    TotalRollErrorMeas = 0;  % cumulative position measurement error
    TotalPitchErrorMeas = 0; % cumulative position measurement error  
    TotalYawErrorMeas = 0;   % cumulative position measurement error 
    TotalRollErrorFil = 0;   % cumulative position error  
    TotalPitchErrorFil = 0;  % cumulative position error
    TotalYawErrorFil = 0;    % cumulative position error
    TotalSensoravgError = 0;
    %%%%TESTING PURPOSES%%%%

    % INPUTS
    duration = 20;    %length of simulation (seconds)
    dt = 0.1;          %step size (seconds)

    % Measurement noise due to imperfect sensors.
    sensornoise = [4     % ultra x1 measurement noise (cm)
        4     % ultra x2 measurement noise (cm)
        4     % ultra y1 measurement noise (cm)
        4     % ultra y2 measurement noise (cm)
        4     % ultra z1 measurement noise (cm)
        4     % ultra z2 measurement noise (cm)
        1     % laser x1 measurement noise (cm)
        1     % laser x2 measurement noise (cm)
        1     % laser y1 measurement noise (cm)
        1     % laser y2 measurement noise (cm)
        1     % laser z1 measurement noise (cm)
        1     % laser z1 measurement noise (cm)
        10    % disparity x measurement noise (cm)
        200   % GPS z measurement noise (cm)
        200   % GPS velocity x measurement noise (cm/s)
        200   % GPS velocity y measurement noise (cm/s)
        200   % GPS velocity z measurement noise (cm/s)
        10    % barometer z measurement noise (cm)
        8.75  % IMU1 angular velocity roll measurement noise (o/s)
        8.75  % IMU1 angular velocity pitch measurement noise (o/s)
        8.75  % IMU1 angular velocity yaw measurement noise (o/s)
        5     % IMU2 angular velocity roll measurement noise (o)
        5     % IMU2 angular velocity pitch measurement noise (o)
        5];   % IMU2 angular velocity yaw measurement noise (o)

    % External noise due to imperfect replication of instruction wrt the environment.
    externalnoise = [1      % x acceleration noise (cm/sec^2)
                     1      % y acceleration noise (cm/sec^2)
                     1      % z acceleration noise (cm/sec^2)
                     1      % roll input noise (o/sec)
                     1      % pitch input noise (o/sec)
                     1];    % yaw input noise (o/sec)

    % Computational matrices.
    F = [1 0 0 dt 0 0 0 0 0
         0 1 0 0 dt 0 0 0 0
         0 0 1 0 0 dt 0 0 0
         0 0 0 1 0 0 0 0 0
         0 0 0 0 1 0 0 0 0
         0 0 0 0 0 1 0 0 0
         0 0 0 0 0 0 1 0 0
         0 0 0 0 0 0 0 1 0
         0 0 0 0 0 0 0 0 1 ]; % transition matrix, used to extrapolate the new state from the previous acceleration
    B = [dt^2/2 0 0 0 0 0
         0 dt^2/2 0 0 0 0
         0 0 dt^2/2 0 0 0
         dt 0 0 0 0 0
         0 dt 0 0 0 0
         0 0 dt 0 0 0
         0 0 0 dt 0 0
         0 0 0 0 dt 0
         0 0 0 0 0 dt    ]; % input matrix, used to account for the effect of inputs on the new state when extrapolating from the old
    H = [1 0 0 0 0 0 0 0 0
         1 0 0 0 0 0 0 0 0
         0 1 0 0 0 0 0 0 0
         0 1 0 0 0 0 0 0 0
         0 0 1 0 0 0 0 0 0
         0 0 1 0 0 0 0 0 0
         1 0 0 0 0 0 0 0 0
         1 0 0 0 0 0 0 0 0
         0 1 0 0 0 0 0 0 0
         0 1 0 0 0 0 0 0 0
         0 0 1 0 0 0 0 0 0
         0 0 1 0 0 0 0 0 0
         1 0 0 0 0 0 0 0 0
         0 0 1 0 0 0 0 0 0
         0 0 0 1 0 0 0 0 0
         0 0 0 0 1 0 0 0 0
         0 0 0 0 0 1 0 0 0
         0 0 1 0 0 0 0 0 0
         0 0 0 0 0 0 1 0 0
         0 0 0 0 0 0 0 1 0
         0 0 0 0 0 0 0 0 1
         0 0 0 0 0 0 1 0 0
         0 0 0 0 0 0 0 1 0
         0 0 0 0 0 0 0 0 1]; % measurement matrix, maps the sensor measurements onto the state space (has to be hand altered upon change of sensor input or state space)

    for i = 1;

    % Initial state.
    x = [0; 0; 0; 0; 0; 0; 0; 0; 0]; % initial state vector, [position x/y/z; velocity x/y/z; orientation roll/pitch/yaw]
    xhat = x; % initial state estimate (accurate)
    %xhat = [1000; 2000; 1500; 20; 40; 30; 0; 0; 0];    % initial state estimate (custom, comment out if not needed)
    R = diag(sensornoise).^2;  % measurement error E
    Qu = repmat(cat(1,externalnoise(1),externalnoise(2),externalnoise(3),externalnoise(1),externalnoise(2),externalnoise(3),externalnoise(4),externalnoise(5),externalnoise(6)).^2,1,length(x)) .* (B * B'); % process noise E (of xhat); describes the effect of process noise on estimated position and velocity
    P = Qu; % initial estimation E (of xhat)

    % Initialize arrays for later plotting.
    pos = [];       % true position array
    poshat = [];    % estimated position array
    posmeas = [];   % measured position array
    vel = [];       % true velocity array
    velmeas = [];   % measures velocity array
    velhat = [];    % estimated velocity array

    for t = 0 : dt: duration,
        % Use a constant commanded acceleration in cm/sec^2 and a constant commanded rotation in o/sec.
        u = [randn; randn; randn; 1; 1; 1];
        % Simulate the linear system.
        ProcessNoise = cat(1,externalnoise(1),externalnoise(2),externalnoise(3),externalnoise(1),externalnoise(2),externalnoise(3),externalnoise(4),externalnoise(5),externalnoise(6)) .* ((randn(length(x),1))); % generates process noise for each state element
        x = F * x + B * u + ProcessNoise;
        % Simulate the noisy measurements
        MeasNoise = sensornoise .* randn(length(sensornoise),1);    % randomly weighted measurement noise
        z = H * x + MeasNoise;   % actual position with added noise to create simulated (mean) measurement readings
        % Extrapolate the most recent state estimate to the present time;
        % this is the new prediction based on extrapolation from the position 
        % and velocity of the previous state plus any compensation that needs 
        % to be made due to acceleration.
        xhat = F * xhat + B * u;
        %%%%TESTING PURPOSES%%%%
        TotalPosxErrorExt = TotalPosxErrorExt + abs(x(1)-xhat(1));
        TotalPosyErrorExt = TotalPosyErrorExt + abs(x(2)-xhat(2));
        TotalPoszErrorExt = TotalPoszErrorExt + abs(x(3)-xhat(3));
        TotalVelxErrorExt = TotalVelxErrorExt + abs(x(4)-xhat(4));
        TotalVelyErrorExt = TotalVelyErrorExt + abs(x(5)-xhat(5));
        TotalVelzErrorExt = TotalVelzErrorExt + abs(x(6)-xhat(6));
        TotalRollErrorExt = TotalRollErrorExt + abs(x(7)-xhat(7));
        TotalPitchErrorExt = TotalPitchErrorExt + abs(x(8)-xhat(8));
        TotalYawErrorExt = TotalYawErrorExt + abs(x(9)-xhat(9));
        %%%%TESTING PURPOSES%%%%
        % Form the Innovation vector;
        % measured position - extrapolated position, describes the measurement 
        % residual.
        Inn = z - H * xhat;
        % Compute the E of the Innovation; 
        % Epp + measurement error E.
        InnE = H * P * H' + R;    
        % Form the Kalman Gain matrix;
        % this represents the proportional confidence we have in our 
        % measurements vs our extrapolated prediction.
        K = (((F * P * F') + Qu) * H') / InnE;
        % Update the state estimate;
        % the estimate plus the confidence weighted innovation.
        xhat = xhat + K * Inn;
        % Correct roll, pitch and yaw to be mod 360
        xroll = mod(xhat(7),360);
        xpitch = mod(xhat(8),360);
        xyaw = mod(xhat(9),360);
        % Compute the E of the estimation error for next iteration.
        P = ((F * P * F') + Qu) - (K * H * (F * P * F' + Qu));
        % Save some parameters for plotting later (must be hand altered upon change of sensor input).
        pos = [pos; x(1) x(2) x(3)];
        posmeas = [posmeas; z(1) z(2) z(3)];
        poshat = [poshat; xhat(1) xhat(2) xhat(3)];
        vel = [vel; x(4) x(5) x(6)];
        velmeas = [velmeas; z(4) z(5) z(6)];
        velhat = [velhat; xhat(4) xhat(5) xhat(6)];
        %%%%TESTING PURPOSES%%%%
        TotalPosxErrorMeas = TotalPosxErrorMeas + abs(x(1)-z(1)) + abs(x(1)-z(2)) + abs(x(1)-z(7)) + abs(x(1)-z(8)) + abs(x(1)-z(13));
        TotalPosyErrorMeas = TotalPosyErrorMeas + abs(x(2)-z(3)) + abs(x(2)-z(4)) + abs(x(2)-z(9)) + abs(x(2)-z(10));
        TotalPoszErrorMeas = TotalPoszErrorMeas + abs(x(3)-z(5)) + abs(x(3)-z(6)) + abs(x(3)-z(11)) + abs(x(3)-z(12)) + abs(x(3)-z(14)) + abs(x(3)-z(18));
        TotalVelxErrorMeas = TotalVelxErrorMeas + abs(x(4)-z(15));
        TotalVelyErrorMeas = TotalVelyErrorMeas + abs(x(5)-z(16));
        TotalVelzErrorMeas = TotalVelzErrorMeas + abs(x(6)-z(17));
        TotalRollErrorMeas = TotalRollErrorMeas + abs(x(7)-z(19)) + abs(x(7)-z(22));
        TotalPitchErrorMeas = TotalPitchErrorMeas + abs(x(8)-z(20)) + abs(x(8)-z(23));
        TotalYawErrorMeas = TotalYawErrorMeas + abs(x(9)-z(21)) + abs(x(9)-z(24));
        TotalPosxErrorFil = TotalPosxErrorFil + abs(x(1)-xhat(1));
        TotalPosyErrorFil = TotalPosyErrorFil + abs(x(2)-xhat(2));
        TotalPoszErrorFil = TotalPoszErrorFil + abs(x(3)-xhat(3));
        TotalVelxErrorFil = TotalVelxErrorFil + abs(x(4)-xhat(4));
        TotalVelyErrorFil = TotalVelyErrorFil + abs(x(5)-xhat(5));
        TotalVelzErrorFil = TotalVelzErrorFil + abs(x(6)-xhat(6));
        TotalRollErrorFil = TotalRollErrorFil + abs(x(7)-xhat(7));
        TotalPitchErrorFil = TotalPitchErrorFil + abs(x(8)-xhat(8));
        TotalYawErrorFil = TotalYawErrorFil + abs(x(9)-xhat(9));
        for i2=1:numel(z);                                          %sensor errors added together weighted by sensor noise
            for i3=1:numel(x);
                TotalSensoravgError = TotalSensoravgError + abs( (x(i3)-z(i2) * 1/sensornoise(i2)) * (H(i2) * x(i3)) );
            end
        end
        %%%%TESTING PURPOSES%%%%
    end

    end

    %%%%TESTING PURPOSES$$$$
    % Calculate average position and velocity errors
    AvgPosxErrorExt = TotalPosxErrorExt / ((duration/dt)*i);
    AvgPosyErrorExt = TotalPosyErrorExt / ((duration/dt)*i);
    AvgPoszErrorExt = TotalPoszErrorExt / ((duration/dt)*i);
    AvgVelxErrorExt = TotalVelxErrorExt / ((duration/dt)*i);
    AvgVelyErrorExt = TotalVelyErrorExt / ((duration/dt)*i);
    AvgVelzErrorExt = TotalVelzErrorExt / ((duration/dt)*i);
    AvgRollErrorExt = TotalRollErrorExt / ((duration/dt)*i);
    AvgPitchErrorExt = TotalPitchErrorExt / ((duration/dt)*i);
    AvgYawErrorExt = TotalYawErrorExt / ((duration/dt)*i);
    AvgPosxErrorMeas = (TotalPosxErrorMeas/5) / ((duration/dt)*i);
    AvgPosyErrorMeas = (TotalPosyErrorMeas/4) / ((duration/dt)*i);
    AvgPoszErrorMeas = (TotalPoszErrorMeas/6) / ((duration/dt)*i);
    AvgVelxErrorMeas = (TotalVelxErrorMeas) / ((duration/dt)*i);
    AvgVelyErrorMeas = (TotalVelyErrorMeas) / ((duration/dt)*i);
    AvgVelzErrorMeas = (TotalVelzErrorMeas) / ((duration/dt)*i);
    AvgRollErrorMeas = (TotalRollErrorMeas/2) / ((duration/dt)*i);
    AvgPitchErrorMeas = (TotalPitchErrorMeas/2) / ((duration/dt)*i);
    AvgYawErrorMeas = (TotalYawErrorMeas/2) / ((duration/dt)*i);
    AvgPosxErrorFil = TotalPosxErrorFil / ((duration/dt)*i);
    AvgPosyErrorFil = TotalPosyErrorFil / ((duration/dt)*i);
    AvgPoszErrorFil = TotalPoszErrorFil / ((duration/dt)*i);
    AvgVelxErrorFil = TotalVelxErrorFil / ((duration/dt)*i);
    AvgVelyErrorFil = TotalVelyErrorFil / ((duration/dt)*i);
    AvgVelzErrorFil = TotalVelzErrorFil / ((duration/dt)*i);
    AvgRollErrorFil = TotalRollErrorFil / ((duration/dt)*i);
    AvgPitchErrorFil = TotalPitchErrorFil / ((duration/dt)*i);
    AvgYawErrorFil = TotalYawErrorFil / ((duration/dt)*i);
    AvgSensoravgError = (TotalSensoravgError*(sum(1/sensornoise))) / ((duration/dt)*i*nnz(H));
    
    figure('name','Position Comparison');    
    analysis = [AvgPosxErrorMeas AvgPosxErrorExt AvgPosxErrorFil; AvgPosyErrorMeas AvgPosyErrorExt AvgPosyErrorFil; AvgPoszErrorMeas AvgPoszErrorExt AvgPoszErrorFil];
    bar(analysis);
    ylabel('Average Error (cm)');
    labels = {'x','y','z'};
    set(gca,'xticklabel',labels)
    title('Average Position Error (Measured, Extrapolated, and Filter Estimated)');

    figure('name','Velocity Comparison');
    analysis = [AvgVelxErrorMeas AvgVelxErrorExt AvgVelxErrorFil; AvgVelyErrorMeas AvgVelyErrorExt AvgVelyErrorFil; AvgVelzErrorMeas AvgVelzErrorExt AvgVelzErrorFil];
    bar(analysis);
    ylabel('Average Error (cm/s)');
    labels = {'x','y','z'};
    set(gca,'xticklabel',labels)
    title('Average Velocity Error (Measured, Extrapolated, and Filter Estimated)');
    
    figure('name','Orientation Comparison');
    analysis = [AvgRollErrorMeas AvgRollErrorExt AvgRollErrorFil; AvgPitchErrorMeas AvgPitchErrorExt AvgPitchErrorFil; AvgYawErrorMeas AvgYawErrorExt AvgYawErrorFil];
    bar(analysis);
    ylabel('Average Error (o)');
    labels = {'Roll','Pitch','Yaw'};
    set(gca,'xticklabel',labels)
    title('Average Orientation Error (Measured, Extrapolated, and Filter Estimated)');
    
    %%%%TESTING PURPOSES%%%%

    % RESULT PLOTTING
    t = 0 : dt : duration;

    % Position Comparison
    figure('name', 'Position Comparison')

    subplot(2,2,1);
    plot(t,pos(:,1), t,poshat(:,1));
    grid;
    xlabel('Time (sec)');
    ylabel('Position (cm)');
    title('Figure 1 - Vehicle X Position (True and Estimated)')

    subplot(2,2,2);
    plot(t,pos(:,2), t,poshat(:,2));
    grid;
    xlabel('Time (sec)');
    ylabel('Position (cm)');
    title('Figure 2 - Vehicle Y Position (True and Estimated)')

    subplot(2,2,3);
    plot(t,pos(:,3), t,posmeas(), t,poshat(:,3));
    grid;
    xlabel('Time (sec)');
    ylabel('Position (cm)');
    title('Figure 3 - Vehicle Z Position (True, Measured, and Estimated)')

    % Position Error
    figure('name', 'Position Error')

    subplot(2,2,1);
    plot(t,pos(:,1)-posmeas(:,1), t,pos(:,1)-poshat(:,1));
    grid;
    xlabel('Time (sec)');
    ylabel('Position Error (cm)');
    title('Figure 1 - X Position Measurement Error and Position Estimation Error');

    subplot(2,2,2);
    plot(t,pos(:,2)-posmeas(:,2), t,pos(:,2)-poshat(:,2));
    grid;
    xlabel('Time (sec)');
    ylabel('Position Error (cm)');
    title('Figure 2 - Y Position Measurement Error and Position Estimation Error');

    subplot(2,2,3);
    plot(t,pos(:,3)-posmeas(:,3), t,pos(:,3)-poshat(:,3));
    grid;
    xlabel('Time (sec)');
    ylabel('Position Error (cm)');
    title('Figure 3 - Z Position Measurement Error and Position Estimation Error');

    % Velocity Comparison
    figure('name','Velocity Comparison')

    subplot(2,2,1);
    plot(t,vel(:,1), t,velmeas(:,1), t,velhat(:,1));
    grid;
    xlabel('Time (sec)');
    ylabel('Velocity (cm/sec)');
    title('Figure 1 - X Velocity (True, Measured, and Estimated)');

    subplot(2,2,2);
    plot(t,vel(:,2), t,velmeas(:,2), t,velhat(:,2));
    grid;
    xlabel('Time (sec)');
    ylabel('Velocity (cm/sec)');
    title('Figure 2 - Y Velocity (True, Measured, and Estimated)');

    subplot(2,2,3);
    plot(t,vel(:,3), t,velmeas(:,3), t,velhat(:,3));
    grid;
    xlabel('Time (sec)');
    ylabel('Velocity (cm/sec)');
    title('Figure 3 - Z Velocity (True, Measured, and Estimated)');

    % Velocity Error
    figure('name', 'Velocity Error')

    subplot(2,2,1);
    plot(t,vel(:,1)-velmeas(:,1), t,vel(:,1)-velhat(:,1));
    grid;
    xlabel('Time (sec)');
    ylabel('Velocity Error (cm/sec)');
    title('Figure 1 - X Velocity Measurement Error and Velocity Estimation Error');

    subplot(2,2,2);
    plot(t,vel(:,2)-velmeas(:,2), t,vel(:,2)-velhat(:,2));
    grid;
    xlabel('Time (sec)');
    ylabel('Velocity Error (cm/sec)');
    title('Figure 2 - Y Velocity Measurement Error and Velocity Estimation Error');

    subplot(2,2,3);
    plot(t,vel(:,3)-velmeas(:,3), t,vel(:,3)-velhat(:,3));
    grid;
    xlabel('Time (sec)');
    ylabel('Velocity Error (cm/sec)');
    title('Figure 3 - Z Velocity Measurement Error and Velocity Estimation Error');

    % 3D Position Over Time
    figure('name', '3D Position Over Time')
    scatter3(pos(:,1),pos(:,2),pos(:,3),10,t,'filled');
    hold on;
    scatter3(poshat(:,1),poshat(:,2),poshat(:,3),20,t);
    ax = gca;
    ax.XDir = 'reverse';
    xlabel('x-position (cm)')
    ylabel('y-position (cm)')
    zlabel('z-position (cm)')
    colormap parula;
    cb = colorbar;
    cb.Label.String = 'Time (s)';
    rotate3d on;

    % 3D Position w/ Velocity
    figure('name', '3D Position with Velocity')
    scatter3(pos(:,1),pos(:,2),pos(:,3),10,sqrt((vel(:,1).^2)+(vel(:,2).^2)+(vel(:,3).^2)),'filled');
    hold on;
    scatter3(poshat(:,1),poshat(:,2),poshat(:,3),20,sqrt((velhat(:,1).^2)+(velhat(:,2).^2)+(velhat(:,3).^2)));
    ax = gca;
    ax.XDir = 'reverse';
    xlabel('x-position (cm)')
    ylabel('y-position (cm)')
    zlabel('z-position (cm)')
    colormap parula;
    cb = colorbar;
    cb.Label.String = 'Velocity (cm/s)';
    rotate3d on;