classdef acrobot_state_estimator < matlab.System
    % Public, tunable properties
    properties

    end

    % Public, non-tunable properties
    properties(Nontunable)

    end

    properties(DiscreteState)

    end

    % Pre-computed constants
    properties(Access = private)
        imuf = imufilter('SampleRate',100);
        prev_time = 0;
        prev_q2 = 0;
        pitch = 0;
        roll = 0;
        yaw = 0;
    end

    methods
        % Constructor
        function obj = acrobot_state_estimator(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
        end
    end

    methods(Access = protected)
        %% Common functions
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            
            obj.imuf.GyroscopeNoise          = 7.6154e-7;
            obj.imuf.AccelerometerNoise      = 0.0015398;
            obj.imuf.GyroscopeDriftNoise     = 3.0462e-12;
            obj.imuf.LinearAccelerationNoise = 0.00096236;
            onj.imuf.InitialProcessNoise = 10*obj.imuf.InitialProcessNoise;
            
            tic
        end

        function state = stepImpl(obj, gyro, acc, motor_step)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
%             fprintf('a(1): %0.3f, a(2): %0.3f, a(3):%0.3f\n', a(1), a(2), a(3));
             [orient, angVelocity] = obj.imuf(acc', gyro');
%              % Find angles from accelerometer
%             tau = 0.98;
%             accelPitch = rad2deg(atan2(a(2), a(3)));
%             accelRoll = rad2deg(atan2(a(1), a(3)));
%             
%             % Calculate dt
            dt = toc - obj.prev_time;
            obj.prev_time = toc;
% 
%             % Apply complementary filter
%             obj.pitch = (tau)*(obj.pitch + g(1) * dt) + (1 - tau)*(accelPitch);
%             obj.roll = (tau)*(obj.roll - g(2) * dt) + (1 - tau)*(accelRoll);
%             obj.yaw = (obj.yaw + g(3) * dt);

            % Print results
%             fprintf('Pitch: %0.3f,', obj.pitch)
%             rpy = eulerd(q);
            zyx = euler(orient,'ZYX','frame');
%             rpy = quat2eul(q);
%             rpy = rad2deg(rpy);
            
            q1 = zyx(2);
            q1_dot = -1*angVelocity(2);
            steps_per_rotation = 2797;
            q2 = motor_step / (steps_per_rotation/2*pi);
            obj.prev_q2 = q2;
            q2_dot = (q2 - obj.prev_q2)/dt;
            state = [q1;q2 ; q1_dot;q2_dot];
        end
        
        function s1 = getOutputSizeImpl(~)
            s1 = [4,1];
        end
        
        function d1 = getOutputDataTypeImpl(~)
            d1 = 'double';
        end
        
        function c1 = isOutputComplexImpl(~)
            c1 = false;
        end
        
        function c1 = isOutputFixedSizeImpl(~)
            c1 = true;
        end
    end
end
