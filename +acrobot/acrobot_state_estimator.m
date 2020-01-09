classdef acrobot_state_estimator < matlab.System
    % Public, tunable properties
    properties
        steps_per_rotation = 2797;
        sample_time = 0.01;
    end

    % Public, non-tunable properties
    properties(Nontunable)

    end

    properties(DiscreteState)
      
        
    end

    % Pre-computed constants
    properties(Access = private)
        imuf;
        leg_length;
        prev_dist_to_floor = 0.0;
        state= [0;0;0;0];
        dir = true;
        entered = false;
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
            
%             obj.imuf.GyroscopeNoise          = 7.6154e-7;
%             obj.imuf.AccelerometerNoise      = 0.0015398;
%             obj.imuf.GyroscopeDriftNoise     = 3.0462e-12;
%             obj.imuf.LinearAccelerationNoise = 0.00096236;
%             obj.imuf.InitialProcessNoise = 10*obj.imuf.InitialProcessNoise;
              obj.imuf = imufilter('SampleRate',100);
              obj.leg_length = 0.4;
        end

        function state = stepImpl(obj, gyro, acc, motor_step)
            % yaw
            [orient, angVelocity] = obj.imuf(acc', gyro');
            zyx = eulerd(orient,'ZYX','frame');
            zyx = deg2rad(zyx);
            % motor angle
            qm = pi  + motor_step / (obj.steps_per_rotation/ 2* pi);
            
            if(obj.dir)
                % imu leg on the ground
                q1 = pi/2 + zyx(1);
                q1_dot = -1*angVelocity(2);
                q2 = qm - pi;
                q2_dot = (qm - obj.state(2))/obj.sample_time;
                fprintf("imu on ground\n");
            else
                % imu leg in the air
                
%                 q1 = (qm -zyx(1)) - pi;
                fprintf("imu in air\n");
                q1_dot = -1*angVelocity(2);
                q2 = pi - qm;
                q2_dot = (qm - obj.state(2))/obj.sample_time;
                if(qm > pi)
                    q1 = pi -(qm - zyx(1));
                else
                    q1 = -(qm - zyx(1));
                end
            end

            rH = obj.leg_length * [cos(q1); sin(q1)];
            rc2 = rH + obj.leg_length * [cos(q1+q2); sin(q1+q2)];
            dist_to_floor = rc2(2);
            delta_dist = dist_to_floor - obj.prev_dist_to_floor;
            obj.prev_dist_to_floor = dist_to_floor;
            fprintf('%.4f\n', delta_dist);  
            if  abs(dist_to_floor) < 0.01 && delta_dist<0
                if (~obj.entered)
                    fprintf("here");
                    obj.dir = ~obj.dir;
                    fprintf('%.4f', dist_to_floor);  
                    obj.entered = true;
                end
            end
            
            state = [q1;q2;q1_dot;qm];
            obj.state = state;
            return;
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
        
        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj,'Type','Discrete',...
              'SampleTime',obj.sample_time,'OffsetTime',0.0);
        end
    end
end
