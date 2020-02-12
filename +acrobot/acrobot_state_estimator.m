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
        imuf = imufilter('SampleRate',100);
        leg_length = 0.4;
        prev_dist_to_floor = 0.0;
        state= [0;0;0;0];
        imu_ground = true;
        entered = false;
        collision_timeout = 0;
        max_velocity_change = 10;
        fall_threshold = 0.05;
        
    end

    methods
        % Constructor
        function obj = acrobot_state_estimator(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
        end
    end
    
    methods(Access = public)
        function [state, on_ground] = stepImplPublic(obj, gyro, acc, motor_step)
                        
            % Yaw
            [orient, ~] = obj.imuf(acc', gyro');
            zyx = deg2rad(eulerd(orient,'ZYX','frame'));
            yaw = zyx(2);
            
            % Motor Angle
            qm = pi + motor_step / (obj.steps_per_rotation/ 2 * pi);

            if(obj.imu_ground)
                % imu leg on the ground
                q1 = yaw + pi/2;
                q2 = qm - pi;
            else
                % Calculate phi
                if zyx(1) > pi/2 || zyx(1) < -pi/2
                    phi = -yaw;
                else
                    if (yaw < 0)
                        phi = pi + yaw;
                    else
                        phi = -pi + yaw;
                    end
                end
                
                q2 = -(qm - pi);
                q1 = pi/2 + phi - q2;
            end
            
            q1_dot = (q1 - obj.state(1))/obj.sample_time;
            q2_dot = (q2 - obj.state(2))/obj.sample_time;

            if(abs(q1_dot) > obj.max_velocity_change)
                q1_dot = obj.state(3);
            end
            if(abs(q2_dot) > obj.max_velocity_change)
                q2_dot = obj.state(4);
            end
            
            % Collision Checking
            if (obj.entered)
                obj.collision_timeout = obj.collision_timeout + 1;
                if obj.collision_timeout == 10
                    obj.entered = ~obj.entered;
                end
            else
                obj.collision_timeout = 0;
            end
            rH = obj.leg_length * [cos(q1); sin(q1)];
            rc2 = rH + obj.leg_length * [cos(q1+q2); sin(q1+q2)];
            dist_to_floor = rc2(2);
            delta_dist = dist_to_floor - obj.prev_dist_to_floor;
            obj.prev_dist_to_floor = dist_to_floor;
            if  abs(dist_to_floor) < obj.fall_threshold && delta_dist<0
                if (~obj.entered)
                    obj.imu_ground = ~obj.imu_ground;
                    obj.entered = true;
                end
            end
            
            state = [q1;q2;q1_dot;q2_dot];
            on_ground = obj.imu_ground;
            obj.state = state;
        end
    end
    
    methods(Access = protected)
        %% Common functions
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            

            obj.imuf = imufilter('SampleRate',100);
            obj.imuf.GyroscopeNoise          = 0.001;
            obj.imuf.AccelerometerNoise      = 0.01;
%             obj.imuf.GyroscopeDriftNoise     = 3.0462e-12;
%             obj.imuf.LinearAccelerationNoise = 0.001;
%             obj.imuf.InitialProcessNoise = 10*obj.imuf.InitialProcessNoise;
        end

        function [state, on_ground] = stepImpl(obj, gyro, acc, motor_step)
            [state, on_ground] = obj.stepImplPublic(gyro, acc, motor_step);
        end
        
        function [s1, s2] = getOutputSizeImpl(~)
            s1 = [4,1];
            s2 = [1,1];
        end
        
        function [d1, d2] = getOutputDataTypeImpl(~)
            d1 = 'double';
            d2 = 'boolean';
        end
        
        function [c1, c2] = isOutputComplexImpl(~)
            c1 = false;
            c2 = false;
        end
        
        function [c1, c2] = isOutputFixedSizeImpl(~)
            c1 = true;
            c2 = true;
        end
        
        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj,'Type','Discrete',...
              'SampleTime',obj.sample_time,'OffsetTime',0.0);
        end
    end 
end
