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
        imu_default = true
        imuf = imufilter('SampleRate',100, 'ReferenceFrame', 'ENU');
        leg_length = 0.335;
        prev_dist_to_floor = 0.0;
        state = [0;0;0;0];
        cycleCount = 0;
        timeout = false;
        max_velocity_change = 10;
        
    end

    methods
        % Constructor
        function obj = acrobot_state_estimator(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
        end
    end
    
    methods(Access = public)
        function setupImplPublic(obj)
            % Perform one-time calculations, such as computing constants
            
            obj.imuf = imufilter('SampleRate',1/obj.sample_time, 'ReferenceFrame', 'ENU');
            obj.imuf.GyroscopeNoise          = 0.001;
            obj.imuf.AccelerometerNoise      = 0.01;
        end
        
        function [state, imu_default] = stepImplPublic(obj, pos1, gyro1, acc1, pos2, gyro2, acc2, motor_step)
            if (obj.timeout)        
                obj.cycleCount = obj.cycleCount + 1;
                if obj.cycleCount == 10
                    obj.timeout = ~obj.timeout;
                end
            else
                obj.cycleCount = 0;
            end
            % Motor Angle
            qm = pi - motor_step / (obj.steps_per_rotation/ (2 * pi));
            
            % gyro1 and acc1 are always on the ground
            if (~obj.imu_default)
                % Swap gyro1 and gyro2
                t_gyro1 = gyro1;
                gyro1 = gyro2;
                gyro2 = t_gyro1;
                
                t_acc1 = acc1;
                acc1 = acc2;
                acc2 = t_acc1;
                
                t_pos1 = pos1;
                pos1 = pos2;
                pos2 = t_pos1;
                
                qm = 2*pi - qm;
            end
            % Shock dection here
            % Threshold 
            xThreshold = 0.01;
            yThreshold = 0.02;
            zThreshold = 0.03;
            
            if(~obj.timeout && abs(acc2(1)) > xThreshold && abs(acc2(2)) > yThreshold && abs(acc2(3)) > zThreshold)
                obj.imu_default = ~obj.imu_default;
                obj.timeout = ~obj.timeout;
            end
            
            % q1 & q1_dot
            [yaw, pitch, roll] = quat2angle(pos1');
            q1 = roll + pi/2;
            q1_dot = (q1 - obj.state(1))/obj.sample_time;
            
            % q2 & q2_dot
            q2 = qm - pi;
            q2_dot = (q2 - obj.state(2))/obj.sample_time;
            
            if(abs(q1_dot) > obj.max_velocity_change)
                q1_dot = obj.state(3);
            end
            if(abs(q2_dot) > obj.max_velocity_change)
                q2_dot = obj.state(4);
            end
            
            rH = obj.leg_length * [cos(q1); sin(q1)];
            rc2 = rH + obj.leg_length * [cos(q1+q2); sin(q1+q2)];
            dist_to_floor = rc2(2);
            delta_dist = dist_to_floor - obj.prev_dist_to_floor;
            obj.prev_dist_to_floor = dist_to_floor;
            if ~obj.timeout && dist_to_floor < 0 && delta_dist < 0
                obj.timeout = ~obj.timeout;
                obj.imu_default = ~obj.imu_default;
            end
            % Yaw
%             [orient, ~] = obj.imuf.step(acc1', gyro1');
%             zyx = deg2rad(eulerd(orient,'ZYX','frame'));
%             yaw = wrapToPi(zyx(3));
%             
%             
% 
%             if(obj.imu_ground)
%                 q1 = yaw + pi/2;
%                 q2 = qm - pi;
%             else
%                 % Calculate phi
%                 if zyx(1) > pi/2 || zyx(1) < -pi/2
%                     phi = -yaw;
%                 else
%                     if (yaw < 0)
%                         phi = pi + yaw;
%                     else
%                         phi = -pi + yaw;
%                     end
%                 end
%                 
%                 q2 = -(qm - pi);
%                 q1 = pi/2 + phi - q2;
%             end
%             
%             
% 
%             if(abs(q1_dot) > obj.max_velocity_change)
%                 q1_dot = obj.state(3);
%             end
%             if(abs(q2_dot) > obj.max_velocity_change)
%                 q2_dot = obj.state(4);
%             end
%             
%             rH = obj.leg_length * [cos(q1); sin(q1)];
%             rc2 = rH + obj.leg_length * [cos(q1+q2); sin(q1+q2)];
%             dist_to_floor = rc2(2);
%             delta_dist = dist_to_floor - obj.prev_dist_to_floor;
%             obj.prev_dist_to_floor = dist_to_floor;
%             if dist_to_floor < 0 && delta_dist < 0
%                 obj.imu_ground = ~obj.imu_ground;
%             end
%             
            state = [q1;q2;0;0];
            imu_default = obj.imu_default;
            obj.state = state;
        end
    end
    
    methods(Access = protected)
        %% Common functions
        function setupImpl(obj)
            obj.setupImplPublic();
        end

        function [state, imu_default] = stepImpl(obj, pos1, gyro1, acc1, pos2, gyro2, acc2, motor_step)
            [state, imu_default] = obj.stepImplPublic(pos1, gyro1, acc1, pos2, gyro2, acc2, motor_step);
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

