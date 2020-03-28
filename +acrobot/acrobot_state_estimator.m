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
        leg_length = 0.335;
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
        end
        
        function [state, collision] = stepImplPublic(obj, step, pos, acc, motor_step)
            % Motor Angle
            qm = pi - motor_step / (obj.steps_per_rotation/ (2 * pi));
            
            % gyro1 and acc1 are always on the ground
            if (mod(step, 2) == 1)
                qm = 2*pi - qm;
            end
            
            % q1 & q1_dot
            roll = pos(2);
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
            if dist_to_floor < 0
                collision = 1;
            else
                collision = 0;
            end
   
            state = [q1;q2;q1_dot;q2_dot];
            obj.state = state;
        end
    end
    
    methods(Access = protected)
        %% Common functions
        function setupImpl(obj)
            obj.setupImplPublic();
        end

        function [state, collision] = stepImpl(obj, step, pos, acc, motor_step)
            [state, collision] = obj.stepImplPublic(step, pos, acc, motor_step);
        end
        
        function [s1, s2] = getOutputSizeImpl(~)
            s1 = [4,1];
            s2 = [1,1];
        end
        
        function [d1, d2] = getOutputDataTypeImpl(~)
            d1 = 'double';
            d2 = 'double';
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

