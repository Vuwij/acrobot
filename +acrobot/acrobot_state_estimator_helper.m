classdef acrobot_state_estimator_helper < matlab.System
    % Public, tunable properties
    properties
        sample_time = 0.01;
    end

    % Public, non-tunable properties
    properties(Nontunable)

    end

    properties(DiscreteState)
        
        
    end

    % Pre-computed constants
    properties(Access = private)
        cycle = 0;
    end

    methods
        % Constructor
        function obj = acrobot_state_estimator_helper(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
        end
    end
    
    methods(Access = public)
        function setupImplPublic(obj)
            % Perform one-time calculations, such as computing constants
        end
    end
    
    methods(Access = protected)
        %% Common functions
        function setupImpl(obj)
            obj.setupImplPublic();
        end

        function [step, pos, acc] = stepImpl(obj, step_count, pos1, acc1, pos2, acc2)
                step = step_count;
            if mod(cast(step, 'int8'), 2) == 0
                pos = pos2;
                acc = acc2;
            else
                pos = pos1;
                acc = acc1;
            end
            obj.cycle = obj.cycle + 1;
        end
        
        function [s1, s2, s3] = getOutputSizeImpl(~)
            s1 = 1;
            s2 = [4,1];
            s3 = [3,1];
        end
        
        function [d1, d2, d3] = getOutputDataTypeImpl(~)
            d1 = 'double';
            d2 = 'double';
            d3 = 'double';
        end
        
        function [c1, c2, c3] = isOutputComplexImpl(~)
            c1 = false;
            c2 = false;
            c3 = false;
        end
        
        function [c1, c2, c3] = isOutputFixedSizeImpl(~)
            c1 = true;
            c2 = true;
            c3 = true;
        end
        
        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj,'Type','Discrete',...
              'SampleTime',obj.sample_time,'OffsetTime',0.0);
        end
    end 
end

