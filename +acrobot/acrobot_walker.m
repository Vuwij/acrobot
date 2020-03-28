classdef acrobot_walker < acrobot.acrobot_control & matlab.System

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

    end

    methods
        % Constructor
        function obj = acrobot_walker(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
        end
    end

    methods(Access = protected)
        %% Common functions
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.tau_q = [0;0];
            obj.tau = [0;0];
        end

        function [step_count, tau] = stepImpl(obj, state, collision)
            if (collision)
                obj.step_count = obj.step_count + 1;
            end
%             obj.x = state;
%             obj.show(0);
            tau = 0;
            step_count = obj.step_count;
        end
        
        function [s1, s2] = getOutputSizeImpl(~)
            s1 = 1;
            s2 = 1;
        end
        
        function [d1,d2] = getOutputDataTypeImpl(~)
            d1 = 'double';
            d2 = 'double';
        end
        
        function [c1,c2] = isOutputComplexImpl(~)
            c1 = false;
            c2 = false;
        end
        
        function [c1,c2] = isOutputFixedSizeImpl(~)
            c1 = true;
            c2 = true;
        end
    end
end
