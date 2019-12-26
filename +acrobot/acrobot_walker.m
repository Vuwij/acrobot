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
        end

        function tau = stepImpl(obj,state)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            tau = obj.getTau(state);
            tau = tau(2);
        end
        
        function s1 = getOutputSizeImpl(~)
            s1 = 1;
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