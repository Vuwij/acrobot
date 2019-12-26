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
            tau = 0;
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
    end
end
