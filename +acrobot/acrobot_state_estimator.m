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
        end

        function state = stepImpl(obj, gyro, acc)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            state = [0; 0; 0; 0];
        end

    end
end
