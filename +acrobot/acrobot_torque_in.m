classdef acrobot_torque_in < matlab.System

    % Public, tunable properties
    properties

    end

    properties(DiscreteState)

    end

    % Pre-computed constants
    properties(Access = private)

    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end

        function tau = stepImpl(obj, pwm, tau_in)
            tau = 0; % Translate pwm to tau using resistance
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
    end
end
