classdef motor
    properties
        torque;
        speed;
        weight;
    end
    
    methods
        function obj = motor(torque, speed, weight)
            obj.torque = torque;
            obj.speed = speed;
            obj.weight = weight;
        end
    end
end
    