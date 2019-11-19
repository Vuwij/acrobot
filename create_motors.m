% Creates a list of motor objects
function [motors] = create_motors()

    % 1st column: max torque
    % 2nd col: max speed
    % 3rd col: weight
    motor_specs_list = [
        1.76, 251, 205;
        0.637, 380, 160;
        0.69, 216, 136;
        0.6, 500, 98;
        1.18, 350, 98;
        0.98, 210, 96;
        0.69, 150, 160
    ];

    motors = [];
    s = size(motor_specs_list);
    number_of_motors = s(1);
    for i=1:number_of_motors
        new_motor = motor(motor_specs_list(i,1), motor_specs_list(i,2), motor_specs_list(i,3));
        motors = [motors, new_motor];
    end
end