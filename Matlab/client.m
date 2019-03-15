function client()
%   provides a menu for accessing PIC32 motor control functions
%
%   client(port)
%
%   Input Arguments:
%       port - the name of the com port.  This should be the same as what
%               you use in screen or putty in quotes ' '
%
%   Example:
%       client('/dev/ttyUSB0') (Linux/Mac)
%       client('COM3') (PC)
%
%   For convenience, you may want to change this so that the port is hardcoded.
port = '/dev/ttyUSB0';
% Opening COM connection
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

fprintf('Opening port %s....\n',port);

% settings for opening the serial port. baud rate 230400, hardware flow control
% wait up to 120 seconds for data before timing out
mySerial = serial(port, 'BaudRate', 230400, 'FlowControl', 'hardware','Timeout',120); 
% opens serial connection
fopen(mySerial);
% closes serial port when function exits
clean = onCleanup(@()fclose(mySerial));                                 

has_quit = false;
% menu loop
while ~has_quit
    fprintf('PIC32 MOTOR DRIVER INTERFACE\n\n');
    % display the menu options; this list will grow
    fprintf('\ta: Read current sensor (ADC counts)\tb: Read current sensor (mA)\n');
    fprintf('\tc: Read encoder (counts)\t\td: Read encoder (deg)\n');
    fprintf('\te: Reset encoder\t\t\tf: Set PWM (-100 to 100)\n');
    fprintf('\tg: Set current gains\t\t\th: Get current gains\n');
    fprintf('\ti: Set position gains\t\t\tj: Get position gains\n');
    fprintf('\tk: Test current control\t\t\tl: Go to angle (deg)\n');
    fprintf('\tm: Load step trajectory\t\t\tn: Load cubic trajectory\n');
    fprintf('\to: Execute trajectory\t\t\tp: Unpower the motor\n');
    fprintf('\tq: Quit client\t\t\t\tr: Get mode\n');
    % read the user's choice
    selection = input('\nENTER COMMAND: ', 's');
     
    % send the command to the PIC32
    fprintf(mySerial,'%c\n',selection);
    
    % take the appropriate action
    switch selection
        case 'a'  % Motor current ADC counts
            counts = fscanf(mySerial, '%d');
            fprintf('The motor current is at %d ADC counts.\n\n', counts);
        
        case 'b'  % Motor current sensor mA
            counts = fscanf(mySerial, '%d');
            fprintf('The motor current is %d mA.\n\n', counts);
            
        case 'c' % Motor angle in counts
            counts = fscanf(mySerial, '%d');
            fprintf('The motor angle is %d counts.\n\n', counts);
        
        case 'd'  % Motor angle in degrees
            degrees = fscanf(mySerial, '%f');
            fprintf('The motor angle is %f degrees.\n\n', degrees);
        
        case 'e'    %Reset encoder counts  
            fprintf('The motor has been reset to 0 degrees.\n\n');
            
        case 'f'    %Set PWM (-100 to 100) 
            pwm_in = input('Please input a PWM value you would like, between -100 and 100:\n\n');
            while(pwm_in < -100 || pwm_in > 100)
                pwm_in = input('Please input a valid value, between -100 and 100:\n\n');
            end
            fprintf(mySerial, '%d\r\n',pwm_in);
            if (pwm_in < 0)
                fprintf("Motor PWM set to %d %% in CCW direction\n\n",abs(pwm_in));
            else
                fprintf("Motor PWM set to %d %% in CW direction\n\n",abs(pwm_in));
            end
        
        case 'g'    %Set current gains
            curr_Kp = input('Enter your desired Kp current gain [recommended: 1.8]: ');
            curr_Ki = input('Enter your desired Ki current gain [recommended: 0.003]: ');
            fprintf('Sending Kp = %f and Ki = %f to the current controller.\n\n',curr_Kp, curr_Ki);
            fprintf(mySerial, '%f %f\r\n',[curr_Kp,curr_Ki]); % send the gains            

        case 'h' %Get Current gains
            curr_gains = fscanf(mySerial, '%f %f');
            fprintf('The current controller is using Kp = %3.3f and Ki = %3.3f\n\n',[curr_gains(1), curr_gains(2)]);
         
        case 'i'    %Set position gains
            pos_Kp = input('Enter your desired Kp position gain [recommended: 75]: ');
            pos_Ki = input('Enter your desired Ki position gain [recommended: 0]: ');
            pos_Kd = input('Enter your desired Kd position gain [recommended: 5000]: ');
            fprintf('Sending Kp = %f, Ki = %f and Kd = %f to the position controller.\n\n',pos_Kp, pos_Ki, pos_Kd);
            fprintf(mySerial, '%f %f %f\r\n',[pos_Kp, pos_Ki, pos_Kd]); % send the gains            

        case 'j' %Get position gains
            pos_gains = fscanf(mySerial, '%f %f %f');
            fprintf('The position controller is using Kp = %3.3f, Ki = %3.3f and Kd = %3.3f\n\n',[pos_gains(1), pos_gains(2), pos_gains(3)]);
            
        case 'k' %Test Current control
            fprintf('The Motor is testing current control, Please wait ...');
            read_plot_matrix(mySerial);
        
        case 'l' %Go to angle (deg)
            deg_desired = input('Enter the desired motor angle in degrees: '); % get the desired angle
            fprintf('Motor moving to %d degrees.\n\n', deg_desired);
            fprintf(mySerial, '%d\n',deg_desired); % send the angle
            
        case 'm' %Load Step Trajectory
            A = input('Enter step trajectory, in sec and degrees [time1, ang1; time2, ang2; ...]: ');
            while A(end, 1)>10
                fprintf('Error: Maximum trajectory time is 10 seconds');
                A = input('Enter step trajectory, in sec and degrees [time1, ang1; time2, ang2; ...]: ');
            end
            step_refs = genRef(A, 'step');
            step_No_samps = size(step_refs,2);
            fprintf(mySerial, '%d\n',step_No_samps); %No. of samples
            for i = 1:step_No_samps
                fprintf(mySerial, '%d\n',step_refs(i)); %refs
            end
            
        case 'n' %Load Cubic Trajectory
             A = input('Enter step trajectory, in sec and degrees [time1, ang1; time2, ang2; ...]: ');
            while A(end, 1)>10
                fprintf('Error: Maximum trajectory time is 10 seconds');
                A = input('Enter step trajectory, in sec and degrees [time1, ang1; time2, ang2; ...]: ');
            end
            cub_refs = genRef(A, 'cubic');
            cub_No_samps = size(cub_refs,2);
            fprintf(mySerial, '%d\n',cub_No_samps); %No. of samples
            for i = 1:cub_No_samps
                fprintf(mySerial, '%f\n',cub_refs(i)); %refs
            end
            
        case 'o' %Execute Trajectory
            fprintf('Executing and tracking given trajectory\n\n');
            fprintf('Please wait for plot...\n\n');
            traj_plot(mySerial);
                       
        case 'p' %Unpower the moter
            fprintf('Motor has been unpowered (set to IDLE)\n\n');
            
        case 'q'
            has_quit = true;         % exit client
            
        case 'r'    %Returns current PIC mode
            mode = fscanf(mySerial, '%d');
            if mode == 1
                fprintf('The PIC32 controller is in IDLE mode\n\n');
            end
            if mode == 2
                fprintf('The PIC32 controller is in PWM mode\n\n');
            end
            if mode == 3
                fprintf('The PIC32 controller is in ITEST mode\n\n');
            end
            if mode == 4
                fprintf('The PIC32 controller is in HOLD mode\n\n');
            end
            if mode == 5
                fprintf('The PIC32 controller is in TRACK mode\n\n');
            end
          
        otherwise
            fprintf('Invalid Selection %c\n\n', selection);
    end
end

end
