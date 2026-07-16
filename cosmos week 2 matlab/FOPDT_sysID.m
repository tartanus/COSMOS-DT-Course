function [J] = FOPDT_sysID(x)
        
    load('/Users/jairoviola/Documents/MATLAB/DT_COSMOS/DCMotorIO.mat')
    
    PWM=BMInput(1:300,2);
    RPM=BMOut(1:300,2);
    ts=BMInput(2,1);
    time=BMInput(1:300,1);
    
    inputVector=[time,PWM];
        
    assignin('base','R',x(1))
    assignin('base','L', x(2))
    assignin('base','K',  x(3))
    assignin('base','J', x(4))
    assignin('base','b',  x(5))

   
    %simulate FOPDT system response
    sim('Motor_Pos.slx')

    %r=simout(:,1);
    ySim=simout(:,1);
    yReal=RPM;


    J= sqrt( (1/length(yReal))*sum(ySim(1:length(yReal))-yReal  ).^2  );

end

