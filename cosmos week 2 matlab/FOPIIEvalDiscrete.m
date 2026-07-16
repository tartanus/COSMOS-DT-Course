function f= FOPIIEvalDiscrete(x)
     assignin('base','alpha',x(1))
     assignin('base','beta', x(2))
     assignin('base','ti1',  x(3))
     assignin('base','ti2', x(4))
     assignin('base','kp',  x(5))

    %fractional-order FOPII derivatives
    s1=fotf('s');
    alpha=x(1);        %fractional order single integrator
    beta=x(2);         %fractional order double integrator
    %substracting -1 to eliminate steady state error
    singleIntegrator= doustafod((-alpha+1),1,0.001,1000,1); 
    doubleIntegrator= doustafod((-beta),1,0.001,1000,1); 

    assignin('base','singleIntegrator', singleIntegrator)
    assignin('base','doubleIntegrator',  doubleIntegrator)


    sim('fopiiTuningDiscrete.slx')
    
    r=simout(:,1);
    y=simout(:,2);
    yd=simout(:,4);
    f= 1*sqrt(1/length(r) *sum((yd-y).^2));%+ (r(1)-max(y))^2   ;  %reference model cost function error tuning
%     f= sqrt(1/length(r) *sum((yd-y).^2));  %reference model cost function error tuning
%     f= sqrt(1/length(r) *sum((r-y).^2));
%     [alpha beta ti1 ti2 kp]
end

