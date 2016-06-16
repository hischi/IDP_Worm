clear all;

createFit
muscle_func = @(u)feval(muscle_func_fit,u);
clear F ft h muscle_func_fit opts p xData yData zData

load('RobotConstants');