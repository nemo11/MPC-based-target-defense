function out1 = MyDtSystem2_f(t1,in2,u1)
%MYDTSYSTEM2_F
%    OUT1 = MYDTSYSTEM2_F(T1,IN2,U1)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    22-Sep-2017 13:38:52

x1 = in2(1,:);
x2 = in2(2,:);
out1 = [u1+t1.*x1;u1.*x2];