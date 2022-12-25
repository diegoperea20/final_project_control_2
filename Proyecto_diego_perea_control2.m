%% Proyecto Control De Sistemas No lineales , Nonlinear Systems Control Project
%Autor:Diego Iván Perea Montealegre 
%Universidad:Universidad Autonoma De Occidente  
%-----------------------------------------------------------------------------------
%Nonlinear Systems Control Project
%Author:Diego Iván Perea Montealegre
% University: Universidad Autonoma De Occidente 
%% Limpiar codigos anterior que pueden interferir en el codigo ,Clean previous codes that can interfere with the code.
clc 
clear all

%% Definición de constantes del sistema , Definition of system constants
A=10;
C=5;

%% Puntos de equilibrio ,Balance points
syms x1 x2 x3 u
%Con u=1 , with u=1
x1punto = (-C/A)*x1^0.5 + (1/A);
x2punto = (-C/A)*x2^0.5 + (C/A)*x1^0.5; 
x3punto = (-C/A)*x3^0.5 + (C/A)*x2^0.5;
%S = solve(eqn,var) solves the equation eqn for the variable var.
%If you do not specify var, the symvar function determines the variable to solve for. For example, solve(x + 1 == 2, x) solves the equation x + 1 = 2 for x
x1e = solve(x1punto==0,x1);%S = solve(eqn,var)
x2e = solve(x2punto==0,x2);%S = solve(eqn,var)
x3e = solve(x3punto==0,x3);%S = solve(eqn,var)

x1e,x2e,x3e

%% Linealizacion del sistema no lineal,Linearization of the nonlinear system
syms x1 x2 x3 u
aj=jacobian([(-C/A)*x1^0.5 + (1/A)*u;(-C/A)*x2^0.5 + (C/A)*x1^0.5;(-C/A)*x3^0.5 + (C/A)*x2^0.5],[x1,x2,x3]);
bj=jacobian([(-C/A)*x1^0.5 + (1/A)*u;(-C/A)*x2^0.5 + (C/A)*x1^0.5;(-C/A)*x3^0.5 + (C/A)*x2^0.5],[u]);

x1 = 1/25; %Valor de x1e ,Value of  x1e
x2 = 1/25; %Valor x2e,Value  of  x1e
x3 = 1/25; %Valor de  x3e,Value of x1e

a=eval(aj);
b=bj;
b = [1/10;0;0];
c=[0,0,1];
d=0;

a,b,c,d

%% Contrabilidad y observabilidad del sistema ,Controllability and observability of the system
 co=ctrb(a,b)
 rangoco=rank(co);
 if rangoco == length(a)
     disp ('el sistema si es controlable')
 else
     disp('el sistema no es controlable')
 end
 
  ob=obsv(a,c)
 rangoob=rank(ob);
 if rangoco == length(a)
     disp ('el sistema si es observable')
 else
     disp('el sistema no es observable')
 end
 
%% Regulador  en  tiempo continuo ,Continuous time regulator
pd = [-10,-11,-12];
k = place(a,b,pd);

%% Seguidor en tiempo continuo ,Continuous time tracker
aa=[a,zeros(3,1);-c,0];
ba=[b;0];

pda=[pd,-9];

kt=place(aa,ba,pda);
kp=kt(1:1,1:3);
ki=kt(1:1,4:4);

%% Observador,Observer
po=[-20,-21,-22]
h=place(a',c',po);
h=h'

%% Discretizacion,Discretization
tm=0.1; % sampling time ,tiempo de muestreo
[adis,bdis,cdis,ddis]=c2dm(a,b,c,d,tm,'zoh');

polo1disdes=exp(pd(1)*tm);
polo2disdes=exp(pd(2)*tm);
polo3disdes=exp(pd(3)*tm);
PoloAdisdes = exp(pda(4)*tm);

polosdiscredes=[polo1disdes,polo2disdes,polo3disdes];
polosdiscredesaum=[polosdiscredes,PoloAdisdes];


%% Regulador en tiempo discreto ,Discrete-time regulator tracker
kdis = place(adis,bdis,polosdiscredes);

%% Seguidor en tiempo discreto ,Discrete-time tracker
%aadis=[adis,bdis;zeros(1,3),0];
%badis=[zeros(3,1);1]
aadis=[adis,zeros(3,1);-cdis,1]
badis=[bdis;0]
ktdis=place(aadis,badis,polosdiscredesaum);
kpdis=ktdis(:,1:3);
kidis=ktdis(:,4:4);

%% Observador en tiempo discreto,Discrete time observer
podis=[exp(-20*tm),exp(-21*tm),exp(-22*tm)];
hdis=place(adis',cdis',podis);
hdis=hdis';

%time of retarded =1;


simulation_diego_perea

