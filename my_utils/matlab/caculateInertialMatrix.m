
%%

clc;
clear;

syms alpha beta gamma u0 v0
A = [alpha gamma u0; 0 beta v0; 0 0 1];
Ainv = inv(A);
B = Ainv'*Ainv;

%%
syms B0 B1 B2 B3 B4 B5
B = [B0 B1 B3;
     B1 B2 B4;
     B3 B4 B5];
% R = chol(B);

%%
syms a1 a2 a3 a4 b1 b2 b3 b4

A = [a1 a2 a3 a4;b1 b2 b3 b4];

B = A' * A;

A1 = [a1 a2 a3 a4];
A2 = [b1 b2 b3 b4];

B1 = A1'* A1;
B2 = A2'* A2;

B3 = B1+B2;






