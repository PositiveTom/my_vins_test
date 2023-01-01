clc;
clear;

syms x y k0 k1
r2 = x^2 + y^2;
r4 = r2^2;
f = x* (1 + k0*r2 + k1*r4  );
f2 = y*(1+k0*r2+k1*r4);