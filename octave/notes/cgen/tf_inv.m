pkg load symbolic;

syms C11 C12 C13;
syms C21 C22 C23;
syms C31 C32 C33;

syms rx ry rz;

C = [C11, C12, C13;
     C21, C22, C23;
     C31, C32, C33];

r = [rx; ry; rz];
