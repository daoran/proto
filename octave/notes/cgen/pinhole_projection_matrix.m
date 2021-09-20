pkg load symbolic;

syms K11 K12 K13;
syms K21 K22 K23;
syms K31 K32 K33;

syms C11 C12 C13;
syms C21 C22 C23;
syms C31 C32 C33;

syms rx ry rz;

C = [C11, C12, C13;
     C21, C22, C23;
     C31, C32, C33];

r = [rx; ry; rz];

K = [K11, K12, K13;
     K21, K22, K23;
     K31, K32, K33];

P = K * [C, r];

ccode(P)
