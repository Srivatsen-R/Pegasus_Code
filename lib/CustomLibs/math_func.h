#ifndef _MATH_FUNC_H
#define _MATH_FUNC_H

//#define MINDIFF 2.2250738585072014e-308   // smallest positive double
#define MINDIFF 2.25e-308                   // use for convergence check

#define EULER_CONST 2.718281828459045235
#define TAYLOR_ITERATIONS 20

double sqroot(double square);

int ipow(int base, int exp);

double nat_log(double x);

#endif