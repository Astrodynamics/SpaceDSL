﻿#include <iostream>
#include <vector>
#include <math.h>

#include <nlopt.hpp>

using namespace std;

typedef struct {
    double a, b;
} my_constraint_data;

double myfunc(unsigned n, const double *x, double *grad, void *my_func_data)
{
    if (grad) {
        grad[0] = 0.0;
        grad[1] = 0.5 / sqrt(x[1]);
    }
    return sqrt(x[1]);
}

double myconstraint(unsigned n, const double *x, double *grad, void *data)
{
    my_constraint_data *d = (my_constraint_data *) data;
    double a = d->a, b = d->b;
    if (grad) {
        grad[0] = 3 * a * (a*x[0] + b) * (a*x[0] + b);
        grad[1] = -1.0;
    }
    return ((a*x[0] + b) * (a*x[0] + b) * (a*x[0] + b) - x[1]);
}

int main ()
{
    NLOpt::NLOptimize opt(NLOpt::LD_SLSQP, 2);
    std::vector<double> lb(2);
    lb[0] = -HUGE_VAL; lb[1] = 0;
    opt.SetLowerBounds(lb);
    opt.SetMinObjective(myfunc, NULL);
    my_constraint_data data[2] = { {2,0}, {-1,1} };
    opt.AddInequalityConstraint(myconstraint, &data[0], 1e-8);
    opt.AddInequalityConstraint(myconstraint, &data[1], 1e-8);
    opt.SetXTolRel(1e-4);
    std::vector<double> x(2);
    x[0] = 1.234; x[1] = 5.678;
    double minf;

    try
    {
        NLOpt::ResultType result = opt.OptimizeStart(x, minf);
        cout.precision(10);
        cout << "found minimum at f(" << x[0] << "," << x[1] << ") = "<< minf << endl;
    }
    catch(exception &e) {
        cout << "nlopt failed: " << e.what() << endl;
    }
}
