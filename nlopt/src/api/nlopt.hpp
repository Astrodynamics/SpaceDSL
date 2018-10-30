/************************************************************************
* Copyright (C) 2018 Niu ZhiYong
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Author: Niu ZhiYong
* Date:2018-07-13
* Description:
*   nlopt.hpp
*
*   Purpose:
*
*       Define The Nonlinear Optimization Library(NLopt) C++ Style API
* This File is Reconstruction of Original Author ,Steven G. Johnson.
* The Purpose is to Conform to The Naming Style of SpaceDSL.
*
*
*   Last modified:
*
*   2018-07-13  Niu Zhiyong (1st edition)
*
*************************************************************************/

#ifndef NLOPT_HPP
#define NLOPT_HPP
#include <nlopt.h>
#include <vector>
#include <stdexcept>
#include <new>
#include <cstdlib>
#include <cstring>
#include <cmath>

// convenience overloading for below (not in nlopt:: since has nlopt_ prefix)
inline nlopt_result NLOptGetInitialStep(const nlopt_opt opt, double *dx)
{
      return nlopt_get_initial_step(opt, (const double *) NULL, dx);
}
namespace NLOpt
{
//////////////////////////////////////////////////////////////////////
// nlopt::* namespace versions of the C enumerated types
//          AUTOMATICALLY GENERATED, DO NOT EDIT
// GEN_ENUMS_HERE
    enum AlgorithmType
    {
        GN_DIRECT = 0,
        GN_DIRECT_L,
        GN_DIRECT_L_RAND,
        GN_DIRECT_NOSCAL,
        GN_DIRECT_L_NOSCAL,
        GN_DIRECT_L_RAND_NOSCAL,
        GN_ORIG_DIRECT,
        GN_ORIG_DIRECT_L,
        GD_STOGO,
        GD_STOGO_RAND,
        LD_LBFGS_NOCEDAL,
        LD_LBFGS,
        LN_PRAXIS,
        LD_VAR1,
        LD_VAR2,
        LD_TNEWTON,
        LD_TNEWTON_RESTART,
        LD_TNEWTON_PRECOND,
        LD_TNEWTON_PRECOND_RESTART,
        GN_CRS2_LM,
        GN_MLSL,
        GD_MLSL,
        GN_MLSL_LDS,
        GD_MLSL_LDS,
        LD_MMA,
        LN_COBYLA,
        LN_NEWUOA,
        LN_NEWUOA_BOUND,
        LN_NELDERMEAD,
        LN_SBPLX,
        LN_AUGLAG,
        LD_AUGLAG,
        LN_AUGLAG_EQ,
        LD_AUGLAG_EQ,
        LN_BOBYQA,
        GN_ISRES,
        AUGLAG,
        AUGLAG_EQ,
        G_MLSL,
        G_MLSL_LDS,
        LD_SLSQP,
        LD_CCSAQ,
        GN_ESCH,
        GN_AGS,
        NUM_ALGORITHMS        /* not an algorithm, just the number of them */
    };
    enum ResultType
    {
        FAILURE = -1, /* generic failure code */
        INVALID_ARGS = -2,
        OUT_OF_MEMORY = -3,
        ROUNDOFF_LIMITED = -4,
        FORCED_STOP = -5,
        SUCCESS = 1, /* generic success code */
        STOPVAL_REACHED = 2,
        FTOL_REACHED = 3,
        XTOL_REACHED = 4,
        MAXEVAL_REACHED = 5,
        MAXTIME_REACHED = 6
    };
    //////////////////////////////////////////////////////////////////////
    typedef nlopt_func FuncPointer;    // nlopt::func synoynm
    typedef nlopt_mfunc mFuncPointer;  // nlopt::mfunc synoynm
    // alternative to nlopt_func that takes std::vector<double>
    // ... unfortunately requires a data copy
    typedef double (*vFuncPointer)(const std::vector<double> &x,
                            std::vector<double> &grad, void *data);
    //////////////////////////////////////////////////////////////////////

    // NLopt-specific exceptions (corresponding to error codes):
    class RoundoffLimited : public std::runtime_error
    {
    public:
        RoundoffLimited() : std::runtime_error("nlopt roundoff-limited") {}
    };

    class ForcedStop : public std::runtime_error
    {
    public:
        ForcedStop() : std::runtime_error("nlopt forced stop") {}
    };
    //////////////////////////////////////////////////////////////////////
    class NLOptimize
    {
    public:
        // Constructors etc.
        NLOptimize() : o(NULL), xtmp(0), gradtmp(0), gradtmp0(0),
                last_result(NLOpt::FAILURE), last_optf(HUGE_VAL),
                forced_stop_reason(NLOPT_FORCED_STOP) {}
        ~NLOptimize()
        {
            nlopt_destroy(o);
        }

        NLOptimize(AlgorithmType a, unsigned n) :  o(nlopt_create(nlopt_algorithm(a), n)),
                                                    xtmp(0), gradtmp(0), gradtmp0(0),
                                                    last_result(NLOpt::FAILURE), last_optf(HUGE_VAL),
                                                    forced_stop_reason(NLOPT_FORCED_STOP)
        {
            if (!o)
                throw std::bad_alloc();
            nlopt_set_munge(o, FreeNLOptFuncData, DupNLOptFuncData);
        }

        NLOptimize(const NLOptimize& f) : o(nlopt_copy(f.o)),
                                        xtmp(f.xtmp), gradtmp(f.gradtmp), gradtmp0(0),
                                        last_result(f.last_result), last_optf(f.last_optf),
                                        forced_stop_reason(f.forced_stop_reason)
        {
            if (f.o && !o)
                throw std::bad_alloc();
        }

        NLOptimize& operator=(NLOptimize const& f)
        {
            if (this == &f)
                return *this; // self-assignment
            nlopt_destroy(o);
            o = nlopt_copy(f.o);
            if (f.o && !o)
                throw std::bad_alloc();
            xtmp = f.xtmp;
            gradtmp = f.gradtmp;
            last_result = f.last_result;
            last_optf = f.last_optf;
            forced_stop_reason = f.forced_stop_reason;
            return *this;
        }
        // Do the optimization:
        ResultType OptimizeStart(std::vector<double> &x, double &opt_f)
        {
            if (o && nlopt_get_dimension(o) != x.size())
                throw std::invalid_argument("dimension mismatch");
            forced_stop_reason = NLOPT_FORCED_STOP;
            nlopt_result ret = nlopt_optimize(o, x.empty() ? NULL : &x[0], &opt_f);
            last_result = ResultType(ret);
            last_optf = opt_f;
            if (ret == NLOPT_FORCED_STOP)
                NLOptThrow(forced_stop_reason);
            NLOptThrow(ret);
            return last_result;
        }
        // variant mainly useful for SWIG wrappers:
        std::vector<double> OptimizeStart(const std::vector<double> &x0)
        {
            std::vector<double> x(x0);
            last_result = OptimizeStart(x, last_optf);
            return x;
        }

        ResultType LastOptimizeResult() const
        {
            return last_result;
        }
        double LastOptimumValue() const
        {
            return last_optf;
        }
        // accessors:
        AlgorithmType GetAlgorithm() const
        {
            if (!o)
                throw std::runtime_error("uninitialized nlopt::opt");
            return AlgorithmType(nlopt_get_algorithm(o));
        }

        const char *GetAlgorithmName() const
        {
            if (!o)
                throw std::runtime_error("uninitialized nlopt::opt");
            return nlopt_algorithm_name(nlopt_get_algorithm(o));
        }
        unsigned GetDimension() const
        {
            if (!o)
                throw std::runtime_error("uninitialized nlopt::opt");
            return nlopt_get_dimension(o);
        }
        // Set the objective function
        void SetMinObjective(FuncPointer f, void *f_data)
        {
            NLOptFuncData *d = new NLOptFuncData;
            if (!d)
                throw std::bad_alloc();
            d->o = this;
            d->f = f;
            d->f_data = f_data;
            d->mf = NULL;
            d->vf = NULL;
            d->munge_destroy = d->munge_copy = NULL;
            NLOptThrow(nlopt_set_min_objective(o, ObjectiveFunc, d)); // d freed via o
        }
        void SetMinObjective(vFuncPointer vf, void *f_data)
        {
            NLOptFuncData *d = new NLOptFuncData;
            if (!d)
                throw std::bad_alloc();
            d->o = this;
            d->f = NULL;
            d->f_data = f_data;
            d->mf = NULL;
            d->vf = vf;
            d->munge_destroy = d->munge_copy = NULL;
            NLOptThrow(nlopt_set_min_objective(o, vObjectiveFunc, d)); // d freed via o
            AllocTemp();
        }
        void SetMaxObjective(FuncPointer f, void *f_data)
        {
            NLOptFuncData *d = new NLOptFuncData;
            if (!d)
                throw std::bad_alloc();
            d->o = this;
            d->f = f;
            d->f_data = f_data;
            d->mf = NULL;
            d->vf = NULL;
            d->munge_destroy = d->munge_copy = NULL;
            NLOptThrow(nlopt_set_max_objective(o, ObjectiveFunc, d)); // d freed via o
        }
        void SetMaxObjective(vFuncPointer vf, void *f_data)
        {
            NLOptFuncData *d = new NLOptFuncData;
            if (!d)
                throw std::bad_alloc();
            d->o = this;
            d->f = NULL;
            d->f_data = f_data;
            d->mf = NULL;
            d->vf = vf;
            d->munge_destroy = d->munge_copy = NULL;
            NLOptThrow(nlopt_set_max_objective(o, vObjectiveFunc, d)); // d freed via o
            AllocTemp();
        }
        // for internal use in SWIG wrappers -- variant that
        // takes ownership of f_data, with munging for destroy/copy
        void SetMinObjective(FuncPointer f, void *f_data, nlopt_munge md, nlopt_munge mc)
        {
            NLOptFuncData *d = new NLOptFuncData;
            if (!d)
                throw std::bad_alloc();
            d->o = this;
            d->f = f;
            d->f_data = f_data;
            d->mf = NULL;
            d->vf = NULL;
            d->munge_destroy = md; d->munge_copy = mc;
            NLOptThrow(nlopt_set_min_objective(o, ObjectiveFunc, d)); // d freed via o
        }
        void SetMaxObjective(FuncPointer f, void *f_data, nlopt_munge md, nlopt_munge mc)
        {
            NLOptFuncData *d = new NLOptFuncData;
            if (!d)
                throw std::bad_alloc();
            d->o = this;
            d->f = f;
            d->f_data = f_data;
            d->mf = NULL;
            d->vf = NULL;
            d->munge_destroy = md;
            d->munge_copy = mc;
            NLOptThrow(nlopt_set_max_objective(o, ObjectiveFunc, d)); // d freed via o
        }
        // Nonlinear constraints:
        void RemoveInequalityConstraints()
        {
            nlopt_result ret = nlopt_remove_inequality_constraints(o);
            NLOptThrow(ret);
        }
        void AddInequalityConstraint(FuncPointer f, void *f_data, double tol=0)
        {
            NLOptFuncData *d = new NLOptFuncData;
            if (!d)
                throw std::bad_alloc();
            d->o = this;
            d->f = f;
            d->f_data = f_data;
            d->mf = NULL;
            d->vf = NULL;
            d->munge_destroy = d->munge_copy = NULL;
            NLOptThrow(nlopt_add_inequality_constraint(o, ObjectiveFunc, d, tol));
        }
        void AddInequalityConstraint(vFuncPointer vf, void *f_data, double tol=0)
        {
            NLOptFuncData *d = new NLOptFuncData;
            if (!d)
                throw std::bad_alloc();
            d->o = this;
            d->f = NULL;
            d->f_data = f_data;
            d->mf = NULL;
            d->vf = vf;
            d->munge_destroy = d->munge_copy = NULL;
            NLOptThrow(nlopt_add_inequality_constraint(o, vObjectiveFunc, d, tol));
            AllocTemp();
        }
        void AddInequalitymConstraint(mFuncPointer mf, void *f_data, const std::vector<double> &tol)
        {
            NLOptFuncData *d = new NLOptFuncData;
            if (!d)
                throw std::bad_alloc();
            d->o = this;
            d->mf = mf;
            d->f_data = f_data;
            d->f = NULL;
            d->vf = NULL;
            d->munge_destroy = d->munge_copy = NULL;
            NLOptThrow(nlopt_add_inequality_mconstraint(o, tol.size(), mObjectiveFunc, d,
                                                        tol.empty() ? NULL : &tol[0]));
        }
        void RemoveEqualityConstraints()
        {
            nlopt_result ret = nlopt_remove_equality_constraints(o);
            NLOptThrow(ret);
        }
        void AddEqualityConstraint(FuncPointer f, void *f_data, double tol=0)
        {
            NLOptFuncData *d = new NLOptFuncData;
            if (!d)
                throw std::bad_alloc();
            d->o = this;
            d->f = f;
            d->f_data = f_data;
            d->mf = NULL;
            d->vf = NULL;
            d->munge_destroy = d->munge_copy = NULL;
            NLOptThrow(nlopt_add_equality_constraint(o, ObjectiveFunc, d, tol));
        }
        void AddEqualityConstraint(vFuncPointer vf, void *f_data, double tol=0)
        {
            NLOptFuncData *d = new NLOptFuncData;
            if (!d)
                throw std::bad_alloc();
            d->o = this;
            d->f = NULL;
            d->f_data = f_data;
            d->mf = NULL;
            d->vf = vf;
            d->munge_destroy = d->munge_copy = NULL;
            NLOptThrow(nlopt_add_equality_constraint(o, vObjectiveFunc, d, tol));
            AllocTemp();
        }
        void AddEqualitymConstraint(mFuncPointer mf, void *f_data, const std::vector<double> &tol)
        {
            NLOptFuncData *d = new NLOptFuncData;
            if (!d)
                throw std::bad_alloc();
            d->o = this;
            d->mf = mf;
            d->f_data = f_data;
            d->f = NULL;
            d->vf = NULL;
            d->munge_destroy = d->munge_copy = NULL;
            NLOptThrow(nlopt_add_equality_mconstraint(o, tol.size(), mObjectiveFunc, d,
                                                    tol.empty() ? NULL : &tol[0]));
        }
        // For internal use in SWIG wrappers (see also above)
        void AddInequalityConstraint(FuncPointer f, void *f_data,
                                        nlopt_munge md, nlopt_munge mc,
                                        double tol=0)
        {
            NLOptFuncData *d = new NLOptFuncData;
            if (!d)
                throw std::bad_alloc();
            d->o = this;
            d->f = f;
            d->f_data = f_data;
            d->mf = NULL;
            d->vf = NULL;
            d->munge_destroy = md; d->munge_copy = mc;
            NLOptThrow(nlopt_add_inequality_constraint(o, ObjectiveFunc, d, tol));
        }
        void AddEqualityConstraint(FuncPointer f, void *f_data,
                                    nlopt_munge md, nlopt_munge mc,
                                    double tol=0)
        {
            NLOptFuncData *d = new NLOptFuncData;
            if (!d)
                throw std::bad_alloc();
            d->o = this;
            d->f = f;
            d->f_data = f_data;
            d->mf = NULL;
            d->vf = NULL;
            d->munge_destroy = md;
            d->munge_copy = mc;
            NLOptThrow(nlopt_add_equality_constraint(o, ObjectiveFunc, d, tol));
        }
        void AddInequalitymConstraint(mFuncPointer mf, void *f_data,
                                        nlopt_munge md, nlopt_munge mc,
                                        const std::vector<double> &tol)
        {
            NLOptFuncData *d = new NLOptFuncData;
            if (!d)
                throw std::bad_alloc();
            d->o = this;
            d->mf = mf;
            d->f_data = f_data;
            d->f = NULL;
            d->vf = NULL;
            d->munge_destroy = md;
            d->munge_copy = mc;
            NLOptThrow(nlopt_add_inequality_mconstraint(o, tol.size(), mObjectiveFunc, d,
                                                        tol.empty() ? NULL : &tol[0]));
        }
        void AddEqualitymConstraint(mFuncPointer mf, void *f_data,
                                    nlopt_munge md, nlopt_munge mc,
                                    const std::vector<double> &tol)
        {
            NLOptFuncData *d = new NLOptFuncData;
            if (!d)
                throw std::bad_alloc();
            d->o = this;
            d->mf = mf;
            d->f_data = f_data;
            d->f = NULL;
            d->vf = NULL;
            d->munge_destroy = md;
            d->munge_copy = mc;
            NLOptThrow(nlopt_add_equality_mconstraint(o, tol.size(), mObjectiveFunc, d,
                                                        tol.empty() ? NULL : &tol[0]));
        }
        /*
        #define NLOPT_GETSET_VEC(name)						\
        void set_##name(double val)                         \
        {                                                   \
            NLOptThrow(nlopt_set_##name##1(o, val));           \
        }                                                   \
        void get_##name(std::vector<double> &v) const                   \
        {                                                               \
            if (o && nlopt_get_dimension(o) != v.size())                \
                throw std::invalid_argument("dimension mismatch");      \
            NLOptThrow(nlopt_get_##name(o, v.empty() ? NULL : &v[0]));     \
        }                                                               \
        std::vector<double> get_##name() const                              \
        {                                                                   \
            if (!o) throw std::runtime_error("uninitialized nlopt::opt");   \
            std::vector<double> v(nlopt_get_dimension(o));                  \
            get_##name(v);                                                  \
            return v;                                                       \
        }                                                                   \
        void set_##name(const std::vector<double> &v)               \
        {                                                           \
            if (o && nlopt_get_dimension(o) != v.size())			\
                throw std::invalid_argument("dimension mismatch");  \
            NLOptThrow(nlopt_set_##name(o, v.empty() ? NULL : &v[0])); \
        }
        */
        //NLOPT_GETSET_VEC(lower_bounds)
        void SetLowerBounds(double val)
        {
            NLOptThrow(nlopt_set_lower_bounds1(o, val));
        }
        void GetLowerBounds(std::vector<double> &v) const
        {
            if (o && nlopt_get_dimension(o) != v.size())
                throw std::invalid_argument("dimension mismatch");
            NLOptThrow(nlopt_get_lower_bounds(o, v.empty() ? NULL : &v[0]));
        }
        void SetLowerBounds(const std::vector<double> &v)
        {
            if (o && nlopt_get_dimension(o) != v.size())
                throw std::invalid_argument("dimension mismatch");
            NLOptThrow(nlopt_set_lower_bounds(o, v.empty() ? NULL : &v[0]));
        }
        std::vector<double> GetLowerBounds() const
        {
            if (!o) throw std::runtime_error("uninitialized nlopt::opt");
            std::vector<double> v(nlopt_get_dimension(o));
            GetLowerBounds(v);
            return v;
        }

        //NLOPT_GETSET_VEC(upper_bounds)
        void SetUpperBounds(double val)
        {
            NLOptThrow(nlopt_set_upper_bounds1(o, val));
        }
        void GetUpperBounds(std::vector<double> &v) const
        {
            if (o && nlopt_get_dimension(o) != v.size())
                throw std::invalid_argument("dimension mismatch");
            NLOptThrow(nlopt_get_upper_bounds(o, v.empty() ? NULL : &v[0]));
        }
        void SetUpperBounds(const std::vector<double> &v)
        {
            if (o && nlopt_get_dimension(o) != v.size())
                throw std::invalid_argument("dimension mismatch");
            NLOptThrow(nlopt_set_upper_bounds(o, v.empty() ? NULL : &v[0]));
        }
        std::vector<double> GetUpperBounds() const
        {
            if (!o) throw std::runtime_error("uninitialized nlopt::opt");
            std::vector<double> v(nlopt_get_dimension(o));
            GetUpperBounds(v);
            return v;
        }
        //NLOPT_GETSET_VEC(xtol_abs)
        void SetXTolAbs(double val)
        {
            NLOptThrow(nlopt_set_xtol_abs1(o, val));
        }
        void GetXTolAbs(std::vector<double> &v) const
        {
            if (o && nlopt_get_dimension(o) != v.size())
                throw std::invalid_argument("dimension mismatch");
            NLOptThrow(nlopt_get_xtol_abs(o, v.empty() ? NULL : &v[0]));
        }
        void SetXTolAbs(const std::vector<double> &v)
        {
            if (o && nlopt_get_dimension(o) != v.size())
                throw std::invalid_argument("dimension mismatch");
            NLOptThrow(nlopt_set_xtol_abs(o, v.empty() ? NULL : &v[0]));
        }
        std::vector<double> GetXTolAbs() const
        {
            if (!o) throw std::runtime_error("uninitialized nlopt::opt");
            std::vector<double> v(nlopt_get_dimension(o));
            GetXTolAbs(v);
            return v;
        }
        // stopping criteria:
        /*
        #define NLOPT_GETSET(T, name)                                       \
        T get_##name() const                                                \
        {                                                                   \
            if (!o)                                                         \
                throw std::runtime_error("uninitialized nlopt::opt");       \
            return nlopt_get_##name(o);                                     \
        }                                                                   \
        void set_##name(T name)                     \
        {                                           \
            NLOptThrow(nlopt_set_##name(o, name));     \
        }
        */
       // NLOPT_GETSET(double, stopval)
        double GetStopVal() const
        {
            if (!o)
                throw std::runtime_error("uninitialized nlopt::opt");
            return nlopt_get_stopval(o);
        }
        void SetStopVal(double name)
        {
            NLOptThrow(nlopt_set_stopval(o, name));
        }
        //NLOPT_GETSET(double, ftol_rel)
        double GetFTolRel() const
        {
            if (!o)
                throw std::runtime_error("uninitialized nlopt::opt");
            return nlopt_get_ftol_rel(o);
        }
        void SetFTolRel(double name)
        {
            NLOptThrow(nlopt_set_ftol_rel(o, name));
        }
        //NLOPT_GETSET(double, ftol_abs)
        double GetFTolAbs() const
        {
            if (!o)
                throw std::runtime_error("uninitialized nlopt::opt");
            return nlopt_get_ftol_abs(o);
        }
        void SetFTolAbs(double name)
        {
            NLOptThrow(nlopt_set_ftol_abs(o, name));
        }
        //NLOPT_GETSET(double, xtol_rel)
        double GetXTolRel() const
        {
            if (!o)
                throw std::runtime_error("uninitialized nlopt::opt");
            return nlopt_get_xtol_rel(o);
        }
        void SetXTolRel(double name)
        {
            NLOptThrow(nlopt_set_xtol_rel(o, name));
        }
        //NLOPT_GETSET(int, maxeval)
        int GetMaxeval() const
        {
            if (!o)
                throw std::runtime_error("uninitialized nlopt::opt");
            return nlopt_get_maxeval(o);
        }
        void SetMaxeval(int name)
        {
            NLOptThrow(nlopt_set_maxeval(o, name));
        }

        int GetNumevals() const
        {
            if (!o)
                throw std::runtime_error("uninitialized nlopt::opt");
            return nlopt_get_numevals(o);
        }

        //NLOPT_GETSET(double, maxtime)
        double GetMaxTime() const
        {
            if (!o)
                throw std::runtime_error("uninitialized nlopt::opt");
            return nlopt_get_maxtime(o);
        }
        void SetMaxTime(double name)
        {
            NLOptThrow(nlopt_set_maxtime(o, name));
        }
        //NLOPT_GETSET(int, force_stop)
        int GetForceStop() const
        {
            if (!o)
                throw std::runtime_error("uninitialized nlopt::opt");
            return nlopt_get_force_stop(o);
        }
        void SetForceStop(int name)
        {
            NLOptThrow(nlopt_set_force_stop(o, name));
        }

        void ForceStop()
        {
            SetForceStop(1);
        }
        const char *GetErrMsg() const
        {

            if (!o)
                throw std::runtime_error("uninitialized nlopt::opt");
            return nlopt_get_errmsg(o);
        }
        // algorithm-specific parameters:
        void SetLocalOptimizer(const NLOptimize &lo)
        {
            nlopt_result ret = nlopt_set_local_optimizer(o, lo.o);
            NLOptThrow(ret);
        }
        //NLOPT_GETSET(unsigned, population)
        unsigned GetPopulation() const
        {
            if (!o)
                throw std::runtime_error("uninitialized nlopt::opt");
            return nlopt_get_population(o);
        }
        void SetPopulation(unsigned name)
        {
            NLOptThrow(nlopt_set_population(o, name));
        }
        //NLOPT_GETSET(unsigned, vector_storage)
        unsigned GetVectorStorage() const
        {
            if (!o)
                throw std::runtime_error("uninitialized nlopt::opt");
            return nlopt_get_vector_storage(o);
        }
        void SetVectorStorage(unsigned name)
        {
            NLOptThrow(nlopt_set_vector_storage(o, name));
        }
        //NLOPT_GETSET_VEC(initial_step)
        void SetInitialStep(double val)
        {
            NLOptThrow(nlopt_set_initial_step1(o, val));
        }
        void GetInitialStep(const std::vector<double> &x, std::vector<double> &dx) const
        {
            if (o && (nlopt_get_dimension(o) != x.size()
                || nlopt_get_dimension(o) != dx.size()))
                throw std::invalid_argument("dimension mismatch");
            nlopt_result ret = nlopt_get_initial_step(o, x.empty() ? NULL : &x[0],
                                                    dx.empty() ? NULL : &dx[0]);
            NLOptThrow(ret);
        }
        void SetInitialStep(const std::vector<double> &v)
        {
            if (o && nlopt_get_dimension(o) != v.size())
                throw std::invalid_argument("dimension mismatch");
            NLOptThrow(nlopt_set_initial_step(o, v.empty() ? NULL : &v[0]));
        }
        std::vector<double> GetInitialStep_(const std::vector<double> &x) const
        {
            if (!o)
                throw std::runtime_error("uninitialized nlopt::opt");
            std::vector<double> v(nlopt_get_dimension(o));
            GetInitialStep(x, v);
            return v;
        }

        void SetDefaultInitialStep(const std::vector<double> &x)
        {
            nlopt_result ret
            = nlopt_set_default_initial_step(o, x.empty() ? NULL : &x[0]);
            NLOptThrow(ret);
        }

    private:
        nlopt_opt o;

        void NLOptThrow(nlopt_result ret) const
        {
            switch (ret)
            {
            case NLOPT_FAILURE:
                throw std::runtime_error(GetErrMsg() ? GetErrMsg() : "nlopt failure");
            case NLOPT_OUT_OF_MEMORY:
                throw std::bad_alloc();
            case NLOPT_INVALID_ARGS:
                throw std::invalid_argument(GetErrMsg() ? GetErrMsg() : "nlopt invalid argument");
            case NLOPT_ROUNDOFF_LIMITED:
                throw RoundoffLimited();
            case NLOPT_FORCED_STOP:
                throw ForcedStop();
            default: break;
            }
        }

        struct NLOptFuncData
        {
            NLOptimize *o;
            mFuncPointer mf;
            FuncPointer f;
            void *f_data;
            vFuncPointer vf;
            nlopt_munge munge_destroy, munge_copy; // non-NULL for SWIG wrappers
        };

        // free/destroy f_data in nlopt_destroy and nlopt_copy, respectively
        static void *FreeNLOptFuncData(void *p)
        {
            NLOptFuncData *d = (NLOptFuncData *) p;
            if (d)
            {
                if (d->f_data && d->munge_destroy)
                    d->munge_destroy(d->f_data);
                delete d;
            }
            return NULL;
        }

        static void *DupNLOptFuncData(void *p)
        {
            NLOptFuncData *d = (NLOptFuncData *) p;
            if (d)
            {
                void *f_data;
                if (d->f_data && d->munge_copy)
                {
                    f_data = d->munge_copy(d->f_data);
                    if (!f_data)
                        return NULL;
                }
                else
                    f_data = d->f_data;

                NLOptFuncData *dnew = new NLOptFuncData;
                if (dnew)
                {
                    *dnew = *d;
                    dnew->f_data = f_data;
                }
                    return (void*) dnew;
            }
            else
                return NULL;
        }
        // nlopt_func wrapper that catches exceptions
        static double   ObjectiveFunc(unsigned n, const double *x, double *grad, void *d_)
        {
            NLOptFuncData *d = reinterpret_cast<NLOptFuncData*>(d_);
            try
            {
                return d->f(n, x, grad, d->f_data);
            }
            catch (std::bad_alloc&)
            {
                d->o->forced_stop_reason = NLOPT_OUT_OF_MEMORY;
            }
            catch (std::invalid_argument&)
            {
                d->o->forced_stop_reason = NLOPT_INVALID_ARGS;
            }
            catch (RoundoffLimited&)
            {
                d->o->forced_stop_reason = NLOPT_ROUNDOFF_LIMITED;
            }
            catch (ForcedStop&)
            {
                d->o->forced_stop_reason = NLOPT_FORCED_STOP;
            }
            catch (...)
            {
                d->o->forced_stop_reason = NLOPT_FAILURE;
            }
            d->o->ForceStop(); // stop gracefully, opt::optimize will re-throw
            return HUGE_VAL;
        }
        // nlopt_mfunc wrapper that catches exceptions
        static void     mObjectiveFunc(unsigned m, double *result,
                                    unsigned n, const double *x, double *grad, void *d_)
        {
            NLOptFuncData *d = reinterpret_cast<NLOptFuncData*>(d_);
            try
            {
                d->mf(m, result, n, x, grad, d->f_data);
                return;
            }
            catch (std::bad_alloc&)
            {
                d->o->forced_stop_reason = NLOPT_OUT_OF_MEMORY;
            }
            catch (std::invalid_argument&)
            {
                d->o->forced_stop_reason = NLOPT_INVALID_ARGS;
            }
            catch (RoundoffLimited&)
            {
                d->o->forced_stop_reason = NLOPT_ROUNDOFF_LIMITED;
            }
            catch (ForcedStop&)
            {
                d->o->forced_stop_reason = NLOPT_FORCED_STOP;
            }
            catch (...)
            {
                d->o->forced_stop_reason = NLOPT_FAILURE; }
                d->o->ForceStop(); // stop gracefully, opt::optimize will re-throw
                for (unsigned i = 0; i < m; ++i)
                    result[i] = HUGE_VAL;
        }

        std::vector<double> xtmp;
        std::vector<double> gradtmp;
        std::vector<double> gradtmp0; // scratch for myvfunc
        // nlopt_func wrapper, using std::vector<double>
        static double vObjectiveFunc(unsigned n, const double *x, double *grad, void *d_)
        {
            NLOptFuncData *d = reinterpret_cast<NLOptFuncData*>(d_);
            try
            {
                std::vector<double> &xv = d->o->xtmp;
                if (n)
                    std::memcpy(&xv[0], x, n * sizeof(double));

                double val = d->vf(xv, grad ? d->o->gradtmp : d->o->gradtmp0, d->f_data);
                if (grad && n)
                {
                    std::vector<double> &gradv = d->o->gradtmp;
                    std::memcpy(grad, &gradv[0], n * sizeof(double));
                }
                return val;
            }
            catch (std::bad_alloc&)
            {
                d->o->forced_stop_reason = NLOPT_OUT_OF_MEMORY;
            }
            catch (std::invalid_argument&)
            {
                d->o->forced_stop_reason = NLOPT_INVALID_ARGS;
            }
            catch (RoundoffLimited&)
            {
                d->o->forced_stop_reason = NLOPT_ROUNDOFF_LIMITED;
            }
            catch (ForcedStop&)
            {
                d->o->forced_stop_reason = NLOPT_FORCED_STOP;
            }
            catch (...)
            {
                d->o->forced_stop_reason = NLOPT_FAILURE; }
                d->o->ForceStop(); // stop gracefully, opt::optimize will re-throw
                return HUGE_VAL;
        }
        void AllocTemp()
        {
            if (xtmp.size() != nlopt_get_dimension(o))
            {
                xtmp = std::vector<double>(nlopt_get_dimension(o));
                gradtmp = std::vector<double>(nlopt_get_dimension(o));
            }
        }

        ResultType      last_result;
        double          last_optf;
        nlopt_result    forced_stop_reason;


    }; //END Of Class NLOptimize
    #undef NLOPT_GETSET
    #undef NLOPT_GETSET_VEC
    //////////////////////////////////////////////////////////////////////
    inline void srand(unsigned long seed)
    {
        nlopt_srand(seed);
    }
    inline void srand_time()
    {
        nlopt_srand_time();
    }
    inline void version(int &major, int &minor, int &bugfix)
    {
        nlopt_version(&major, &minor, &bugfix);
    }
    inline int version_major()
    {
        int major, minor, bugfix;
        nlopt_version(&major, &minor, &bugfix);
        return major;
    }
    inline int version_minor()
    {
        int major, minor, bugfix;
        nlopt_version(&major, &minor, &bugfix);
        return minor;
    }
    inline int version_bugfix()
    {
        int major, minor, bugfix;
        nlopt_version(&major, &minor, &bugfix);
        return bugfix;
    }
    inline const char *algorithm_name(AlgorithmType a)
    {
        return nlopt_algorithm_name(nlopt_algorithm(a));
    }
    //////////////////////////////////////////////////////////////////////
} // namespace nlopt
#endif /* NLOPT_HPP */
