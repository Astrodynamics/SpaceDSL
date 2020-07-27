/************************************************************************
* Copyright (C) 2020 Niu ZhiYong
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
* Date:2020-04-14
* Description:
*   SpAlgorithm.h
*
*   Purpose:
*
*       Optimization Algorithm Class
*
*
*   Last modified:
*
*   2020-04-14  Niu Zhiyong (1st edition)
*
*************************************************************************/

#ifndef SPALGORITHM_H
#define SPALGORITHM_H

#include "SpaceDSL_Global.h"

#include <Eigen/Core>

#include <vector>

using namespace Eigen;
using namespace std;
/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {


    /*************************************************
     * Class type:  Algorithm Result Information Base Class
     * Author: Niu ZhiYong
     * Date:2020-04-14
     * Description:
    **************************************************/
    class SPACEDSL_API ResultInfoBase
    {
    public:
        explicit ResultInfoBase();
        virtual ~ResultInfoBase();
    //
    // Attribute.
    //
    public:
        int                 m_Flag;
        double              m_ObjectValue;          ///< f
        int                 m_IterationTimes;       ///< niter;
        VectorXd            m_X;
    };
    /*************************************************
     * Class type:  Algorithm Config Base Class
     * Author: Niu ZhiYong
     * Date:2020-04-14
     * Description:
    **************************************************/
    class SPACEDSL_API AlgorithmConfigBase
    {
    public:
        explicit AlgorithmConfigBase();
        virtual ~AlgorithmConfigBase();
        //
        // Attribute.
        //
    };
    /*************************************************
     * Class type:  Algorithm Base Class
     * Author: Niu ZhiYong
     * Date:2020-04-14
     * Description:
    **************************************************/
    class SPACEDSL_API AlgorithmBase
    {
    public:
        explicit AlgorithmBase();
        virtual ~AlgorithmBase();

    public:
        virtual ResultInfoBase  StartOptimize(AlgorithmConfigBase &config,
                                  double (*pObjectFunc)(VectorXd x),
                                  VectorXd &(*pConstraintFunc)(VectorXd x),
                                  VectorXd &x0,VectorXd &lowBound, VectorXd &upBound,
                                  int eConstraintNum = 0, int ineConstraintNum = 0) = 0;
        //
        // Attribute.
        //
    };
    /*************************************************
     * Class type:  Sequential-Quadratic-Programming Derivative-Free Optimizatio Class
     * Author: Niu ZhiYong
     * Date:2020-04-14
     * Description:
    **************************************************/
    class SPACEDSL_API SQPDF : public AlgorithmBase
    {
    public:
        explicit SQPDF();
        virtual ~SQPDF();

        ///< Type of Hessian approximation
        enum HessianApproxType
        {
            E_NotDefinedApproxType  = 0,
            E_Model                 = 1,
            E_BFGS                  = 2
        };

        ///< Type of XXXX Model
        enum ModelType
        {
            E_NotDefinedModelType   = 0,
            E_Subbasis              = 1,
            E_Frobnorm              = 2,
            E_L2norm                = 3,
            E_Regression            = 4
        };

        ///< Final degree of the Model
        enum FinalDegreeType
        {
            E_NotDefinedDegreeType  = 0,
            E_Linear                = 1,
            E_Diagonal              = 2,
            E_Quadratic             = 3
        };

        ///< Final degree of the Model
        enum AlgoDescentType
        {
            E_NotDefinedDescentType = 0,
            E_Powell                = 1,
            E_Wolfe                 = 2
        };

        ///< Define sqpdfo constant values
        struct SQPDFConst
        {
            int success = 0;                        ///< solution found
            int fail_on_argument = 1;               ///< an argument is wrong
            int fail_on_problem = 2;                ///< unaccepted problem structure
            int fail_on_simul = 3;                  ///< error on a simulation
            int stop_on_simul = 4;                  ///< simulator wants to stop
            int stop_on_max_iter = 5;               ///< max iterations
            int stop_on_max_simul = 6;              ///< max simulations
            int stop_on_dxmin = 7;                  ///< stop on dxmin
            int fail_on_non_decrease = 8;           ///< the merit function no longer decrease
            int fail_on_ascent_dir = 9;             ///< nondescent direction in linesearch
            int fail_on_ill_cond = 11;              ///< ill-conditioning
            int stop_on_small_trust_region = 15;    ///< small trust-region radius
            int fail_on_null_step = 20;             ///< null step d is solution of 'values.max_null_steps' QPs
            int fail_on_infeasible_QP = 21;         ///< infeasible QP
            int fail_on_unbounded_QP = 22;          ///< unbounded QP
            int fail_unexpected = 30;               ///< should not have happened

            int nsimultype = 4;                     ///< nb of simulation types
            int max_null_steps = 1;                 ///< maximum nbr of null QP steps

            int powell = 120;
            int wolfe = 121;
            int bfgs = 130;
            int model = 131;
            int linear = 140;
            int diagonal = 141;
            int quadratic = 142;
            int subbasis =0;
            int frobnorm = 1;
            int l2norm = 2;
            int regression = 3;
        };

        struct SQPDFInfo
        {
            VectorXd nsimul;
            double f;
            VectorXd ce;
            VectorXd g;
            VectorXd ai;
            VectorXd ae;
            VectorXd hl;
            int niter;
            double glagn;
            double feasn;
            double complex;                ///< compl
        };

        /*************************************************
         * Class type:  Algorithm Result Information Base Class
         * Author: Niu ZhiYong
         * Date:2020-04-14
         * Description:
         *      m_Flag: output status
         *               0: solution found
         *               1: an argument is wrong
         *               2: unaccepted problem structure
         *               3: error on a function evaluation
         *               4: max iterations
         *               5: max simulations
         *               6: stop required
         *               7: stop on dxmin
         *               8: infeasible QP
         *               9: unbounded QP
         *              30: unexpected exit
        **************************************************/
        class SPACEDSL_API SQPDFResultInfo : public ResultInfoBase
        {
        public:
            explicit SQPDFResultInfo();
            virtual ~SQPDFResultInfo();
            //
            // Attribute.
            //
        public:
            VectorXd            nsimul;                 ///<nsimul[1]: number of function evaluations
            vector<VectorXd>    ce;
            vector<VectorXd>    g;
            vector<VectorXd>    ai;
            vector<VectorXd>    ae;
            vector<VectorXd>    hl;
            double              glagn;
            double              feasn;
            double              complex;                ///< compl
            vector<VectorXd>    ci;
            vector<VectorXd>    glag;
        };
        /*************************************************
         * Class type:  SQPDF Config Class
         * Author: Niu ZhiYong
         * Date:2020-04-14
         * Description:
         *  options (optional): structure for tuning the behavior of SQPDF
         *   (when a string is required, both lower or uppercase letters can
         *   be used, multiple white spaces are considered as a single white
         *   space)
         *   - options.miter = maximum number of iterations
         *   - options.tol = tolerance on optimality
         *  (1) = tolerance on the Lagrangian gradient
         *  (2) = tolerance on feasibility
         *  (3) = tolerance on bounds
         *  - options.verbose = verbosity level for the outputs (default 1)
        **************************************************/
        class SPACEDSL_API SQPDFConfig : public AlgorithmConfigBase
        {
        public:
            explicit SQPDFConfig();
            virtual ~SQPDFConfig();
            //
            // Attribute.
            //
            ///< Type of Hessian approximation, options: 'model' or 'bfgs'
            HessianApproxType hess_approx = E_Model;
            //Restart of the BFGS Hessian approximation after how many iterations
            //(only taken into account if hess_approx is 'bfgs')
            int bfgs_restart = 0;
            ModelType whichmodel = E_Subbasis;
            FinalDegreeType final_degree = E_Quadratic;  ///< Final degree of the model, options:

            double tol_grad = 1e-4;     ///< tolerance on the gradient of the Lagrangian
            double tol_feas = 1e-4;     ///< tolerance on the feasibility
            double tol_bnds = 1e-4;     ///< tolerance on the bounds
            int miter = 500;            ///< max iterations
            int msimul = 500;           ///< max evaluations
            int verbose = 1;            ///< verbosity level 0,...,3 (default: 1)

            AlgoDescentType algo_descent = E_Powell;
            double dxmin = 1e-08;
            int fout = 1;
            double inf;

        };


    public:
        ResultInfoBase  StartOptimize(AlgorithmConfigBase &config,
                                       double (*pObjectFunc)(VectorXd x),
                                       VectorXd &(*pConstraintFunc)(VectorXd x),
                                       VectorXd &x0,VectorXd &lowBound, VectorXd &upBound,
                                       int eConstraintNum = 0, int ineConstraintNum = 0) override;

    private:
        /*************************************************
         * This function realizes the following preliminary jobs:
         *- build initial poised interpolation set
         *- check the bounds and that initial point inside the bounds
         *- check the given options
         *- compute function and constraint values
         *- compute initial multipliers (if not given)
         *- initial printings
        **************************************************/
        void AlgoPreliminary();
        /*check user options or get default values if no values given by the user*/
        void AlgoOptions();
        //sqpdfo_options_()
            //concatenate_()
            //setdiff_()




        /*************************************************
         * This Main optimization loop for SQPDF.
        **************************************************/
        void AlgoMain();
        //sqpdfo_main_();


    //
    // Attribute.
    //
    private:
        SQPDFConfig     m_SQPDFConfig;
        double          (*m_pObjectFunc)(VectorXd x);
        VectorXd        &(*m_pConstraintFunc)(VectorXd x);
        ///< m_X0: n vector giving the initial guess of the solution,
        /// the number of variables is assumed to be the length of the given x0
        VectorXd        m_X0;
        int             m_XDim;
        ///< (m_EConstraintNum>=0) Equality constraints and Rn -> Rmi which means that there are
        int             m_EConstraintNum;
        ///< (m_IneConstraintNum>=0) Inequality constraints. Lower and upper bounds can be infinite
        int             m_IneConstraintNum;
        /*
         * m_LowBound
         * (optional): m_XDim + m_IneConstraintNum vector giving the lower bound
         *  on x (first n components) and ci(x),
         * it can have infinite components (default -inf)
        */
        VectorXd        m_LowBound;
        /*
         * m_LowBound
         * (optional): m_XDim + m_IneConstraintNum vector giving the upper bound
         *  on x (first n components) and ci(x),
         * it can have infinite components (default inf)
        */
        VectorXd        m_UpBound;

        SQPDFConst      m_SQPDFConstants;
        double          m_Threshold;



    };

}

#endif //SPALGORITHM_H

