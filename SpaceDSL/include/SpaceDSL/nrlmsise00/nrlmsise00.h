/* -------------------------------------------------------------------- */
/* ---------  N R L M S I S E - 0 0    M O D E L    2 0 0 1  ---------- */
/* -------------------------------------------------------------------- */

/* This file is part of the NRLMSISE-00  C source code package - release
 * 20041227
 *
 * The NRLMSISE-00 model was developed by Mike Picone, Alan Hedin, and
 * Doug Drob. They also wrote a NRLMSISE-00 distribution package in 
 * FORTRAN which is available at
 * http://uap-www.nrl.navy.mil/models_web/msis/msis_home.htm
 *
 * Dominik Brodowski implemented and maintains this C version. You can
 * reach him at mail@brodo.de. See the file "DOCUMENTATION" for details,
 * and check http://www.brodo.de/english/pub/nrlmsise/index.html for
 * updated releases of this package.
 */

#include "SpaceDSL/SpaceDSL_Global.h"

#include <math.h>

/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {
    /*************************************************
     * Class type: NRLMSISE-00 model Class of SpaceDSL
     * Author: Niu ZhiYong
     * Date:2018-05-20
     * Description:
     *  This Class is Thread Safety.
    **************************************************/
    class SPACEDSL_API NRLMSISE00
    {
    public:
        NRLMSISE00();
        ~NRLMSISE00();

        /* ------------------------------------------------------------------- */
        /* ------------------------------- INPUT ----------------------------- */
        /* ------------------------------------------------------------------- */
        struct nrlmsise_flags
        {
            int switches[24];
            double sw[24];
            double swc[24];
        };
        /*
         *   Switches: to turn on and off particular variations use these switches.
         *   0 is off, 1 is on, and 2 is main effects off but cross terms on.
         *
         *   Standard values are 0 for switch 0 and 1 for switches 1 to 23. The
         *   array "switches" needs to be set accordingly by the calling program.
         *   The arrays sw and swc are set internally.
         *
         *   switches[i]:
         *    i - explanation
         *   -----------------
         *    0 - output in meters and kilograms instead of centimeters and grams
         *    1 - F10.7 effect on mean
         *    2 - time independent
         *    3 - symmetrical annual
         *    4 - symmetrical semiannual
         *    5 - asymmetrical annual
         *    6 - asymmetrical semiannual
         *    7 - diurnal
         *    8 - semidiurnal
         *    9 - daily ap [when this is set to -1 (!) the pointer
         *                  ap_a in struct nrlmsise_input must
         *                  point to a struct ap_array]
         *   10 - all UT/long effects
         *   11 - longitudinal
         *   12 - UT and mixed UT/long
         *   13 - mixed AP/UT/LONG
         *   14 - terdiurnal
         *   15 - departures from diffusive equilibrium
         *   16 - all TINF var
         *   17 - all TLB var
         *   18 - all TN1 var
         *   19 - all S var
         *   20 - all TN2 var
         *   21 - all NLB var
         *   22 - all TN3 var
         *   23 - turbo scale height var
         */

        struct ap_array
        {
            double a[7];
        };
        /* Array containing the following magnetic values:
         *   0 : daily AP
         *   1 : 3 hr AP index for current time
         *   2 : 3 hr AP index for 3 hrs before current time
         *   3 : 3 hr AP index for 6 hrs before current time
         *   4 : 3 hr AP index for 9 hrs before current time
         *   5 : Average of eight 3 hr AP indicies from 12 to 33 hrs
         *           prior to current time
         *   6 : Average of eight 3 hr AP indicies from 36 to 57 hrs
         *           prior to current time
         */


        struct nrlmsise_input
        {
            int year;      /* year, currently ignored */
            int doy;       /* day of year */
            double sec;    /* seconds in day (UT) */
            double alt;    /* altitude in kilometers */
            double g_lat;  /* geodetic latitude */
            double g_long; /* geodetic longitude */
            double lst;    /* local apparent solar time (hours), see note below */
            double f107A;  /* 81 day average of F10.7 flux (centered on doy) */
            double f107;   /* daily F10.7 flux for previous day */
            double ap;     /* magnetic index(daily) */
            struct ap_array *ap_a; /* see above */
        };
        /*
         *   NOTES ON INPUT VARIABLES:
         *      UT, Local Time, and Longitude are used independently in the
         *      model and are not of equal importance for every situation.
         *      For the most physically realistic calculation these three
         *      variables should be consistent (lst=sec/3600 + g_long/15).
         *      The Equation of Time departures from the above formula
         *      for apparent local time can be included if available but
         *      are of minor importance.
         *
         *      f107 and f107A values used to generate the model correspond
         *      to the 10.7 cm radio flux at the actual distance of the Earth
         *      from the Sun rather than the radio flux at 1 AU. The following
         *      site provides both classes of values:
         *      ftp://ftp.ngdc.noaa.gov/STP/SOLAR_DATA/SOLAR_RADIO/FLUX/
         *
         *      f107, f107A, and ap effects are neither large nor well
         *      established below 80 km and these parameters should be set to
         *      150., 150., and 4. respectively.
         */


        /* ------------------------------------------------------------------- */
        /* ------------------------------ OUTPUT ----------------------------- */
        /* ------------------------------------------------------------------- */

        struct nrlmsise_output
        {
            double d[9];   /* densities */
            double t[2];   /* temperatures */
        };
        /*
         *   OUTPUT VARIABLES:
         *      d[0] - HE NUMBER DENSITY(CM-3)
         *      d[1] - O NUMBER DENSITY(CM-3)
         *      d[2] - N2 NUMBER DENSITY(CM-3)
         *      d[3] - O2 NUMBER DENSITY(CM-3)
         *      d[4] - AR NUMBER DENSITY(CM-3)
         *      d[5] - TOTAL MASS DENSITY(GM/CM3) [includes d[8] in td7d]
         *      d[6] - H NUMBER DENSITY(CM-3)
         *      d[7] - N NUMBER DENSITY(CM-3)
         *      d[8] - Anomalous oxygen NUMBER DENSITY(CM-3)
         *      t[0] - EXOSPHERIC TEMPERATURE
         *      t[1] - TEMPERATURE AT ALT
         *
         *
         *      O, H, and N are set to zero below 72.5 km
         *
         *      t[0], Exospheric temperature, is set to global average for
         *      altitudes below 120 km. The 120 km gradient is left at global
         *      average value for altitudes below 72 km.
         *
         *      d[5], TOTAL MASS DENSITY, is NOT the same for subroutines GTD7
         *      and GTD7D
         *
         *        SUBROUTINE GTD7 -- d[5] is the sum of the mass densities of the
         *        species labeled by indices 0-4 and 6-7 in output variable d.
         *        This includes He, O, N2, O2, Ar, H, and N but does NOT include
         *        anomalous oxygen (species index 8).
         *
         *        SUBROUTINE GTD7D -- d[5] is the "effective total mass density
         *        for drag" and is the sum of the mass densities of all species
         *        in this model, INCLUDING anomalous oxygen.
         */



        /* ------------------------------------------------------------------- */
        /* --------------------------- PROTOTYPES ---------------------------- */
        /* ------------------------------------------------------------------- */

        /* GTD7 */
        /*   Neutral Atmosphere Empircial Model from the surface to lower
         *   exosphere.
         */
        void  gtd7 (struct nrlmsise_input *input,
                    struct nrlmsise_flags *flags,
                    struct nrlmsise_output *output);


        /* GTD7D */
        /*   This subroutine provides Effective Total Mass Density for output
         *   d[5] which includes contributions from "anomalous oxygen" which can
         *   affect satellite drag above 500 km. See the section "output" for
         *   additional details.
         */
        void  gtd7d(struct nrlmsise_input *input,
                    struct nrlmsise_flags *flags,
                    struct nrlmsise_output *output);


        /* GTS7 */
        /*   Thermospheric portion of NRLMSISE-00
         */
        void  gts7 (struct nrlmsise_input *input,
                    struct nrlmsise_flags *flags,
                    struct nrlmsise_output *output);


        /* GHP7 */
        /*   To specify outputs at a pressure level (press) rather than at
         *   an altitude.
         */
        void  ghp7 (struct nrlmsise_input *input,
                    struct nrlmsise_flags *flags,
                    struct nrlmsise_output *output,
                    double press);
    private:

        void tselec(struct nrlmsise_flags *flags);

        /* ------------------------------------------------------------------- */
        /* ------------------------------ GLATF ------------------------------ */
        /* ------------------------------------------------------------------- */

        void glatf(double lat, double *gv, double *reff);

        /* ------------------------------------------------------------------- */
        /* ------------------------------ CCOR ------------------------------- */
        /* ------------------------------------------------------------------- */

        double ccor(double alt, double r, double h1, double zh);

        /* ------------------------------------------------------------------- */
        /* ------------------------------ CCOR ------------------------------- */
        /* ------------------------------------------------------------------- */

        double ccor2(double alt, double r, double h1, double zh, double h2);

        /* ------------------------------------------------------------------- */
        /* ------------------------------- SCALH ----------------------------- */
        /* ------------------------------------------------------------------- */

        double scalh(double alt, double xm, double temp);

        /* ------------------------------------------------------------------- */
        /* -------------------------------- DNET ----------------------------- */
        /* ------------------------------------------------------------------- */

        double dnet (double dd, double dm, double zhm, double xmm, double xm);

        /* ------------------------------------------------------------------- */
        /* ------------------------------- SPLINI ---------------------------- */
        /* ------------------------------------------------------------------- */

        void splini (double *xa, double *ya, double *y2a, int n, double x, double *y);

        /* ------------------------------------------------------------------- */
        /* ------------------------------- SPLINT ---------------------------- */
        /* ------------------------------------------------------------------- */

        void splint (double *xa, double *ya, double *y2a, int n, double x, double *y);

        /* ------------------------------------------------------------------- */
        /* ------------------------------- SPLINE ---------------------------- */
        /* ------------------------------------------------------------------- */

        void spline (double *x, double *y, int n, double yp1, double ypn, double *y2);

        /* ------------------------------------------------------------------- */
        /* ------------------------------- DENSM ----------------------------- */
        /* ------------------------------------------------------------------- */

        inline double zeta(double zz, double zl) {
            return ((zz-zl)*(re+zl)/(re+zz));
        }

        double densm (double alt, double d0, double xm, double *tz, int mn3, double *zn3, double *tn3, double *tgn3, int mn2, double *zn2, double *tn2, double *tgn2);

        /* ------------------------------------------------------------------- */
        /* ------------------------------- DENSU ----------------------------- */
        /* ------------------------------------------------------------------- */

        double densu (double alt, double dlb, double tinf, double tlb, double xm, double alpha, double *tz, double zlb, double s2, int mn1, double *zn1, double *tn1, double *tgn1);

        /* ------------------------------------------------------------------- */
        /* ------------------------------- GLOBE7 ---------------------------- */
        /* ------------------------------------------------------------------- */

        /*    3hr Magnetic activity functions */
        /*    Eq. A24d */
        inline double g0(double a, double *p) {
            return (a - 4.0 + (p[25] - 1.0) * (a - 4.0 + (exp(-sqrt(p[24]*p[24]) * (a - 4.0)) - 1.0) / sqrt(p[24]*p[24])));
        }

        /*    Eq. A24c */
        inline double sumex(double ex) {
            return (1.0 + (1.0 - pow(ex,19.0)) / (1.0 - ex) * pow(ex,0.5));
        }

        /*    Eq. A24a */
        inline double sg0(double ex, double *p, double *ap) {
            return (g0(ap[1],p) + (g0(ap[2],p)*ex + g0(ap[3],p)*ex*ex + \
                        g0(ap[4],p)*pow(ex,3.0)	+ (g0(ap[5],p)*pow(ex,4.0) + \
                        g0(ap[6],p)*pow(ex,12.0))*(1.0-pow(ex,8.0))/(1.0-ex)))/sumex(ex);
        }


        double globe7(double *p, struct nrlmsise_input *input, struct nrlmsise_flags *flags);

        /* ------------------------------------------------------------------- */
        /* ------------------------------- GLOB7S ---------------------------- */
        /* ------------------------------------------------------------------- */

        double glob7s(double *p, struct nrlmsise_input *input, struct nrlmsise_flags *flags);

    private:
        /* ------------------------------------------------------------------- */
        /* ------------------------- SHARED VARIABLES ------------------------ */
        /* ------------------------------------------------------------------- */

        /* PARMB */
        double gsurf;
        double re;

        /* GTS3C */
        double dd;

        /* DMIX */
        double dm04, dm16, dm28, dm32, dm40, dm01, dm14;

        /* MESO7 */
        double *meso_tn1;
        double *meso_tn2;
        double *meso_tn3;
        double *meso_tgn1;
        double *meso_tgn2;
        double *meso_tgn3;

        /* LPOLY */
        double dfa;
        double **plg;
        double ctloc, stloc;
        double c2tloc, s2tloc;
        double s3tloc, c3tloc;
        double apdf;
        double *apt;
    };

}

