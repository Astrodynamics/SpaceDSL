#include <iostream>

#include "SpaceDSL/SpOrbitParam.h"
#include "SpaceDSL/SpTimeSystem.h"
#include "SpaceDSL/SpJplEph.h"
#include "SpaceDSL/SpCoordSystem.h"
#include "SpaceDSL/SpGravity.h"
#include "SpaceDSL/SpAtmosphere.h"
#include "SpaceDSL/SpPerturbation.h"
#include "SpaceDSL/SpInterpolation.h"
#include "SpaceDSL/SpRightFunction.h"
#include "SpaceDSL/SpIntegration.h"
#include "SpaceDSL/SpOrbitPredict.h"
#include "SpaceDSL/SpMath.h"
#include "SpaceDSL/SpConst.h"
#include "SpaceDSL/SpUtils.h"

using namespace std;
using namespace SpaceDSL;


int main(int argc, char *argv[])
{
    cout<<"SpaceDSL Test Run!"<<endl;
    try
    {
        double mass0 = 10000.0;
        CartState cart1(-5107324.219684929600, -3582177.314118019800, -2477461.707380509900,
                        4925.070866580500, -3984.883326371030, -4391.339344033090);
        UTCCalTime time(2018,4,4,16,58,11.1);
        double Mjd_TT = CalendarTimeToMjd(time);
        double Mjd_UT1 = Mjd_TT;

        ThirdBodyGravitySign thirdGravSign;
        thirdGravSign.m_bIsUseSunGrav = true;
        thirdGravSign.m_bIsUseMoonGrav = true;

        OrbitPredictConfig preConfig;
        preConfig.Initializer(E_Earth,Mjd_TT,E_EGM08Model,20,20,E_1976StdAtmosphere,1.2,2,1.2,5,true,true,thirdGravSign);

        OrbitPredict orbit;
        Vector3d pos = cart1.Pos(),vel = cart1.Vel();
        cout << "start = " << pos << endl;
        cout << vel << endl;
        orbit.OrbitStep(preConfig, 10, E_RungeKutta4, mass0, pos, vel);
        cout << "end = " << pos << endl;
        cout << vel << endl;

    }
    catch (SPException &e)
    {
        e.what();
    }
    return 0;
}
