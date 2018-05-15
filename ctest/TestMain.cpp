#include <iostream>

#include "SpaceDSL/SpaceDSL.h"

using namespace std;
using namespace SpaceDSL;


int main(int argc, char *argv[])
{
    cout<<"SpaceDSL Test Run!"<<endl;
    try
    {
        UTCCalTime time(2018,1,4,16,58,11.1);
        IERSService iers;

        double mass0 = 10000.0;
        CartState cart1(-5107324.219684929600, -3582177.314118019800, -2477461.707380509900,
                        4925.070866580500, -3984.883326371030, -4391.339344033090);
        double Mjd_UTC = CalendarTimeToMjd(time);
        double y_pole = iers.GetValue(Mjd_UTC, "y_pole");
        char *timeStr = NULL;
        time.ToCharArray(timeStr);

        cout << "y_pole at " << timeStr <<" = "<< y_pole << "\"" <<endl;
        double Mjd_TT = CalendarTimeToMjd(time);
        double Mjd_UT1 = Mjd_TT;

        ThirdBodyGravitySign thirdGravSign;
        thirdGravSign.bIsUseSunGrav = true;
        thirdGravSign.bIsUseMoonGrav = true;

        OrbitPredictConfig preConfig;
        preConfig.Initializer(E_Earth,thirdGravSign,Mjd_TT,E_EGM08Model,20,20,E_1976StdAtmosphere,1.2,2,1.2,5,true,true);

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
