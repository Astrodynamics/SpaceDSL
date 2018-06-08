#include <iostream>

#include "SpaceDSL/SpaceDSL.h"

using namespace std;
using namespace SpaceDSL;
class MyTread : public SpThread
{
public:
    MyTread(){}
    ~MyTread(){}

public:

    void Run() override
    {
        UTCCalTime time(2018,1,4,16,58,11.1);


        double mass0 = 1000.0;
        CartState cart0(-5107324.219684929600, -3582177.314118019800, -2477461.707380509900,
                        4925.070866580500, -3984.883326371030, -4391.339344033090);
        double Mjd_UTC0 = CalendarTimeToMjd(time);

        double step = 10;
        double Mjd_UTC = Mjd_UTC0;

        OrbitPredictConfig::ThirdBodyGravitySign thirdGravSign;
        thirdGravSign.bIsUseSunGrav = true;
        thirdGravSign.bIsUseMoonGrav = true;

        OrbitPredictConfig preConfig1,preConfig2,preConfig_twoBody;
        preConfig1.Initializer(Mjd_UTC0, E_Earth, false,
                              GravityModel::GravModelType::E_EGM08Model,20 , 20, thirdGravSign,
                              AtmosphereModel::AtmosphereModelType::E_NRLMSISE00Atmosphere, 1.2, 2, 2.2, 20, false, true);
        preConfig2.Initializer(Mjd_UTC0,E_Earth,false,GravityModel::GravModelType::E_EGM08Model,20,20);
        preConfig_twoBody.Initializer(Mjd_UTC0,E_Earth,false);
        TwoBodyOrbitPredict twoBodyPre;

        Vector3d pos = cart0.Pos(), vel = cart0.Vel();

        /*
        cout << "Time (ModJDate)      x (km)         y (km)        z (km)   vx (km/sec)   vy (km/sec)   vz (km/sec)" <<endl;
        cout << "---------------    ---------       -------        -------   -----------   -----------  -----------" <<endl;
        cout.precision(12);
        cout << Mjd_UTC << "       " <<pos(0)/1000 << "       " <<pos(1)/1000 << "       " <<pos(2)/1000 << "       " <<
                vel(0)/1000<< "       " <<vel(1)/1000<< "       " <<vel(2)/1000<< "       " <<endl;
        for (int i = 0; i < DayToSec/step; ++i)
        {
            preConfig_twoBody.SetMJD_UTC(Mjd_UTC);
            twoBodyPre.OrbitStep(preConfig_twoBody,step,RungeKutta::E_RungeKutta4, mass0, pos, vel);
            Mjd_UTC = Mjd_UTC0 + (i+1) * step/DayToSec;
            //cout << Mjd_UTC << "       " <<pos(0)/1000 << "       " <<pos(1)/1000 << "       " <<pos(2)/1000 << "       " <<
            //       vel(0)/1000<< "       " <<vel(1)/1000<< "       " <<vel(2)/1000<< "       " <<endl;
        }
        cout << Mjd_UTC << "       " <<pos(0)/1000 << "       " <<pos(1)/1000 << "       " <<pos(2)/1000 << "       " <<
                vel(0)/1000<< "       " <<vel(1)/1000<< "       " <<vel(2)/1000<< "       " <<endl;

        */
        OrbitPredict orbit;

        cout << "Time (ModJDate)      x (km)         y (km)        z (km)   vx (km/sec)   vy (km/sec)   vz (km/sec)" <<endl;
        cout << "---------------    ---------       -------        -------   -----------   -----------  -----------" <<endl;
        cout.precision(15);
        cout << Mjd_UTC << "       " <<pos(0)/1000 << "       " <<pos(1)/1000 << "       " <<pos(2)/1000 << "       " <<
                vel(0)/1000<< "       " <<vel(1)/1000<< "       " <<vel(2)/1000<< "       " <<endl;
        for (int i = 0; i < DayToSec/step; ++i)
        {
            preConfig1.SetMJD_UTC(Mjd_UTC);
            orbit.OrbitStep(preConfig1, step, RungeKutta::E_RungeKutta4, mass0, pos, vel);
            Mjd_UTC = Mjd_UTC0 + (i+1) * step/DayToSec;
            //cout << Mjd_UTC << "       " <<pos(0)/1000 << "       " <<pos(1)/1000 << "       " <<pos(2)/1000 << "       " <<
            //       vel(0)/1000<< "       " <<vel(1)/1000<< "       " <<vel(2)/1000<< "       " <<endl;
        }
        cout << Mjd_UTC << "       " <<pos(0)/1000 << "       " <<pos(1)/1000 << "       " <<pos(2)/1000 << "       " <<
                vel(0)/1000<< "       " <<vel(1)/1000<< "       " <<vel(2)/1000<< "       " <<endl;


    }
};

int main(int argc, char *argv[])
{
    cout<<"SpaceDSL Test Run!"<<endl;
    try
    {
        MyTread t;
        t.Start();
        t.SetPriority(SpThread::Priority::HighestPriority);
        cout<<"Before Suspend!"<<endl;
        t.Suspend();
        cout<<"After Suspend!"<<endl;
        t.Resume();
        t.Wait();
    }
    catch (SPException &e)
    {
        e.what();
    }
    return 0;
}
