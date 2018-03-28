#include <iostream>

#include "SpaceDSL/SpOrbitParam.h"
#include "SpaceDSL/SpTimeSystem.h"
#include "SpaceDSL/SpMath.h"
#include "SpaceDSL/SpConst.h"
#include "SpaceDSL/SpUtils.h"


using namespace std;
using namespace SpaceDSL;
int main(int argc, char *argv[])
{
    cout<<"SpaceDSL Test Run!"<<endl;

    CartState state(1,2,3,0.1,0.2,0.3);
    CalendarTime time(2018,3,28,4,0,0.0);
    double mjd = CalendarTimeToMjd(time);

    auto pos = state.Pos();

    cout << pos << endl;

    double f = Fraction(5.6);
    double m = Modulo(9,4);
    double y = 9 % 4;
    try
    {
        //throw SPException(__FILE__, __FUNCTION__, __LINE__, "Test!");
    }
    catch (SPException &e)
    {
        e.what();
    }
    return 0;
}
