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
    try
    {
        UTCCalTime time(2018,4,4,16,58,11.1);
        double mjd = CalendarTimeToMjd(time);
        OrbitElem elem1;
        CartState cart1(-5107324.219684929600, -3582177.314118019800, -2477461.707380509900,
                        4925.070866580500, -3984.883326371030, -4391.339344033090);
        CartToOrbitElem(cart1, GM_Earth, elem1);

        CartState cart2;
        OrbitElemToCart(elem1, GM_Earth, cart2);
    }
    catch (SPException &e)
    {
        e.what();
    }
    return 0;
}
