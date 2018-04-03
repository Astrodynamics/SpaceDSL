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
        OrbitElem elem1;
        CartState cart1(-5107324.219684929600, -3582177.314118019800, -2477461.707380509900,
                        4925.070866580500, -3984.883326371030, -4391.339344033090);
        CartToOrbitElem(cart1, GM_Earth,elem1 );

        //OrbitElem elem(6716300, 0.0006, 0.74612, 0.167579, 3.71648, 6.283185);
        //CartState cart;
        //OrbitElemToCart(elem, GM_Earth, cart);
    }
    catch (SPException &e)
    {
        e.what();
    }
    return 0;
}
