#include <iostream>

#include "SpaceDSL/SpOrbitParam.h"
#include "SpaceDSL/SpUtils.h"

using namespace std;
using namespace SpaceDSL;
int main(int argc, char *argv[])
{
    cout<<"SpaceDSL Test Run!"<<endl;

    CartState state(1,2,3,0.1,0.2,0.3);

    auto pos = state.Pos();

    cout << pos << endl;
    try
    {
        throw SPException(__FILE__, __FUNCTION__, __LINE__, "Test!");
    }
    catch (SPException &e)
    {
        e.what();
    }
    return 0;
}
