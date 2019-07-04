#include "SpaceDSL/SpUtils.h"
#include <iostream>

using namespace std;
namespace SpaceDSL{

    /*************************************************
    Class type: Class of Exception Handling
    Author: Niu ZhiYong xiaogongwei
    Date:2018-09-08
    Description:
    Defined self Exception Class
    **************************************************/
    const char *SPException::what() const noexcept
    {
        char buf[10];
#if defined(_WIN32)
// Windows OS
        sprintf_s(buf, "%d", m_nLine);
#else
// Linux OS or MAC OS
        snprintf(buf, m_nLine, "%d");
#endif
        string sLine = buf;
        string sFile = m_pFileChar;
        string sFunc = m_pFunctionChar;
        string sReason = m_pReasonChar;
        string Exceptstr = "The File is:" + (sFile) + " \n"
                + "The Function Name is:" + (sFunc) + " \n"
                + "The Line is:" + sLine + " \n"
                + "The Reason is: " + (sReason) + " \n";

        static const char *Exceptchar ;
        Exceptchar = Exceptstr.c_str();
        cout << "An Exception has been violated here! \n" << endl;
        cout << Exceptchar << endl;
        return "The Program has Exited!";
    }
}


