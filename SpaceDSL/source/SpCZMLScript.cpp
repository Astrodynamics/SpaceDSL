/************************************************************************
* Copyright (C) 2018 Niu ZhiYong
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
* Date:2018-07-27
* Description:
*   SpCZMLScript.cpp
*
*   Purpose:
*
*        Read and Write CZML File by Jsoncpp
*
*
*   Last modified:
*
*   2018-07-27  Niu Zhiyong (1st edition)
*
*************************************************************************/

#include <SpaceDSL/SpCZMLScript.h>
#include <SpaceDSL/SpMission.h>
#include <SpaceDSL/SpUtils.h>

#include <fstream>
#include <iomanip>

namespace SpaceDSL {


    /*************************************************
     * Class type: The class of SpaceDSL Moderator
     * Author: Niu ZhiYong
     * Date:2018-07-27
     * Description:
    **************************************************/
    CZMLScript::CZMLScript()
    {
        m_pMission = nullptr;
        m_pJsonList = new vector<json>();
        m_pJsonToWirte = nullptr;
        m_bIsInitialized = false;
    }

    CZMLScript::~CZMLScript()
    {
        /*
        for(auto j:(*m_pJsonList))
        {
            delete &j;
        }
        */
        m_pJsonList->clear();

        if (m_pJsonToWirte != nullptr)
            delete m_pJsonToWirte;

    }

    void CZMLScript::Initializer(const string &filePath, const Mission *pMission, const double step)
    {
        m_FilePath = filePath;
        m_pMission = pMission;
        m_Step = step;

        m_bIsInitialized = true;
    }

    void CZMLScript::WirteCZML()
    {
        if (m_bIsInitialized != true)
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "CZMLScript Uninitialized!");

        auto initialEpoch = m_pMission->GetInitialEpoch();
        auto terminationEpoch = m_pMission->GetTerminationEpoch();
        string initialEpochStr = FormTimeStr(initialEpoch.Year(),initialEpoch.Mon(),initialEpoch.Day(),
                                             initialEpoch.Hour(),initialEpoch.Min(),initialEpoch.Sec());
        string intervalEpochStr = FormTimeIntervalStr(initialEpoch.Year(),initialEpoch.Mon(),initialEpoch.Day(),
                                                 initialEpoch.Hour(),initialEpoch.Min(),initialEpoch.Sec(),
                                                 terminationEpoch.Year(),terminationEpoch.Mon(),terminationEpoch.Day(),
                                                 terminationEpoch.Hour(),terminationEpoch.Min(),terminationEpoch.Sec());
        // Head
        json *pJhead = new json();
        (*pJhead)["id"] = "document";
        (*pJhead)["name"] = "DataFile";
        (*pJhead)["version"] = "1.0";
        (*pJhead)["clock"]["interval"] = intervalEpochStr;
        (*pJhead)["clock"]["currentTime"] = initialEpochStr;
        (*pJhead)["clock"]["multiplier"] = 60;
        (*pJhead)["clock"]["range"] = "LOOP_STOP";
        (*pJhead)["clock"]["step"] = "SYSTEM_CLOCK_MULTIPLIER";
        m_pJsonList->push_back(*pJhead);
        //
        int colorStep = int(255/(m_pMission->GetSpaceVehicleNumber()+1));
        int vehiclCount = 0;
        for(auto pVehicl:m_pMission->GetSpaceVehicleList())
        {
            ++vehiclCount;
            int bias = colorStep * vehiclCount;
            json *pJvehicl = new json();
            string name = pVehicl->GetName();

            (*pJvehicl)["id"] = "Satellite/" + name;
            (*pJvehicl)["name"] = name;
            (*pJvehicl)["availability"] = intervalEpochStr;
            (*pJvehicl)["description"] = "<!--HTML-->\r\n<p>  Test Satellite  </p>";

            (*pJvehicl)["billboard"]["horizontalOrigin"] = "CENTER";
            (*pJvehicl)["billboard"]["verticalOrigin"] = "CENTER";
            (*pJvehicl)["billboard"]["image"] = "data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABAAAAAQCAYAAAAf8/9hAAAAAXNSR0IArs4c6QAAAARnQU1BAACxjwv8YQUAAAAJcEhZcwAADsMAAA7DAcdvqGQAAADJSURBVDhPnZHRDcMgEEMZjVEYpaNklIzSEfLfD4qNnXAJSFWfhO7w2Zc0Tf9QG2rXrEzSUeZLOGm47WoH95x3Hl3jEgilvDgsOQUTqsNl68ezEwn1vae6lceSEEYvvWNT/Rxc4CXQNGadho1NXoJ+9iaqc2xi2xbt23PJCDIB6TQjOC6Bho/sDy3fBQT8PrVhibU7yBFcEPaRxOoeTwbwByCOYf9VGp1BYI1BA+EeHhmfzKbBoJEQwn1yzUZtyspIQUha85MpkNIXB7GizqDEECsAAAAASUVORK5CYII=";
            (*pJvehicl)["billboard"]["scale"] = 1;
            (*pJvehicl)["billboard"]["show"] = true;

            (*pJvehicl)["label"]["text"] = name;
            (*pJvehicl)["label"]["horizontalOrigin"] ="LEFT";
            (*pJvehicl)["label"]["verticalOrigin"] ="CENTER";
            (*pJvehicl)["label"]["fillColor"]["rgba"] = {bias , 255 - bias, 0, 255};
            (*pJvehicl)["label"]["font"] ="11pt Lucida Console";
            (*pJvehicl)["label"]["pixelOffset"]["cartesian2"] = {6, -4};
            (*pJvehicl)["label"]["show"] = true;

            vector<json> showTimePeriodList;
            json showTimePeriod;
            showTimePeriod["interval"] = intervalEpochStr;
            showTimePeriod["boolean"] = true;
            showTimePeriodList.push_back(showTimePeriod);
            (*pJvehicl)["path"]["show"]= showTimePeriodList;
            (*pJvehicl)["path"]["width"] = 1;
            (*pJvehicl)["path"]["material"]["solidColor"]["color"]["rgba"] = {bias , 255 - bias, 0, 255};

            (*pJvehicl)["path"]["resolution"] = 120;

            OrbitElem elem;
            CartToOrbitElem (pVehicl->GetCartState(), GM_Earth, elem);
            double T = 2*PI*sqrt(pow(elem.SMajAx(),3)/GM_Earth);

            vector<json> leadTimeList;
            json leadTimePeriod;
            leadTimePeriod["interval"] = intervalEpochStr;
            leadTimePeriod["epoch"] = initialEpochStr;
            leadTimePeriod["number"] = {0, T , T, 0};
            leadTimeList.push_back(leadTimePeriod);
            (*pJvehicl)["path"]["leadTime"] = leadTimeList;

            vector<json> trailTimeList;
            json trailTimePeriod;
            trailTimePeriod["interval"] = intervalEpochStr;
            trailTimePeriod["epoch"] = initialEpochStr;
            trailTimePeriod["number"] = {T};
            trailTimeList.push_back(trailTimePeriod);
            (*pJvehicl)["path"]["trailTime"] = trailTimeList;

            (*pJvehicl)["position"]["interpolationAlgorithm"] = "LAGRANGE";
            (*pJvehicl)["position"]["interpolationDegree"] = 5;
            (*pJvehicl)["position"]["referenceFrame"] = "INERTIAL";
            (*pJvehicl)["position"]["epoch"] = initialEpochStr;
            auto pProcessDataMap = m_pMission->GetProcessDataMap();
            auto pVehiclProcessData = pProcessDataMap->find(name)->second;
            double initialMjd = (*pVehiclProcessData)[0][0];
            vector<double> processData;
            for (auto pData:(*pVehiclProcessData))
            {
                for (int i = 0; i < 4; ++i)
                {
                    if (i == 0)
                    {
                        processData.push_back((pData[i] - initialMjd)*DayToSec);
                    }
                    else
                        processData.push_back(pData[i]);
                }
            }
            (*pJvehicl)["position"]["cartesian"] = processData;

            m_pJsonList->push_back(*pJvehicl);
        }

        m_pJsonToWirte = new json(*m_pJsonList);
        ofstream o(m_FilePath);
        o << setw(4);
        o << (*m_pJsonToWirte) << endl;
        o.close();

    }

    string CZMLScript::FormTimeStr(int year, int month, int day, int hour, int min, double sec)
    {
        string monStr;
        string dayStr;
        string hourStr;
        string minStr;
        string secStr;
        if (month < 10)
            monStr = "0" + to_string(month);
        else
            monStr = to_string(month);

        if (day < 10)
            dayStr = "0" + to_string(day);
        else
            dayStr = to_string(day);

        if (hour < 10)
            hourStr = "0" + to_string(hour);
        else
            hourStr = to_string(hour);

        if (min < 10)
            minStr = "0" + to_string(min);
        else
            minStr = to_string(min);

        if (sec < 10)
            secStr = "0" + to_string(int(sec));
        else
            secStr = to_string(int(sec));
        string ISO8601 = to_string(year)+ "-" +monStr + "-" +dayStr + "T" +
                        hourStr+ ":" +minStr + ":" +secStr + "Z";

        return ISO8601;
    }

    string CZMLScript::FormTimeIntervalStr(int year0, int month0, int day0, int hour0, int min0, double sec0,
                                              int year1, int month1, int day1, int hour1, int min1, double sec1)
    {
        string firstStr = FormTimeStr(year0, month0, day0, hour0, min0, sec0);
        string sceondStr = FormTimeStr(year1, month1, day1,  hour1, min1, sec1);
        string ISO8601 = firstStr+ "/" + sceondStr;

        return ISO8601;
    }
}
