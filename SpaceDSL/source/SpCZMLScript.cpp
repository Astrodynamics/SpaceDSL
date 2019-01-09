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
#include <SpaceDSL/SpFacility.h>
#include <SpaceDSL/SpTarget.h>
#include <SpaceDSL/SpSpaceVehicle.h>
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

        map<string, json> vehicleColorMap;
        map<string, json> facilityColorMap;
        map<string, json> targetColorMap;
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

        // Space Vehicle Data
        int colorStep = int(255/(m_pMission->GetSpaceVehicleNumber()+1));
        int vehiclCount = 0;
        for(auto pVehicl:m_pMission->GetSpaceVehicleList())
        {
            ++vehiclCount;
            int bias = colorStep * vehiclCount;
            json *pJvehicl = new json();
            string name = pVehicl->GetName();
            json Jcolor;
            Jcolor["rgba"] = {bias , 255 - bias, 0, 255};
            vehicleColorMap.insert(pair<string, json>(name, Jcolor));

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
            (*pJvehicl)["label"]["fillColor"] = Jcolor;
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

            double T = m_pMission->GetAverageOrbitalPeriod(name);

            (*pJvehicl)["path"]["leadTime"] = T;
            (*pJvehicl)["path"]["trailTime"] = 0.0;
            (*pJvehicl)["position"]["interpolationAlgorithm"] = "LAGRANGE";
            (*pJvehicl)["position"]["interpolationDegree"] = 5;
            (*pJvehicl)["position"]["referenceFrame"] = "INERTIAL";
            (*pJvehicl)["position"]["epoch"] = initialEpochStr;
            auto pProcessDataMap = m_pMission->GetProcessDataMap();
            auto iter = pProcessDataMap->find(pVehicl);
            if (iter == pProcessDataMap->end())
                throw SPException(__FILE__, __FUNCTION__, __LINE__,
                          "MissionThread::SaveProcessDataLine Cant Find ProcessData!");

            auto pProcessDataList = iter->second;
            double initialMjd = (*pProcessDataList)[0][0];
            vector<double> processData;
            for (auto pData:(*pProcessDataList))
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
        // Facility Data
        colorStep = int(255/(m_pMission->GetFacilityNumber()+1));
        int facilityCount = 0;
        for(auto pFacility:m_pMission->GetFacilityList())
        {
            ++facilityCount;
            int bias = colorStep * facilityCount;
            json *pJfacility = new json();
            string name = pFacility->GetName();

            json Jcolor;
            Jcolor["rgba"] = {255 , 255 - bias, bias , 255};
            facilityColorMap.insert(pair<string, json>(name, Jcolor));

            (*pJfacility)["id"] = "Facility/" + name;
            (*pJfacility)["name"] = name;
            (*pJfacility)["availability"] = intervalEpochStr;
            (*pJfacility)["description"] = "<!--HTML-->\r\n<p>  Test Facility  </p>";

            (*pJfacility)["billboard"]["horizontalOrigin"] = "CENTER";
            (*pJfacility)["billboard"]["verticalOrigin"] = "CENTER";
            (*pJfacility)["billboard"]["image"] = "data:image/bmp;base64,Qk04BQAAAAAAADYEAAAoAAAAEAAAABAAAAABAAgAAAAAAAIBAADDDgAAww4AAAAAAAAAAAAAAAAAAAAAgAAAgAAAAICAAIAAAACAAIAAgIAAAMDAwADA3MAA8MqmAAAgQAAAIGAAACCAAAAgoAAAIMAAACDgAABAAAAAQCAAAEBAAABAYAAAQIAAAECgAABAwAAAQOAAAGAAAABgIAAAYEAAAGBgAABggAAAYKAAAGDAAABg4AAAgAAAAIAgAACAQAAAgGAAAICAAACAoAAAgMAAAIDgAACgAAAAoCAAAKBAAACgYAAAoIAAAKCgAACgwAAAoOAAAMAAAADAIAAAwEAAAMBgAADAgAAAwKAAAMDAAADA4AAA4AAAAOAgAADgQAAA4GAAAOCAAADgoAAA4MAAAODgAEAAAABAACAAQABAAEAAYABAAIAAQACgAEAAwABAAOAAQCAAAEAgIABAIEAAQCBgAEAggABAIKAAQCDAAEAg4ABAQAAAQEAgAEBAQABAQGAAQECAAEBAoABAQMAAQEDgAEBgAABAYCAAQGBAAEBgYABAYIAAQGCgAEBgwABAYOAAQIAAAECAIABAgEAAQIBgAECAgABAgKAAQIDAAECA4ABAoAAAQKAgAECgQABAoGAAQKCAAECgoABAoMAAQKDgAEDAAABAwCAAQMBAAEDAYABAwIAAQMCgAEDAwABAwOAAQOAAAEDgIABA4EAAQOBgAEDggABA4KAAQODAAEDg4ACAAAAAgAAgAIAAQACAAGAAgACAAIAAoACAAMAAgADgAIAgAACAICAAgCBAAIAgYACAIIAAgCCgAIAgwACAIOAAgEAAAIBAIACAQEAAgEBgAIBAgACAQKAAgEDAAIBA4ACAYAAAgGAgAIBgQACAYGAAgGCAAIBgoACAYMAAgGDgAICAAACAgCAAgIBAAICAYACAgIAAgICgAICAwACAgOAAgKAAAICgIACAoEAAgKBgAICggACAoKAAgKDAAICg4ACAwAAAgMAgAIDAQACAwGAAgMCAAIDAoACAwMAAgMDgAIDgAACA4CAAgOBAAIDgYACA4IAAgOCgAIDgwACA4OAAwAAAAMAAIADAAEAAwABgAMAAgADAAKAAwADAAMAA4ADAIAAAwCAgAMAgQADAIGAAwCCAAMAgoADAIMAAwCDgAMBAAADAQCAAwEBAAMBAYADAQIAAwECgAMBAwADAQOAAwGAAAMBgIADAYEAAwGBgAMBggADAYKAAwGDAAMBg4ADAgAAAwIAgAMCAQADAgGAAwICAAMCAoADAgMAAwIDgAMCgAADAoCAAwKBAAMCgYADAoIAAwKCgAMCgwADAoOAAwMAAAMDAIADAwEAAwMBgAMDAgADAwKAA8Pv/AKSgoACAgIAAAAD/AAD/AAAA//8A/wAAAP8A/wD//wAA////AAdSLi4cNzcmLlIdHR0dHVIHUi4uHDc3Ji5SHR0dHR1SB1IuLhw3NyYuUh0dHR0dUgdSLi4cHBwmLlImHR0dHVIHUi4uJhwcJi5SJiYdHR1SB1JSLi4uLi4uUiYmJh1SUgcHB1JSLi4uLlImJlJSBwcHB1L//1JSLi5SUlL391IHBwdS/wf/B1JSUgf39/dSBwcHUv///wcHBwcHB/f3UgcHB1IH/wf//wcH9wcH91IHBwdS/////wf/BwcH9wdSBwcHB1L/B/////cHBwdSBwcHBwdS////B///B/cHUgcHBwcHB1JS////BwdSUgcHBwcHBwcHB1JSUlJSBwcHBwcAAA==";
            (*pJfacility)["billboard"]["scale"] = 1;
            (*pJfacility)["billboard"]["show"] = true;

            (*pJfacility)["label"]["text"] = name;
            (*pJfacility)["label"]["horizontalOrigin"] ="LEFT";
            (*pJfacility)["label"]["verticalOrigin"] ="CENTER";
            (*pJfacility)["label"]["fillColor"] = Jcolor;
            (*pJfacility)["label"]["font"] ="11pt Lucida Console";
            (*pJfacility)["label"]["pixelOffset"]["cartesian2"] = {6, -4};
            (*pJfacility)["label"]["show"] = true;

            (*pJfacility)["position"]["cartesian"] = pFacility->GetGeodeticPos();

            m_pJsonList->push_back(*pJfacility);
        }
        // Target Data
        int targetWithoutFacilityNum = m_pMission->GetTargetNumber() - m_pMission->GetFacilityNumber();
        colorStep = int(255/(targetWithoutFacilityNum + 1));
        int targetWithoutFacilityCount = 0;
        for(auto pTarget:m_pMission->GetTargetList())
        {
            auto targetType = pTarget->GetTargetType();
            if ( targetType == Target::E_Facility)
                continue;

            ++targetWithoutFacilityCount;
            int bias = colorStep * targetWithoutFacilityCount;
            json *pJtarget = new json();
            string name = pTarget->GetName();
            json Jcolor;
            Jcolor["rgba"] = {255 , 255 - bias, bias , 255};
            targetColorMap.insert(pair<string, json>(name, Jcolor));

            switch (targetType)
            {
            case Target::E_NotDefindTargetType:
                throw SPException(__FILE__, __FUNCTION__, __LINE__, "CZMLScript::WirteCZML TargetType = E_NotDefinedTargetType!");
            case Target::E_PointTarget:
                (*pJtarget)["id"] = "Target/" + name;
                (*pJtarget)["name"] = name;
                (*pJtarget)["availability"] = intervalEpochStr;
                (*pJtarget)["description"] = "<!--HTML-->\r\n<p>  Test Target  </p>";

                (*pJtarget)["billboard"]["horizontalOrigin"] = "CENTER";
                (*pJtarget)["billboard"]["verticalOrigin"] = "CENTER";
                (*pJtarget)["billboard"]["image"] = "data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABAAAAAQCAYAAAAf8/9hAAAAAXNSR0IArs4c6QAAAARnQU1BAACxjwv8YQUAAAAJcEhZcwAADsMAAA7DAcdvqGQAAACVSURBVDhPnZHBDYQwDARTmkuhNEqhFErgzwNslA2OWUfASCsFez3S6QpDRA6Wus5BcZVCMxSNDmMeki/HSCexj8VLJjkiNqMCe8y6bAJyDKgkCsA6L3dZ36DNNGOBK2ZzLti3R/GaV/wsF2h+/YRLom9IIq/+iUySHhsQdBJXZukEBpOw0GNgCy+Kwb7Wc1CMqWtHKScCcvdPa1WrSwAAAABJRU5ErkJggg==";
                (*pJtarget)["billboard"]["scale"] = 1;
                (*pJtarget)["billboard"]["show"] = true;

                (*pJtarget)["label"]["text"] = name;
                (*pJtarget)["label"]["horizontalOrigin"] ="LEFT";
                (*pJtarget)["label"]["verticalOrigin"] ="CENTER";
                (*pJtarget)["label"]["fillColor"] = Jcolor;
                (*pJtarget)["label"]["font"] ="11pt Lucida Console";
                (*pJtarget)["label"]["pixelOffset"]["cartesian2"] = {6, -4};
                (*pJtarget)["label"]["show"] = true;

                (*pJtarget)["position"]["cartesian"] = static_cast<PointTarget *>(pTarget)->GetGeodeticPos();

                m_pJsonList->push_back(*pJtarget);
                break;
            case Target::E_LineTarget:
                // Support Later
                break;
            case Target::E_AreaTarget:
                // Support Later
                break;
            default:
                throw SPException(__FILE__, __FUNCTION__, __LINE__, "CalObservation: Just Support Point Target!");
            }

        }
        // Access Data
        json *pJaccessParent = new json();
        (*pJaccessParent)["id"] = "AccessParent";
        (*pJaccessParent)["name"] = "Accesses";
        (*pJaccessParent)["description"] = "List of Accesses";
        m_pJsonList->push_back(*pJaccessParent);

        int accessCount = 0;
        colorStep = int(255/(m_pMission->GetSpaceVehicleNumber() + 1));
        for(auto iterAccess = m_pMission->GetAccessData()->begin();
            iterAccess != m_pMission->GetAccessData()->end();
            ++iterAccess)
        {
            ++accessCount;
            json *pJaccess = new json();

            string targetName = iterAccess->first.first->GetName();
            string vehicleName = iterAccess->first.second->GetName();
            auto targetType = iterAccess->first.first->GetTargetType();
            string id;
            vector<string> refList;
            switch (targetType)
            {
            case Target::E_NotDefindTargetType:
                throw SPException(__FILE__, __FUNCTION__, __LINE__, "CZMLScript::WirteCZML TargetType = E_NotDefinedTargetType!");
            case Target::E_Facility:
                refList.clear();
                id = "Facility/" + targetName + "-to-Satellite/" + vehicleName;
                refList.push_back("Facility/" + targetName + "#position");
                refList.push_back("Satellite/" + vehicleName+ "#position");
                break;
            case Target::E_PointTarget:
                refList.clear();
                id = "Target/" + targetName + "-to-Satellite/" + vehicleName;
                refList.push_back("Target/" + targetName + "#position");
                refList.push_back("Satellite/" + vehicleName+ "#position");
                break;
            case Target::E_LineTarget:
                refList.clear();
                id = "LineTarget/" + targetName + "-to-Satellite/" + vehicleName;
                refList.push_back("LineTarget/" + targetName + "#position");
                refList.push_back("Satellite/" + vehicleName+ "#position");
                break;
            case Target::E_AreaTarget:
                refList.clear();
                id = "AreaTarget/" + targetName + "-to-Satellite/" + vehicleName;
                refList.push_back("AreaTarget/" + targetName + "#position");
                refList.push_back("Satellite/" + vehicleName+ "#position");
                break;
            default:
                throw SPException(__FILE__, __FUNCTION__, __LINE__, "CalObservation: Just Support Point Target!");
            }

            string name =  targetName + " to " + vehicleName;

            vector<string> availabilityList;
            vector<json> showJsonList;

            json jtimePairFirst;
            auto firstPair = iterAccess->second[0];
            jtimePairFirst["interval"] = "0000-01-01T00:00:00Z/"+
                                        FormTimeStr(firstPair.first.Year(),firstPair.first.Mon(),firstPair.first.Day(),
                                                    firstPair.first.Hour(),firstPair.first.Min(),firstPair.first.Sec());
            jtimePairFirst["boolean"] = false;
            showJsonList.push_back(jtimePairFirst);

            int pairCount = 0;
            for(auto &timePair:iterAccess->second)
            {
                string intervalStrTrue = FormTimeIntervalStr(timePair.first.Year(),timePair.first.Mon(),timePair.first.Day(),
                                                         timePair.first.Hour(),timePair.first.Min(),timePair.first.Sec(),
                                                         timePair.second.Year(),timePair.second.Mon(),timePair.second.Day(),
                                                         timePair.second.Hour(),timePair.second.Min(),timePair.second.Sec());
                availabilityList.push_back(intervalStrTrue);
                json jtimePairTrue;
                jtimePairTrue["interval"] = intervalStrTrue;
                jtimePairTrue["boolean"] = true;
                showJsonList.push_back(jtimePairTrue);
                ++pairCount;

                if (pairCount < iterAccess->second.size())
                {
                    auto laterPair = iterAccess->second[pairCount];
                    string intervalStrFalse = FormTimeIntervalStr(timePair.second.Year(),timePair.second.Mon(),timePair.second.Day(),
                                                             timePair.second.Hour(),timePair.second.Min(),timePair.second.Sec(),
                                                             laterPair.first.Year(),laterPair.first.Mon(),laterPair.first.Day(),
                                                             laterPair.first.Hour(),laterPair.first.Min(),laterPair.first.Sec());
                    json jtimePairFalse;
                    jtimePairFalse["interval"] = intervalStrFalse;
                    jtimePairFalse["boolean"] = false;
                    showJsonList.push_back(jtimePairFalse);
                }
            }
            auto lastPair = iterAccess->second[pairCount-1];
            jtimePairFirst["interval"] = FormTimeStr(lastPair.second.Year(),lastPair.second.Mon(),lastPair.second.Day(),
                                                    lastPair.second.Hour(),lastPair.second.Min(),lastPair.second.Sec()) +
                                        "/9999-12-31T24:00:00Z";
            jtimePairFirst["boolean"] = false;
            showJsonList.push_back(jtimePairFirst);

            (*pJaccess)["id"] = id;
            (*pJaccess)["name"] = name;
            (*pJaccess)["parent"] = "AccessParent";
            (*pJaccess)["availability"] = availabilityList;
            (*pJaccess)["polyline"]["show"] = showJsonList;
            (*pJaccess)["polyline"]["width"] = 1;
            (*pJaccess)["polyline"]["followSurface"] = false;
            (*pJaccess)["polyline"]["positions"]["references"] = refList;
            auto iterColor = vehicleColorMap.find(vehicleName);
            if (iterColor == vehicleColorMap.end())
                throw SPException(__FILE__, __FUNCTION__, __LINE__, "CZMLScript::WirteCZML vehicleColorMap.find(vehicleName) = NULL");
            (*pJaccess)["polyline"]["material"]["solidColor"]["color"] = iterColor->second;

            m_pJsonList->push_back(*pJaccess);

        }


        // Save To Json File
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
