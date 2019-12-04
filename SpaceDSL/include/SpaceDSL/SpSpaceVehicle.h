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
* Date:2018-07-30
* Description:
*   SpSpaceVehicle.h
*
*   Purpose:
*
*       The Base Class of All Kinds of Space Vehicle.
*
*
*   Last modified:
*
*   2018-07-30  Niu Zhiyong (1st edition)
*
*************************************************************************/

#ifndef SPSPACEVEHICLE_H
#define SPSPACEVEHICLE_H

#include "SpaceDSL_Global.h"
#include "SpOrbitParam.h"
#include "SpSensor.h"
#include "SpTimeSystem.h"

#include <string>
#include <atomic>

using namespace std;
/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {
	
    /*************************************************
     * Class type: The Base Class of All Kinds of Space Vehicle
     * Author: Niu ZhiYong
     * Date:2018-07-30
     * Description:
    **************************************************/
    class SPACEDSL_API SpaceVehicle
	{
	public:
        explicit SpaceVehicle();

        SpaceVehicle(const string &name, const CalendarTime& initialEpochDate,
                     const CartState& initialState, const double initialMass,
                     const double dragCoef, const double dragArea,
                     const double SRPCoef, const double SRPArea);

        SpaceVehicle(const string &name, const double initialEpochMjd,
                     const CartState& initialState, const double initialMass,
                     const double dragCoef, const double dragArea,
                     const double SRPCoef, const double SRPArea);

		virtual ~SpaceVehicle();
		
	public:
        inline void		SetName (const string &name)                    { m_Name = name; }

        inline void		SetInitialCartState (const CartState& state)    { m_InitialCartState = state; }

        inline void		SetInitialEpoch (const CalendarTime& time)      { m_InitialEpoch = CalendarTimeToMjd(time); }

        inline void		SetInitialEpoch (const double Mjd)              { m_InitialEpoch = Mjd; }

        inline void		SetInitialMass (const double mass)              { m_InitialMass = mass; }

        inline void		SetCartState (const CartState& state)           { m_CartState = state; }

        inline void		SetTime (const CalendarTime& time)              { m_Epoch = CalendarTimeToMjd(time); }

        inline void		SetTime (const double Mjd)                      { m_Epoch = Mjd; }

        inline void		SetMass (const double mass)                     { m_Mass = mass; }

        inline void     SetDragCoef(const double dragCoef)              { m_DragCoef = dragCoef; }

        inline void     SetDragArea(const double dragArea)              { m_DragArea = dragArea; }

        inline void     SetSRPCoef(const double SRPCoef)                { m_SRPCoef = SRPCoef; }

        inline void     SetSRPArea(const double SRPArea)                { m_SRPArea = SRPArea; }

        inline int              GetID() const                       { return m_ID; }

        inline const string&    GetName() const                     { return m_Name; }

        inline const CartState& GetInitialCartState() const         { return m_InitialCartState; }

        inline double           GetInitialEpoch() const             { return m_InitialEpoch; }

        inline double           GetInitialMass() const              { return m_InitialMass; }

        inline const CartState& GetCartState() const                { return m_CartState; }

        inline double           GetEpoch() const                    { return m_Epoch; }

        inline double           GetMass() const                     { return m_Mass; }

        inline double           GetDragCoef() const                 { return m_DragCoef; }

        inline double           GetDragArea() const                 { return m_DragArea; }

        inline double           GetSRPCoef()  const                 { return m_SRPCoef; }

        inline double           GetSRPArea()  const                 { return m_SRPArea; }

        void                    UpdateState(const double Mjd, const CartState& state, const double mass);

        void                    UpdateState(const double Mjd, const Vector3d& pos, const Vector3d& vel,const double mass);

        void                    Reset();

        /********************************************************************/
        /// Insert/Remove a Sensor Belong to The Space Vehicle
        /// @Author     Niu Zhiyong
        /// @Date       2018-12-26
        /// @Input
        /// @Param  halfAngle1    	Cone Half Angle/Horizontal Half Angle
        /// @Param	halfAngle2		Vertical Half Angle
        /// @Output
        /// @Param
        /**********************************************************************/
        void InsertSensor(const string &name, const Sensor::SensorType type, const double halfAngle1, const double halfAngle2 = 0);

        bool RemoveSensor(const int id);

	//
	// Attribute.
	//
	protected:
        static atomic<int>      VehicleID;          ///< Aircraft Global Counter
        int                     m_ID;               ///< Aircraft ID
        string                  m_Name;				///< Aircraft name
        double                  m_InitialEpoch;		///< Initial MJD Epoch of Aircraft
        double                  m_InitialMass;		///< Initial Mass of Aircraft
        CartState               m_InitialCartState;	///< Initial State of Aircraft

        double                  m_Epoch;			///< MJD Epoch of Aircraft
        double                  m_Mass;				///< Mass of Aircraft at the Epoch
        CartState               m_CartState;		///< State of Aircraft at the Epoch

        //Atmosphere Drag Parameters
        double                  m_DragCoef;			///< drag coefficient
        double                  m_DragArea;         ///< drag term m2/kg

        //Solar Radiation Parameters
        double                  m_SRPCoef;          ///< Solar Radiation Pressure Coeff
        double                  m_SRPArea;          ///< Area for SRP

        //Sensor on Space Vehicle
        int                     m_SensorNumber;
        vector<Sensor *>        m_SensorList;      

    };
	
}

#endif //SPSPACEVEHICLE_H
