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
#include "SpTimeSystem.h"

#include <string>

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
		SpaceVehicle();
		virtual ~SpaceVehicle();
		
	public:
        inline void		SetName (const string &name)                    { m_Name = name; }

        inline void		SetInitialCartState (const CartState& state)    { m_InitialCartState = state; }
        inline void		SetInitialEpoch (const CalendarTime& time)      { m_InitialEpoch = time; }
        inline void		SetInitialMass (double mass)                    { m_InitialMass = mass; }

        inline void		SetCartState (const CartState& state)           { m_CartState = state; }
        inline void		SetTime (const CalendarTime& time)              { m_Epoch = time; }
        inline void		SetMass (double mass)                           { m_Mass = mass; }


        inline const string&        GetName() const                     { return m_Name; }
        inline const CartState&		GetInitialCartState() const         { return m_InitialCartState; }
        inline const CalendarTime&	GetInitialEpoch() const             { return m_InitialEpoch; }
        inline double				GetInitialMass() const              { return m_InitialMass; }

        inline const CartState&		GetCartState() const                { return m_CartState; }
        inline const CalendarTime&	GetEpoch() const                    { return m_Epoch; }
        inline double				GetMass() const                     { return m_Mass; }

	//
	// Attribute.
	//
	protected:
        string          m_Name;				///< Aircraft ame
        CalendarTime    m_InitialEpoch;		///< Initial Epoch of Aircraft
        double			m_InitialMass;		///< Initial Mass of Aircraft
        CartState		m_InitialCartState;	///< Initial State of Aircraft

        CalendarTime    m_Epoch;			///< Epoch of Aircraft
        double			m_Mass;				///< Mass of Aircraft at the Epoch
        CartState		m_CartState;		///< State of Aircraft at the Epoch

    };
	
}

#endif //SPSPACEVEHICLE_H
