/************************************************************************
* Copyright (C) 2017 Niu ZhiYong
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
* Date:2018-03-20
* Description:
*   SpOrbitParam.cpp
*
*   Purpose:
*
*       Related Classes and Functions of Orbital Parameters
*
*
*   Last modified:
*
*   2018-03-20  Niu Zhiyong (1st edition)
*
*************************************************************************/

#include "SpaceDSL/SpOrbitParam.h"
#include "SpaceDSL/SpUtils.h"

using namespace SpaceDSL;
/*************************************************
 * Class type: Cartesian State Elements
 * Author: Niu ZhiYong
 * Date:2018-03-20
 * Description:
 *  Position and velocity in cartesian coordinates
 *  This Class is Thread Safe!
**************************************************/
CartState::CartState()
{
    this->m_Pos.fill(0);
    this->m_Vel.fill(0);
}

CartState::CartState(const Vector3d &pos, const Vector3d &vel)
{
    this->m_Pos = pos;
    this->m_Vel = vel;
}

CartState::CartState(double xPos, double yPos, double zPos, double xVel, double yVel, double zVel)
{
    this->m_Pos[0] = xPos;
    this->m_Pos[1] = yPos;
    this->m_Pos[2] = zPos;

    this->m_Vel[0] = xVel;
    this->m_Vel[1] = yVel;
    this->m_Vel[2] = zVel;

}

CartState::~CartState()
{

}

const CartState CartState::operator -() const
{
    CartState tempState(-m_Pos, -m_Vel);

    return tempState;
}

const CartState CartState::operator +(const CartState &state) const
{
    CartState tempState(m_Pos + state.Pos(), m_Vel + state.Vel());

    return tempState;
}

const CartState CartState::operator -(const CartState &state) const
{
    CartState tempState(m_Pos - state.Pos(), m_Vel - state.Vel());

    return tempState;
}

const CartState &CartState::operator+=(const CartState &state)
{
    m_Pos += state.Pos();
    m_Vel += state.Vel();
    return *this;
}

const CartState &CartState::operator-=(const CartState &state)
{
    m_Pos -= state.Pos();
    m_Vel -= state.Vel();
    return *this;
}
/*************************************************
 * Class type: Classic Orbit Element
 * Author: Niu ZhiYong
 * Date:2018-03-20
 *  This Class is Thread Safe!
**************************************************/
OrbitElem::OrbitElem()
{
    this->m_SMajAx = 0;
    this->m_Ecc = 0;
    this->m_I = 0;
    this->m_RAAN = 0;
    this->m_ArgPeri = 0;
    this->m_TrueA = 0;
}

OrbitElem::OrbitElem(double sMajAx, double ecc, double i, double raan, double argPeri, double trueA)
{
    this->m_SMajAx = sMajAx;
    this->m_Ecc = ecc;
    this->m_I = i;
    this->m_RAAN = raan;
    this->m_ArgPeri = argPeri;
    this->m_TrueA = trueA;
}

OrbitElem::~OrbitElem()
{

}
/*************************************************
 * Class type: Modified Orbital Elements
 * Author: Niu ZhiYong
 * Date:2018-03-20
 * Description:
 *  This Class is Thread Safe!
**************************************************/
ModOrbElem::ModOrbElem()
{

}

ModOrbElem::ModOrbElem(double periRad, double ecc, double i, double raan, double argPeri, double trueA)
{

}

ModOrbElem::~ModOrbElem()
{

}

//====================================================================
//
// Grouping: Parameter Conversion
//
//====================================================================
double MeanAnomalyToEcc(double meanAnomaly, double eccentricity)
{
    return 0;
}

double TrueAnomalyToEcc(double trueAnomaly, double eccentricity)
{
    return 0;
}

double MeanAnomalyToTrue(double meanAnomaly, double eccentricity)
{
    return 0;
}

double TrueAnomalyToMean(double trueAnomaly, double eccentricity)
{
    return 0;
}

void CartToOrbitElem(const CartState &cart, double gm, OrbitElem &elem)
{

}

void OrbitElemToCart(const OrbitElem &elem, double gm, CartState &cart)
{

}


