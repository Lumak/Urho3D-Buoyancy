//
// Copyright (c) 2008-2016 the Urho3D project.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
#pragma once
#include <Urho3D/Math/Vector3.h>
#include <Urho3D/Math/Quaternion.h>

using namespace Urho3D;

//=============================================================================
//=============================================================================
Vector3 SmoothStep(const Vector3 &p1, const Vector3 &p2, float t, float tolerance=0.001f);
Quaternion SmoothStepAngle(const Quaternion &p1, const Quaternion &p2, float t, float tolerance=0.001f);

// spring damp
float SpringDamping(float curDist, float desiredDist, float &velocity, float damping, float maxVel, float timestep);
Vector3 SpringDampingV3(const Vector3& curDist, const Vector3& desiredDist, Vector3& velocity, float damping, float maxVel, float t);

Vector3 LerpTowards(const Vector3 &from, const Vector3 &to, float gradualLerpRate, float timeStep);






