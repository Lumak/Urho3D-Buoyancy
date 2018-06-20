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

#include <Urho3D/Scene/LogicComponent.h>

using namespace Urho3D;

namespace Urho3D
{
class RigidBody;
class CollisionShape;
}

//=============================================================================
//=============================================================================
class BuoyCol
{
    enum PointSize{ MaxPoints = 8 };
public:

    BuoyCol();
    virtual ~BuoyCol(){}

    void SetupBuoyShapeData(RigidBody *_rbody, CollisionShape *_colShape);
    void UpdateColPoints(const BoundingBox &waterBbox);
    void SetFrameUpdated(bool updated)  { frameUpdate = updated; }
    bool IsFrameUpdated() const         { return frameUpdate;    }

protected:
    void SetContainerSize(unsigned csize);
    void ConfigSphereData();
    void ConfigBboxPointData();
    void UpdateBuoyPoints(const BoundingBox &waterBbox);
    void GetLocalAabb(Vector3 &mmin, Vector3 &mmax);
    float GetShapeVolume(const Vector3 &shapeSize) const;

public:
    RigidBody          *rbody;
    CollisionShape     *colShape;
   
    // points setup 
    Vector<Vector3>    localPointList;
    float              pointRadius;             // radius of each point on the bbox
    float              distributedVolumePerPt;  // point volume proportinal to the whole volume
    float              pointMaxSphVol;          // point's max sphere volume when completely submerged

    // runtime update vars
    Vector<Vector3>    worldPointList;          // transformed in world space
    Vector<float>      pointHeightList;         // point height above water
    Vector<float>      partialVolList;          // point volume
    float              totalVol;                // total volume

    bool               frameUpdate;
    Timer              updateTimer;
};

//=============================================================================
//=============================================================================
class WaterVolume : public LogicComponent
{
    URHO3D_OBJECT(WaterVolume, LogicComponent);

public:
    WaterVolume(Context *context);
    virtual ~WaterVolume(){}

    static void RegisterObject(Context* context);

protected:
    virtual void Start();
    virtual void FixedUpdate(float timeStep);
    float CalcSubmergedVolume(BuoyCol &buoyCol);

    void RemoveExpired();
    int GetBodyListIdx(RigidBody *rbody) const;

    void HandleWaterCollisionEvent(StringHash eventType, VariantMap& eventData);

protected:
    Vector<BuoyCol> buoyColList_;

    BoundingBox     waterBbox_;
    float           waterDensity_;
    float           verticalViscosity_;
    float           horizontalViscosity_;
    float           angularViscosity_;
};


