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
#include <Urho3D/Core/Context.h>
#include <Urho3D/IO/MemoryBuffer.h>
#include <Urho3D/Physics/PhysicsEvents.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Physics/CollisionShape.h>
#include <Urho3D/Physics/PhysicsUtils.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Scene/SceneEvents.h>
#include <Bullet/BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <Bullet/BulletCollision/CollisionShapes/btCompoundShape.h>
#include <SDL/SDL_log.h>

#include "WaterVolume.h"
#include "CollisionLayer.h"

#include <Urho3D/DebugNew.h>
//=============================================================================
//=============================================================================
const float MPI = (float)M_PI; // msvc has its own M_PI def that's declared as a double, hence this assignment

//=============================================================================
//=============================================================================
void WaterVolume::RegisterObject(Context* context)
{
    context->RegisterFactory<WaterVolume>();
}

WaterVolume::WaterVolume(Context *context) 
    : LogicComponent(context)
    , waterDensity_(1.0f)
    , verticalViscosity_(0.05f)
    , horizontalViscosity_(0.05f)
    , angularViscosity_(0.01f)
{
    SetUpdateEventMask(USE_FIXEDUPDATE);
}

void WaterVolume::Start()
{
    // get water bbox
    waterBbox_ = GetNode()->GetComponent<CollisionShape>(true)->GetWorldBoundingBox();

    SubscribeToEvent(GetNode(), E_NODECOLLISION, URHO3D_HANDLER(WaterVolume, HandleWaterCollisionEvent));
}

void WaterVolume::FixedUpdate(float timeStep)
{
    for ( unsigned i = 0; i < buoyColList_.Size(); ++i )
    {
        BuoyCol &buoyCol = buoyColList_[i];

        if (buoyCol.IsFrameUpdated())
        {
            // apply viscosity
            RigidBody *rbody = buoyCol.rbody;
            Vector3 linVel = rbody->GetLinearVelocity();
            Vector3 angVel = rbody->GetAngularVelocity();
            linVel.x_ *= (1.0f - horizontalViscosity_);
            linVel.y_ *= (1.0f - verticalViscosity_);
            linVel.z_ *= (1.0f - horizontalViscosity_);
            rbody->SetLinearVelocity(linVel);
            rbody->SetAngularVelocity(angVel * (1.0f - angularViscosity_));

            // calc submerged volume
            float volume = CalcSubmergedVolume(buoyCol);

            if (volume > 0.0f)
            {
                // calc buoyant force
                Vector3 wgravity = GetScene()->GetComponent<PhysicsWorld>()->GetGravity() * -1.0f;
                Vector3 buoyantForce = waterDensity_ * wgravity * volume;

                // apply partial forces at relative positions
                Vector3 center = buoyCol.rbody->GetPosition();
                for ( unsigned j = 0; j < buoyCol.partialVolList.Size(); ++j )
                {
                    if (buoyCol.partialVolList[j] > 0.0f)
                    {
                        Vector3 partialBF = buoyantForce * (buoyCol.partialVolList[j]/buoyCol.totalVol);
                        Vector3 relPos = buoyCol.worldPointList[j] - center;

                        rbody->ApplyForce(partialBF, relPos);
                    }
                }
            }

            // clear status
            buoyCol.SetFrameUpdated(false);
        }
    }

    // remove expired
    RemoveExpired();
}

float WaterVolume::CalcSubmergedVolume(BuoyCol &buoyCol)
{
    // clear
    buoyCol.totalVol = 0.0f;

    for ( unsigned i = 0; i < buoyCol.pointHeightList.Size(); ++i )
    {
        // init partial vol
        buoyCol.partialVolList[i] = 0.0f;
        const float depth = buoyCol.pointRadius - buoyCol.pointHeightList[i];
        const float diameter = buoyCol.pointRadius * 2.0f;

        // completely submerged
        if (depth > (diameter - M_EPSILON))
        {
            buoyCol.partialVolList[i] = buoyCol.distributedVolumePerPt;
            buoyCol.totalVol += buoyCol.distributedVolumePerPt;
        }
        // partially submerged
        else if (depth > 0.0f)
        {
            // partial sphere vol eqn http://www.lmnoeng.com/Volume/CylConeSphere.php
            // y is the submerged height, and the volume eqn is what's submerged
            // Vol = pi/3 * y^2 *(1.5 * diameter - y)
            // where y = depth, then:
            // Vol = pi/3 *(depth*depth) * (1.5*diameter - depth)
            float partSubmrgSph = (MPI/3.0f) * (depth*depth) * (1.5f * diameter - depth);
            float partialVol = partSubmrgSph/buoyCol.pointMaxSphVol;
            buoyCol.partialVolList[i] = partialVol * buoyCol.distributedVolumePerPt;
            buoyCol.totalVol += buoyCol.partialVolList[i];
        }
    }

    return buoyCol.totalVol;
}

void WaterVolume::RemoveExpired()
{
    // five sec timer
    const unsigned expTime = 5000;
     
    // remove expired, could do this in the FixedUpdate loop but don't want a loop bloat
    for ( unsigned i = 0; i < buoyColList_.Size(); ++i )
    {
        BuoyCol &buoyCol = buoyColList_[i];

        if (buoyCol.updateTimer.GetMSec(false) > expTime)
        {
            buoyColList_.Erase(i--);
        }
    }
}

int WaterVolume::GetBodyListIdx(RigidBody *rbody) const
{
    int idx = -1;

    for ( unsigned i = 0; i < buoyColList_.Size(); ++i )
    {
        if (buoyColList_[i].rbody == rbody)
        {
            idx = (int)i;
            break;
        }
    }

    return idx;
}

void WaterVolume::HandleWaterCollisionEvent(StringHash eventType, VariantMap& eventData)
{
    using namespace NodeCollision;

    RigidBody *rbody = (RigidBody*)eventData[P_OTHERBODY].GetVoidPtr();

    // chars and projectiles have no buoyancy
    if (rbody->GetCollisionLayer() & (ColLayer_Character | ColLayer_Projectile))
    {
        return;
    }

    int idx;
    BuoyCol *pbuoyCol = NULL;

    // get or create rbody obj list
    if ((idx = GetBodyListIdx(rbody)) >= 0)
    {
        pbuoyCol = &buoyColList_.At(idx);
    }
    else
    {
        // create
        BuoyCol buoyCol;
        buoyColList_.Push(buoyCol);
        pbuoyCol = &buoyColList_.At(buoyColList_.Size() - 1);
        pbuoyCol->SetupBuoyShapeData(rbody, rbody->GetNode()->GetComponent<CollisionShape>());
    }

    // update collision points
    pbuoyCol->UpdateColPoints(waterBbox_);
}

//=============================================================================
//=============================================================================
BuoyCol::BuoyCol() 
    : rbody(NULL)
    , colShape(NULL)
{
    // default list size
    SetContainerSize(MaxPoints);
}

void BuoyCol::SetContainerSize(unsigned csize)
{
    localPointList.Resize(csize);
    worldPointList.Resize(csize);
    pointHeightList.Resize(csize);
    partialVolList.Resize(csize);
}

void BuoyCol::SetupBuoyShapeData(RigidBody *_rbody, CollisionShape *_colShape)
{
    // init
    rbody = _rbody;
    colShape = _colShape;

    if (colShape->GetShapeType() == SHAPE_SPHERE)
    {
        ConfigSphereData();
    }
    else
    {
        ConfigBboxPointData();
    }
}

void BuoyCol::UpdateColPoints(const BoundingBox &waterBbox)
{
    UpdateBuoyPoints(waterBbox);

    // reset flags and timer
    updateTimer.Reset();
    SetFrameUpdated(true);
}

void BuoyCol::ConfigSphereData()
{
    // sphere requires only a single point
    SetContainerSize(1);

    // get center and radius
    Vector3 mmin, mmax;
    GetLocalAabb(mmin, mmax);

    localPointList[0] = (mmax + mmin) * 0.5f;
    pointRadius = (mmax - mmin).x_ * 0.5f;

    // point volume
    pointMaxSphVol = 4.0f/3.0f * MPI * pointRadius * pointRadius * pointRadius;
    distributedVolumePerPt = pointMaxSphVol;
}

void BuoyCol::ConfigBboxPointData()
{
    // config pts of a bbox
    // **note** obviously you can position the pts where ever you need
    // example 1, for boats, move the bottom +z points back a bit to match the hull bottom
    // example 2, for cones, only add five pts - one top center and four bottom corners
    Vector3 mmin, mmax;
    GetLocalAabb(mmin, mmax);
    localPointList[0] = mmin;                               // (-x, -y, -z)
    localPointList[1] = Vector3(mmax.x_, mmin.y_, mmin.z_); // (+x, -y, -z)
    localPointList[2] = Vector3(mmin.x_, mmax.y_, mmin.z_); // (-x, +y, -z)
    localPointList[3] = Vector3(mmax.x_, mmax.y_, mmin.z_); // (+x, +y, -z)
    localPointList[4] = Vector3(mmin.x_, mmin.y_, mmax.z_); // (-x, -y, +z)
    localPointList[5] = Vector3(mmax.x_, mmin.y_, mmax.z_); // (+x, -y, +z)
    localPointList[6] = Vector3(mmin.x_, mmax.y_, mmax.z_); // (-x, +y, +z)
    localPointList[7] = mmax;                               // (+x, +y, +z)

    Vector3 bbSize = mmax - mmin;
    float bbVolume = GetShapeVolume(bbSize);

    // distribute the volume to eight equal sub-volumes
    distributedVolumePerPt = bbVolume / 8.0f;
    pointRadius = Min(Min(bbSize.x_, bbSize.y_), bbSize.z_) * 0.5f;
    pointMaxSphVol = 4.0f/3.0f * MPI * pointRadius * pointRadius * pointRadius;
}

void BuoyCol::UpdateBuoyPoints(const BoundingBox &waterBbox)
{
    btTransform worldTrans;
    rbody->getWorldTransform(worldTrans);

    for ( unsigned i = 0; i < worldPointList.Size(); ++i )
    {
        worldPointList[i] = ToVector3(worldTrans * ToBtVector3(localPointList[i]));
        pointHeightList[i] = worldPointList[i].y_ - waterBbox.max_.y_;
    }
}

void BuoyCol::GetLocalAabb(Vector3 &mmin, Vector3 &mmax)
{
    btTransform idTrans;
    btVector3 btaabbMin, btaabbMax;
    idTrans.setIdentity();
    colShape->GetCollisionShape()->getAabb(idTrans, btaabbMin, btaabbMax);
    mmin = ToVector3(btaabbMin);
    mmax = ToVector3(btaabbMax);
}

float BuoyCol::GetShapeVolume(const Vector3 &shapeSize) const
{
    // volume formulas - https://en.wikipedia.org/wiki/Volume#Volume_formulas
    float volume = 1.0f, radius, height;

    switch (colShape->GetShapeType())
    {
    case SHAPE_BOX:
    default: // **note** types not defined in the list defaults to the box type
        volume = (shapeSize.x_ * shapeSize.y_ * shapeSize.z_);
        break;

    case SHAPE_SPHERE:
        radius = shapeSize.x_ * 0.5f;
        volume = 4.0f/3.0f * MPI * radius * radius * radius;
        break;

    case SHAPE_CYLINDER:
    case SHAPE_CAPSULE:
        radius = shapeSize.x_ * 0.5f;
        height = shapeSize.y_;
        volume = MPI * radius * radius * height;
        break;

    case SHAPE_TRIANGLEMESH:
    case SHAPE_CONVEXHULL:
        // rough estimate - assume 1/3 of the vol is empty
        volume = (shapeSize.x_ * shapeSize.y_ * shapeSize.z_) * 2.0f/3.0f;
        break;
    }

    return volume;
}




