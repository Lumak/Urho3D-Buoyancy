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

#include <Urho3D/Core/CoreEvents.h>
#include <Urho3D/Core/ProcessUtils.h>
#include <Urho3D/Engine/Engine.h>
#include <Urho3D/Graphics/AnimatedModel.h>
#include <Urho3D/Graphics/AnimationController.h>
#include <Urho3D/Graphics/Camera.h>
#include <Urho3D/Graphics/Light.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Octree.h>
#include <Urho3D/Graphics/Renderer.h>
#include <Urho3D/Graphics/Zone.h>
#include <Urho3D/Graphics/Technique.h>
#include <Urho3D/Graphics/Texture2D.h>
#include <Urho3D/Graphics/Renderer.h>
#include <Urho3D/Graphics/RenderPath.h>
#include <Urho3D/Input/Controls.h>
#include <Urho3D/Input/Input.h>
#include <Urho3D/IO/FileSystem.h>
#include <Urho3D/Physics/CollisionShape.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/PhysicsEvents.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/IO/MemoryBuffer.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Graphics/DebugRenderer.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/UI/Font.h>
#include <Urho3D/UI/Text.h>
#include <Urho3D/UI/UI.h>
#include <Urho3D/Engine/DebugHud.h>
#include <SDL/SDL_log.h>
#include <Urho3D/Physics/PhysicsUtils.h>
#include <Bullet/BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <Bullet/BulletCollision/CollisionShapes/btCompoundShape.h>

#include "Character.h"
#include "CharacterDemo.h"
#include "Touch.h"
#include "WaterVolume.h"
#include "CollisionLayer.h"
#include "SmoothStep.h"

#include <Urho3D/DebugNew.h>
//=============================================================================
//=============================================================================
URHO3D_DEFINE_APPLICATION_MAIN(CharacterDemo)

//=============================================================================
//=============================================================================
CharacterDemo::CharacterDemo(Context* context)
    : Sample(context)
    , firstPerson_(false)
    , drawDebug_(false)
    , cameraUnderwater_(false)
{
    Character::RegisterObject(context);
    WaterVolume::RegisterObject(context);
}

CharacterDemo::~CharacterDemo()
{
}

void CharacterDemo::Setup()
{
    engineParameters_["WindowTitle"]  = GetTypeName();
    engineParameters_["LogName"]      = GetSubsystem<FileSystem>()->GetProgramDir() + "buoyancy.log";
    engineParameters_["FullScreen"]   = false;
    engineParameters_["Headless"]     = false;
    engineParameters_["WindowWidth"]  = 1280; 
    engineParameters_["WindowHeight"] = 720;
}

void CharacterDemo::Start()
{
    // Execute base class startup
    Sample::Start();
    if (touchEnabled_)
        touch_ = new Touch(context_, TOUCH_SENSITIVITY);

    ChangeDebugHudText();

    // Create static scene content
    CreateScene();
    CreateWaterVolume();
    CreateWaterRefection();

    // Create the controllable character
    CreateCharacter();

    // Create the UI content
    CreateInstructions();

    // Subscribe to necessary events
    SubscribeToEvents();

    // Set the mouse mode to use in the sample
    Sample::InitMouseMode(MM_RELATIVE);
}

void CharacterDemo::ChangeDebugHudText()
{
    // change profiler text
    if (GetSubsystem<DebugHud>())
    {
        Text *dbgText = GetSubsystem<DebugHud>()->GetProfilerText();
        dbgText->SetColor(Color::CYAN);
        dbgText->SetTextEffect(TE_NONE);

        dbgText = GetSubsystem<DebugHud>()->GetStatsText();
        dbgText->SetColor(Color::CYAN);
        dbgText->SetTextEffect(TE_NONE);

        dbgText = GetSubsystem<DebugHud>()->GetMemoryText();
        dbgText->SetColor(Color::CYAN);
        dbgText->SetTextEffect(TE_NONE);

        dbgText = GetSubsystem<DebugHud>()->GetModeText();
        dbgText->SetColor(Color::CYAN);
        dbgText->SetTextEffect(TE_NONE);
    }
}

void CharacterDemo::CreateScene()
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();
    Renderer* renderer = GetSubsystem<Renderer>();
    Graphics* graphics = GetSubsystem<Graphics>();

    scene_ = new Scene(context_);

    cameraNode_ = new Node(context_);
    Camera* camera = cameraNode_->CreateComponent<Camera>();
    camera->SetFarClip(300.0f);
    SharedPtr<Viewport> viewport(new Viewport(context_, scene_, camera));
    renderer->SetViewport(0, viewport);

    // post-process underwater
    effectRenderPath_ = viewport->GetRenderPath()->Clone();
    effectRenderPath_->Append(cache->GetResource<XMLFile>("PostProcess/UnderWater.xml"));
    effectRenderPath_->SetEnabled("Underwater", false);

    // set BlurHInvSize to proper value
    // **note** be sure to do this if screen size changes (not done for this demo)
    effectRenderPath_->SetShaderParameter("BlurHInvSize", Vector2(1.0f/(float)(graphics->GetWidth()), 1.0f/(float)(graphics->GetHeight())));
    viewport->SetRenderPath(effectRenderPath_);

    // load scene
    XMLFile *xmlLevel = cache->GetResource<XMLFile>("Buoyancy/waterScene.xml");
    scene_->LoadXML(xmlLevel->GetRoot());
}

void CharacterDemo::UpdateUnderwaterView()
{
    if (waterBbox_.IsInside(cameraNode_->GetPosition()))
    {
        if (!cameraUnderwater_)
        {
            effectRenderPath_->SetEnabled("Underwater", true);
            cameraUnderwater_ = true;
            SetWaterClipplane(cameraUnderwater_);
        }
    }
    else
    {
        if (cameraUnderwater_)
        {
            effectRenderPath_->SetEnabled("Underwater", false);
            cameraUnderwater_ = false;
            SetWaterClipplane(cameraUnderwater_);
        }
    }

}

void CharacterDemo::SetWaterClipplane(bool inwater)
{

    Vector3 waterSurfacePos = waterNode_->GetWorldPosition();
    waterSurfacePos.y_ = !inwater?waterBbox_.max_.y_:waterBbox_.min_.y_;

    waterClipPlane_.Define(waterNode_->GetWorldRotation() * Vector3(0.0f, 1.0f, 0.0f), waterSurfacePos - Vector3(0.0f, 0.1f, 0.0f));
    Camera* reflectionCamera = reflectionCameraNode_->GetComponent<Camera>();
    reflectionCamera->SetClipPlane(waterClipPlane_);
}

void CharacterDemo::CreateWaterVolume()
{
    Node *waterNode = scene_->GetChild("water");

    if (waterNode)
    {
        waterNode->CreateComponent<WaterVolume>();
    }
}

void CharacterDemo::CreateWaterRefection()
{
    // right out of 23_Water sample
    Graphics* graphics = GetSubsystem<Graphics>();
    Renderer* renderer = GetSubsystem<Renderer>();
    ResourceCache* cache = GetSubsystem<ResourceCache>();
    waterNode_ = scene_->GetChild("water", true);

    waterNode_->GetComponent<StaticModel>()->SetViewMask(0x80000000);

    // get water bbox
    waterBbox_ = waterNode_->GetComponent<CollisionShape>(true)->GetWorldBoundingBox();
    Vector3 waterSurfacePos = waterNode_->GetWorldPosition();
    waterSurfacePos.y_ = waterBbox_.max_.y_;

    waterPlane_ = Plane(waterNode_->GetWorldRotation() * Vector3(0.0f, 1.0f, 0.0f), waterSurfacePos);
    waterClipPlane_ = Plane(waterNode_->GetWorldRotation() * Vector3(0.0f, 1.0f, 0.0f), waterSurfacePos - Vector3(0.0f, 0.1f, 0.0f));

    reflectionCameraNode_ = cameraNode_->CreateChild();
    Camera* reflectionCamera = reflectionCameraNode_->CreateComponent<Camera>();
    reflectionCamera->SetFarClip(750.0);
    reflectionCamera->SetViewMask(0x7fffffff); // Hide objects with only bit 31 in the viewmask (the water plane)
    reflectionCamera->SetAutoAspectRatio(false);
    reflectionCamera->SetUseReflection(true);
    reflectionCamera->SetReflectionPlane(waterPlane_);
    reflectionCamera->SetUseClipping(true); // Enable clipping of geometry behind water plane
    reflectionCamera->SetClipPlane(waterClipPlane_);
    reflectionCamera->SetAspectRatio((float)graphics->GetWidth() / (float)graphics->GetHeight());

    int texSize = 1024;
    SharedPtr<Texture2D> renderTexture(new Texture2D(context_));
    renderTexture->SetSize(texSize, texSize, Graphics::GetRGBFormat(), TEXTURE_RENDERTARGET);
    renderTexture->SetFilterMode(FILTER_BILINEAR);
    RenderSurface* surface = renderTexture->GetRenderSurface();
    SharedPtr<Viewport> rttViewport(new Viewport(context_, scene_, reflectionCamera));
    surface->SetViewport(0, rttViewport);

    Material* waterMat = cache->GetResource<Material>("Buoyancy/Materials/waterMat.xml");
    waterMat->SetTexture(TU_SPECULAR, renderTexture);
}

void CharacterDemo::CreateCharacter()
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();

    Node *spawnNode = scene_->GetChild("playerSpawn");
    Node* objectNode = scene_->CreateChild("Player");
    objectNode->SetPosition(spawnNode->GetPosition());

    // spin node
    Node* adjustNode = objectNode->CreateChild("spinNode");
    adjustNode->SetRotation( Quaternion(180, Vector3(0,1,0) ) );
    
    // Create the rendering component + animation controller
    AnimatedModel* object = adjustNode->CreateComponent<AnimatedModel>();
    object->SetModel(cache->GetResource<Model>("Platforms/Models/BetaLowpoly/Beta.mdl"));
    object->SetMaterial(0, cache->GetResource<Material>("Platforms/Materials/BetaBody_MAT.xml"));
    object->SetMaterial(1, cache->GetResource<Material>("Platforms/Materials/BetaBody_MAT.xml"));
    object->SetMaterial(2, cache->GetResource<Material>("Platforms/Materials/BetaJoints_MAT.xml"));
    object->SetCastShadows(true);
    adjustNode->CreateComponent<AnimationController>();

    // Create rigidbody, and set non-zero mass so that the body becomes dynamic
    RigidBody* body = objectNode->CreateComponent<RigidBody>();
    body->SetCollisionLayer(ColLayer_Character);
    body->SetCollisionMask(ColMask_Character);
    body->SetMass(1.0f);

    body->SetAngularFactor(Vector3::ZERO);
    body->SetCollisionEventMode(COLLISION_ALWAYS);

    // Set a capsule shape for collision
    CollisionShape* shape = objectNode->CreateComponent<CollisionShape>();
    //shape->SetCapsule(0.7f, 1.8f, Vector3(0.0f, 0.9f, 0.0f));
    shape->SetCapsule(0.7f, 1.8f, Vector3(0.0f, 0.94f, 0.0f));

    // character
    character_ = objectNode->CreateComponent<Character>();

    // set rotation
    Quaternion qspRot = spawnNode->GetRotation();
    Vector3 euAng = qspRot.EulerAngles();
    character_->controls_.pitch_ = euAng.x_;
    character_->controls_.yaw_ = euAng.y_;

    // set water node
    character_->SetWaterNode(waterNode_);

    // init char pos/rot
    charRot_ = Quaternion(character_->controls_.yaw_, Vector3::UP);
    charPos_ = objectNode->GetPosition();
}

void CharacterDemo::CreateInstructions()
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();
    UI* ui = GetSubsystem<UI>();

    // Construct new Text object, set string to display and font to use
    Text* instructionText = ui->GetRoot()->CreateChild<Text>();
    instructionText->SetFont(cache->GetResource<Font>("Fonts/Anonymous Pro.ttf"), 15);
    instructionText->SetTextAlignment(HA_CENTER);

    // Position the text relative to the screen center
    instructionText->SetHorizontalAlignment(HA_CENTER);
    instructionText->SetVerticalAlignment(VA_CENTER);
    instructionText->SetPosition(0, ui->GetRoot()->GetHeight() / 4);
}

void CharacterDemo::SubscribeToEvents()
{
    SubscribeToEvent(E_UPDATE, URHO3D_HANDLER(CharacterDemo, HandleUpdate));
    SubscribeToEvent(E_POSTUPDATE, URHO3D_HANDLER(CharacterDemo, HandlePostUpdate));
    SubscribeToEvent(E_POSTRENDERUPDATE, URHO3D_HANDLER(CharacterDemo, HandlePostRenderUpdate));
    UnsubscribeFromEvent(E_SCENEUPDATE);
}

void CharacterDemo::HandleUpdate(StringHash eventType, VariantMap& eventData)
{
    using namespace Update;

    Input* input = GetSubsystem<Input>();

    if (character_)
    {
        // Clear previous controls
        character_->controls_.Set(CTRL_FORWARD | CTRL_BACK | CTRL_LEFT | CTRL_RIGHT | CTRL_JUMP, false);

        // Update controls using touch utility class
        if (touch_)
            touch_->UpdateTouches(character_->controls_);

        // Update controls using keys
        UI* ui = GetSubsystem<UI>();
        if (!ui->GetFocusElement())
        {
            if (!touch_ || !touch_->useGyroscope_)
            {
                character_->controls_.Set(CTRL_FORWARD, input->GetKeyDown(KEY_W));
                character_->controls_.Set(CTRL_BACK, input->GetKeyDown(KEY_S));
                character_->controls_.Set(CTRL_LEFT, input->GetKeyDown(KEY_A));
                character_->controls_.Set(CTRL_RIGHT, input->GetKeyDown(KEY_D));
            }
            character_->controls_.Set(CTRL_JUMP, input->GetKeyDown(KEY_SPACE));

            // Add character yaw & pitch from the mouse motion or touch input
            if (touchEnabled_)
            {
                for (unsigned i = 0; i < input->GetNumTouches(); ++i)
                {
                    TouchState* state = input->GetTouch(i);
                    if (!state->touchedElement_)    // Touch on empty space
                    {
                        Camera* camera = cameraNode_->GetComponent<Camera>();
                        if (!camera)
                            return;

                        Graphics* graphics = GetSubsystem<Graphics>();
                        character_->controls_.yaw_ += TOUCH_SENSITIVITY * camera->GetFov() / graphics->GetHeight() * state->delta_.x_;
                        character_->controls_.pitch_ += TOUCH_SENSITIVITY * camera->GetFov() / graphics->GetHeight() * state->delta_.y_;
                    }
                }
            }
            else
            {
                character_->controls_.yaw_ += (float)input->GetMouseMoveX() * YAW_SENSITIVITY;
                character_->controls_.pitch_ += (float)input->GetMouseMoveY() * YAW_SENSITIVITY;
            }
            // Limit pitch
            character_->controls_.pitch_ = Clamp(character_->controls_.pitch_, -80.0f, 80.0f);
            // Set rotation already here so that it's updated every rendering frame instead of every physics frame
            character_->GetNode()->SetRotation(Quaternion(character_->controls_.yaw_, Vector3::UP));

            // Turn on/off gyroscope on mobile platform
            if (touch_ && input->GetKeyPress(KEY_G))
                touch_->useGyroscope_ = !touch_->useGyroscope_;
        }
    }

    // Toggle debug geometry with space
    if (input->GetKeyPress(KEY_F4))
        drawDebug_ = !drawDebug_;

}

void CharacterDemo::HandlePostUpdate(StringHash eventType, VariantMap& eventData)
{
    if (!character_)
        return;

    using namespace Update;
    float timeStep = eventData[P_TIMESTEP].GetFloat();

    Node* characterNode = character_->GetNode();
    Vector3 charPos = characterNode->GetPosition();
    Quaternion rot = characterNode->GetRotation();

    // smooth cam
    const float posLerpRate = 14.0f;
    const float rotLerpRate = 15.0f;

    charPos_ = SmoothStep(charPos_, charPos, timeStep * posLerpRate);
    charRot_ = SmoothStepAngle(charRot_, rot, timeStep * rotLerpRate);

    Quaternion dir = charRot_ * Quaternion(character_->controls_.pitch_, Vector3::RIGHT);

    {
        Vector3 aimPoint = charPos_ + charRot_ * Vector3(0.0f, 1.7f, 0.0f);
        Vector3 rayDir = dir * Vector3::BACK;
        float rayDistance = touch_ ? touch_->cameraDistance_ : CAMERA_INITIAL_DIST;
        const float sphRadius = 0.1f;
        PhysicsRaycastResult result;

        scene_->GetComponent<PhysicsWorld>()->SphereCast(result, Ray(aimPoint, rayDir), sphRadius, rayDistance, ColMask_Camera);

        if (result.body_)
        {
            rayDistance = Min(rayDistance, result.distance_);
        }

        cameraNode_->SetPosition(aimPoint + rayDir * rayDistance);
        cameraNode_->SetRotation(dir);

        // underwater
        UpdateUnderwaterView();
    }
}


void CharacterDemo::HandlePostRenderUpdate(StringHash eventType, VariantMap& eventData)
{
    if (drawDebug_)
    {
        scene_->GetComponent<PhysicsWorld>()->DrawDebugGeometry(true);
    }
}




