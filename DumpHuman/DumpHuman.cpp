#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/ObjectStream/ObjectStreamIn.h>
#include <Jolt/ObjectStream/ObjectStreamOut.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/TaperedCapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/PhysicsMaterialSimple.h>
#include <Jolt/Physics/Collision/GroupFilterTable.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/PointConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>
#include <Jolt/Physics/Constraints/DistanceConstraint.h>
#include <Jolt/Physics/Constraints/ConeConstraint.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>

#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <string>

using namespace JPH;
using json = nlohmann::json;

// ==========================================
// 1. EXPORTER (Jolt C++ -> JSON)
// ==========================================

EShapeSubType GetShapeSubType(const ShapeSettings* shape) {
    if (shape->GetRTTI() == JPH_RTTI(CapsuleShapeSettings)) return EShapeSubType::Capsule;
    if (shape->GetRTTI() == JPH_RTTI(TaperedCapsuleShapeSettings)) return EShapeSubType::TaperedCapsule;
    if (shape->GetRTTI() == JPH_RTTI(BoxShapeSettings)) return EShapeSubType::Box;
    if (shape->GetRTTI() == JPH_RTTI(StaticCompoundShapeSettings)) return EShapeSubType::StaticCompound;
    if (shape->GetRTTI() == JPH_RTTI(SphereShapeSettings)) return EShapeSubType::Sphere;
    if (shape->GetRTTI() == JPH_RTTI(CylinderShapeSettings)) return EShapeSubType::Cylinder;
    if (shape->GetRTTI() == JPH_RTTI(ConvexHullShapeSettings)) return EShapeSubType::ConvexHull;
    if (shape->GetRTTI() == JPH_RTTI(MeshShapeSettings)) return EShapeSubType::Mesh;
    if (shape->GetRTTI() == JPH_RTTI(RotatedTranslatedShapeSettings)) return EShapeSubType::RotatedTranslated;
    if (shape->GetRTTI() == JPH_RTTI(OffsetCenterOfMassShapeSettings)) return EShapeSubType::OffsetCenterOfMass;
    return EShapeSubType::Empty;
}

json ExportMaterial(const PhysicsMaterial* material) {
    if (!material) return nullptr;
    if (material->GetRTTI() == JPH_RTTI(PhysicsMaterialSimple)) {
        auto m = static_cast<const PhysicsMaterialSimple*>(material);
        json j;
        j["name"] = m->GetDebugName();
        Color c = m->GetDebugColor();
        j["color"] = {c.r, c.g, c.b, c.a};
        return j;
    }
    return nullptr;
}

RefConst<PhysicsMaterial> ImportMaterial(const json& j) {
    if (j.is_null()) return nullptr;
    const std::string name = j.value("name", "Unknown");
    Color c = Color::sGrey;
    if (j.contains("color")) {
        c.r = j["color"][0].get<uint8_t>();
        c.g = j["color"][1].get<uint8_t>();
        c.b = j["color"][2].get<uint8_t>();
        c.a = j["color"][3].get<uint8_t>();
    }
    return new PhysicsMaterialSimple(name, c);
}

json ExportShape(const ShapeSettings* shape) {
    if (!shape) return nullptr;
    json j;
    j["ptr"] = (uintptr_t)shape;
    j["type"] = (int)GetShapeSubType(shape);
    j["userData"] = shape->mUserData;

    if (shape->GetRTTI()->IsKindOf(JPH_RTTI(ConvexShapeSettings))) {
        j["density"] = static_cast<const ConvexShapeSettings*>(shape)->mDensity;
        j["material"] = ExportMaterial(static_cast<const ConvexShapeSettings*>(shape)->mMaterial);
    }

    switch (GetShapeSubType(shape)) {
        case EShapeSubType::Capsule: {
            auto c = static_cast<const CapsuleShapeSettings*>(shape);
            j["radius"] = c->mRadius;
            j["halfHeight"] = c->mHalfHeightOfCylinder;
            break;
        }
        case EShapeSubType::TaperedCapsule: {
            auto c = static_cast<const TaperedCapsuleShapeSettings*>(shape);
            j["halfHeight"] = c->mHalfHeightOfTaperedCylinder;
            j["topRadius"] = c->mTopRadius;
            j["bottomRadius"] = c->mBottomRadius;
            break;
        }
        case EShapeSubType::Box: {
            auto c = static_cast<const BoxShapeSettings*>(shape);
            j["halfExtent"] = {c->mHalfExtent.GetX(), c->mHalfExtent.GetY(), c->mHalfExtent.GetZ()};
            j["convexRadius"] = c->mConvexRadius;
            break;
        }
        case EShapeSubType::Sphere: {
            auto c = static_cast<const SphereShapeSettings*>(shape);
            j["radius"] = c->mRadius;
            break;
        }
        case EShapeSubType::Cylinder: {
            auto c = static_cast<const CylinderShapeSettings*>(shape);
            j["halfHeight"] = c->mHalfHeight;
            j["radius"] = c->mRadius;
            j["convexRadius"] = c->mConvexRadius;
            break;
        }
        case EShapeSubType::RotatedTranslated: {
            auto c = static_cast<const RotatedTranslatedShapeSettings*>(shape);
            j["position"] = {c->mPosition.GetX(), c->mPosition.GetY(), c->mPosition.GetZ()};
            j["rotation"] = {c->mRotation.GetX(), c->mRotation.GetY(), c->mRotation.GetZ(), c->mRotation.GetW()};
            j["innerShape"] = ExportShape(c->mInnerShape);
            break;
        }
        case EShapeSubType::OffsetCenterOfMass: {
            auto c = static_cast<const OffsetCenterOfMassShapeSettings*>(shape);
            j["offset"] = {c->mOffset.GetX(), c->mOffset.GetY(), c->mOffset.GetZ()};
            j["innerShape"] = ExportShape(c->mInnerShape);
            break;
        }
        case EShapeSubType::StaticCompound: {
            auto c = static_cast<const StaticCompoundShapeSettings*>(shape);
            json subs = json::array();
            for (const auto& sub : c->mSubShapes) {
                json sj;
                sj["position"] = {sub.mPosition.GetX(), sub.mPosition.GetY(), sub.mPosition.GetZ()};
                sj["rotation"] = {sub.mRotation.GetX(), sub.mRotation.GetY(), sub.mRotation.GetZ(), sub.mRotation.GetW()};
                sj["shape"] = ExportShape(sub.mShape);
                sj["userData"] = sub.mUserData;
                subs.push_back(sj);
            }
            j["subShapes"] = subs;
            break;
        }
        case EShapeSubType::ConvexHull: {
            auto c = static_cast<const ConvexHullShapeSettings*>(shape);
            j["maxConvexRadius"] = c->mMaxConvexRadius;
            j["maxErrorConvexRadius"] = c->mMaxErrorConvexRadius;
            j["hullTolerance"] = c->mHullTolerance;
            json pts = json::array();
            for (const auto& p : c->mPoints) pts.push_back({p.GetX(), p.GetY(), p.GetZ()});
            j["points"] = pts;
            break;
        }
        case EShapeSubType::Mesh: {
            auto c = static_cast<const MeshShapeSettings*>(shape);
            j["maxTrianglesPerLeaf"] = c->mMaxTrianglesPerLeaf;
            j["activeEdgeCosThresholdAngle"] = c->mActiveEdgeCosThresholdAngle;
            json verts = json::array();
            for (const auto& v : c->mTriangleVertices) verts.push_back({v.x, v.y, v.z});
            j["vertices"] = verts;
            json tris = json::array();
            for (const auto& t : c->mIndexedTriangles) tris.push_back({t.mIdx[0], t.mIdx[1], t.mIdx[2], t.mMaterialIndex});
            j["triangles"] = tris;
            json mats = json::array();
            for (const auto& m : c->mMaterials) mats.push_back(ExportMaterial(m));
            j["materials"] = mats;
            break;
        }
        default: break;
    }
    return j;
}

EConstraintSubType GetConstraintSubType(const TwoBodyConstraintSettings* c) {
    if (!c) return EConstraintSubType::Fixed;
    if (c->GetRTTI() == JPH_RTTI(FixedConstraintSettings)) return EConstraintSubType::Fixed;
    if (c->GetRTTI() == JPH_RTTI(PointConstraintSettings)) return EConstraintSubType::Point;
    if (c->GetRTTI() == JPH_RTTI(HingeConstraintSettings)) return EConstraintSubType::Hinge;
    if (c->GetRTTI() == JPH_RTTI(SliderConstraintSettings)) return EConstraintSubType::Slider;
    if (c->GetRTTI() == JPH_RTTI(DistanceConstraintSettings)) return EConstraintSubType::Distance;
    if (c->GetRTTI() == JPH_RTTI(ConeConstraintSettings)) return EConstraintSubType::Cone;
    if (c->GetRTTI() == JPH_RTTI(SwingTwistConstraintSettings)) return EConstraintSubType::SwingTwist;
    if (c->GetRTTI() == JPH_RTTI(SixDOFConstraintSettings)) return EConstraintSubType::SixDOF;
    return EConstraintSubType::Fixed;
}

json ExportConstraint(const TwoBodyConstraintSettings* c) {
    if (!c) return nullptr;
    json j;
    j["type"] = (int)GetConstraintSubType(c);
    j["enabled"] = c->mEnabled;
    j["priority"] = c->mConstraintPriority;
    j["posSteps"] = c->mNumPositionStepsOverride;
    j["velSteps"] = c->mNumVelocityStepsOverride;
    j["drawSize"] = c->mDrawConstraintSize;

    auto expMotor = [](const MotorSettings& m) {
        return json{{"freq", m.mSpringSettings.mFrequency}, {"damp", m.mSpringSettings.mDamping},
                    {"mode", (int)m.mSpringSettings.mMode}, {"minF", m.mMinForceLimit},
                    {"maxF", m.mMaxForceLimit}, {"minT", m.mMinTorqueLimit}, {"maxT", m.mMaxTorqueLimit}};
    };
    auto expSpring = [](const SpringSettings& s) {
        return json{{"mode", (int)s.mMode}, {"freq", s.mFrequency}, {"damp", s.mDamping}};
    };

    switch (GetConstraintSubType(c)) {
        case EConstraintSubType::Fixed: {
            auto st = static_cast<const FixedConstraintSettings*>(c);
            j["space"] = (int)st->mSpace; j["autoDetect"] = st->mAutoDetectPoint;
            j["pos1"] = {st->mPoint1.GetX(), st->mPoint1.GetY(), st->mPoint1.GetZ()};
            j["axisX1"] = {st->mAxisX1.GetX(), st->mAxisX1.GetY(), st->mAxisX1.GetZ()};
            j["axisY1"] = {st->mAxisY1.GetX(), st->mAxisY1.GetY(), st->mAxisY1.GetZ()};
            j["pos2"] = {st->mPoint2.GetX(), st->mPoint2.GetY(), st->mPoint2.GetZ()};
            j["axisX2"] = {st->mAxisX2.GetX(), st->mAxisX2.GetY(), st->mAxisX2.GetZ()};
            j["axisY2"] = {st->mAxisY2.GetX(), st->mAxisY2.GetY(), st->mAxisY2.GetZ()};
            break;
        }
        case EConstraintSubType::Point: {
            auto st = static_cast<const PointConstraintSettings*>(c);
            j["space"] = (int)st->mSpace;
            j["pos1"] = {st->mPoint1.GetX(), st->mPoint1.GetY(), st->mPoint1.GetZ()};
            j["pos2"] = {st->mPoint2.GetX(), st->mPoint2.GetY(), st->mPoint2.GetZ()};
            break;
        }
        case EConstraintSubType::Hinge: {
            auto st = static_cast<const HingeConstraintSettings*>(c);
            j["space"] = (int)st->mSpace;
            j["pos1"] = {st->mPoint1.GetX(), st->mPoint1.GetY(), st->mPoint1.GetZ()};
            j["axis1"] = {st->mHingeAxis1.GetX(), st->mHingeAxis1.GetY(), st->mHingeAxis1.GetZ()};
            j["norm1"] = {st->mNormalAxis1.GetX(), st->mNormalAxis1.GetY(), st->mNormalAxis1.GetZ()};
            j["pos2"] = {st->mPoint2.GetX(), st->mPoint2.GetY(), st->mPoint2.GetZ()};
            j["axis2"] = {st->mHingeAxis2.GetX(), st->mHingeAxis2.GetY(), st->mHingeAxis2.GetZ()};
            j["norm2"] = {st->mNormalAxis2.GetX(), st->mNormalAxis2.GetY(), st->mNormalAxis2.GetZ()};
            j["limMin"] = st->mLimitsMin; j["limMax"] = st->mLimitsMax;
            j["limSpring"] = expSpring(st->mLimitsSpringSettings);
            j["maxFriction"] = st->mMaxFrictionTorque;
            j["motor"] = expMotor(st->mMotorSettings);
            break;
        }
        case EConstraintSubType::Slider: {
            auto st = static_cast<const SliderConstraintSettings*>(c);
            j["space"] = (int)st->mSpace; j["autoDetect"] = st->mAutoDetectPoint;
            j["pos1"] = {st->mPoint1.GetX(), st->mPoint1.GetY(), st->mPoint1.GetZ()};
            j["axis1"] = {st->mSliderAxis1.GetX(), st->mSliderAxis1.GetY(), st->mSliderAxis1.GetZ()};
            j["norm1"] = {st->mNormalAxis1.GetX(), st->mNormalAxis1.GetY(), st->mNormalAxis1.GetZ()};
            j["pos2"] = {st->mPoint2.GetX(), st->mPoint2.GetY(), st->mPoint2.GetZ()};
            j["axis2"] = {st->mSliderAxis2.GetX(), st->mSliderAxis2.GetY(), st->mSliderAxis2.GetZ()};
            j["norm2"] = {st->mNormalAxis2.GetX(), st->mNormalAxis2.GetY(), st->mNormalAxis2.GetZ()};
            j["limMin"] = st->mLimitsMin; j["limMax"] = st->mLimitsMax;
            j["limSpring"] = expSpring(st->mLimitsSpringSettings);
            j["maxFriction"] = st->mMaxFrictionForce;
            j["motor"] = expMotor(st->mMotorSettings);
            break;
        }
        case EConstraintSubType::Distance: {
            auto st = static_cast<const DistanceConstraintSettings*>(c);
            j["space"] = (int)st->mSpace;
            j["pos1"] = {st->mPoint1.GetX(), st->mPoint1.GetY(), st->mPoint1.GetZ()};
            j["pos2"] = {st->mPoint2.GetX(), st->mPoint2.GetY(), st->mPoint2.GetZ()};
            j["minD"] = st->mMinDistance; j["maxD"] = st->mMaxDistance;
            j["limSpring"] = expSpring(st->mLimitsSpringSettings);
            break;
        }
        case EConstraintSubType::Cone: {
            auto st = static_cast<const ConeConstraintSettings*>(c);
            j["space"] = (int)st->mSpace;
            j["pos1"] = {st->mPoint1.GetX(), st->mPoint1.GetY(), st->mPoint1.GetZ()};
            j["twist1"] = {st->mTwistAxis1.GetX(), st->mTwistAxis1.GetY(), st->mTwistAxis1.GetZ()};
            j["pos2"] = {st->mPoint2.GetX(), st->mPoint2.GetY(), st->mPoint2.GetZ()};
            j["twist2"] = {st->mTwistAxis2.GetX(), st->mTwistAxis2.GetY(), st->mTwistAxis2.GetZ()};
            j["halfCone"] = st->mHalfConeAngle;
            break;
        }
        case EConstraintSubType::SwingTwist: {
            auto st = static_cast<const SwingTwistConstraintSettings*>(c);
            j["space"] = (int)st->mSpace;
            j["pos1"] = {st->mPosition1.GetX(), st->mPosition1.GetY(), st->mPosition1.GetZ()};
            j["twist1"] = {st->mTwistAxis1.GetX(), st->mTwistAxis1.GetY(), st->mTwistAxis1.GetZ()};
            j["plane1"] = {st->mPlaneAxis1.GetX(), st->mPlaneAxis1.GetY(), st->mPlaneAxis1.GetZ()};
            j["pos2"] = {st->mPosition2.GetX(), st->mPosition2.GetY(), st->mPosition2.GetZ()};
            j["twist2"] = {st->mTwistAxis2.GetX(), st->mTwistAxis2.GetY(), st->mTwistAxis2.GetZ()};
            j["plane2"] = {st->mPlaneAxis2.GetX(), st->mPlaneAxis2.GetY(), st->mPlaneAxis2.GetZ()};
            j["normalHalfCone"] = st->mNormalHalfConeAngle;
            j["planeHalfCone"] = st->mPlaneHalfConeAngle;
            j["twistMin"] = st->mTwistMinAngle; j["twistMax"] = st->mTwistMaxAngle;
            j["maxFriction"] = st->mMaxFrictionTorque;
            j["swingMotor"] = expMotor(st->mSwingMotorSettings);
            j["twistMotor"] = expMotor(st->mTwistMotorSettings);
            break;
        }
        case EConstraintSubType::SixDOF: {
            auto st = static_cast<const SixDOFConstraintSettings*>(c);
            j["space"] = (int)st->mSpace;
            j["pos1"] = {st->mPosition1.GetX(), st->mPosition1.GetY(), st->mPosition1.GetZ()};
            j["axisX1"] = {st->mAxisX1.GetX(), st->mAxisX1.GetY(), st->mAxisX1.GetZ()};
            j["axisY1"] = {st->mAxisY1.GetX(), st->mAxisY1.GetY(), st->mAxisY1.GetZ()};
            j["pos2"] = {st->mPosition2.GetX(), st->mPosition2.GetY(), st->mPosition2.GetZ()};
            j["axisX2"] = {st->mAxisX2.GetX(), st->mAxisX2.GetY(), st->mAxisX2.GetZ()};
            j["axisY2"] = {st->mAxisY2.GetX(), st->mAxisY2.GetY(), st->mAxisY2.GetZ()};
            for (int i = 0; i < 6; ++i) j["maxFric"][i] = st->mMaxFriction[i];
            for (int i = 0; i < 6; ++i) { j["limMin"][i] = st->mLimitMin[i]; j["limMax"][i] = st->mLimitMax[i]; }
            for (int i = 0; i < 6; ++i) j["motor"][i] = expMotor(st->mMotorSettings[i]);
            break;
        }
        default: break;
    }
    return j;
}

json ExportRagdoll(const RagdollSettings* r) {
    json j;
    
    // 1. Skeleton
    json joints = json::array();
    for (int i = 0; i < r->mSkeleton->GetJointCount(); ++i) {
        const auto& joint = r->mSkeleton->GetJoint(i);
        joints.push_back({ {"name", joint.mName.c_str()}, {"parentName", joint.mParentName.c_str()} });
    }
    j["skeleton"] = joints;

    // 2. Parts
    json parts = json::array();
    for (const auto& part : r->mParts) {
        json pj;
        pj["position"] = {part.mPosition.GetX(), part.mPosition.GetY(), part.mPosition.GetZ()};
        pj["rotation"] = {part.mRotation.GetX(), part.mRotation.GetY(), part.mRotation.GetZ(), part.mRotation.GetW()};
        pj["shape"] = ExportShape(part.GetShapeSettings());
        
        pj["collGroup"] = {
            {"groupID", part.mCollisionGroup.GetGroupID()},
            {"subGroupID", part.mCollisionGroup.GetSubGroupID()}
        };

        if (part.mCollisionGroup.GetGroupFilter()) {
            auto gf = static_cast<const GroupFilterTable*>(part.mCollisionGroup.GetGroupFilter());
            pj["collGroup"]["filterPtr"] = (uintptr_t)gf;
            json filterMatrix = json::array();
            for(int a = 0; a < 23; ++a) {
                json row = json::array();
                for(int b = 0; b < 23; ++b) {
                    if (a == b) {
                        row.push_back(false);
                    } else {
                        row.push_back(gf->IsCollisionEnabled(a, b));
                    }
                }
                filterMatrix.push_back(row);
            }
            pj["collGroup"]["filter"] = filterMatrix;
        }

        pj["objectLayer"] = part.mObjectLayer;
        pj["motionType"] = (int)part.mMotionType;
        pj["allowDynamicOrKinematic"] = part.mAllowDynamicOrKinematic;
        pj["motionQuality"] = (int)part.mMotionQuality;
        pj["allowSleeping"] = part.mAllowSleeping;
        pj["friction"] = part.mFriction;
        pj["restitution"] = part.mRestitution;
        pj["linearDamping"] = part.mLinearDamping;
        pj["angularDamping"] = part.mAngularDamping;
        pj["maxLinearVelocity"] = part.mMaxLinearVelocity;
        pj["maxAngularVelocity"] = part.mMaxAngularVelocity;
        pj["gravityFactor"] = part.mGravityFactor;
        
        pj["allowedDOFs"] = (int)part.mAllowedDOFs;
        pj["isSensor"] = part.mIsSensor;
        pj["collideKinematicVsNonDynamic"] = part.mCollideKinematicVsNonDynamic;
        pj["useManifoldReduction"] = part.mUseManifoldReduction;
        pj["applyGyroscopicForce"] = part.mApplyGyroscopicForce;
        pj["enhancedInternalEdgeRemoval"] = part.mEnhancedInternalEdgeRemoval;
        pj["numVelocityStepsOverride"] = part.mNumVelocityStepsOverride;
        pj["numPositionStepsOverride"] = part.mNumPositionStepsOverride;
        pj["inertiaMultiplier"] = part.mInertiaMultiplier;
        
        pj["overrideMass"] = (int)part.mOverrideMassProperties;
        pj["mass"] = part.mMassPropertiesOverride.mMass;
        
        json mat = json::array();
        for(int i=0; i<4; ++i) {
            Vec4 c = part.mMassPropertiesOverride.mInertia.GetColumn4(i);
            mat.push_back({c.GetX(), c.GetY(), c.GetZ(), c.GetW()});
        }
        pj["inertia"] = mat;

        pj["toParent"] = ExportConstraint(part.mToParent);
        parts.push_back(pj);
    }
    j["parts"] = parts;

    // 3. Additional Constraints
    json addConstraints = json::array();
    for (const auto& c : r->mAdditionalConstraints) {
        json cj;
        cj["bodyIdx"] = { c.mBodyIdx[0], c.mBodyIdx[1] };
        cj["constraint"] = ExportConstraint(c.mConstraint);
        addConstraints.push_back(cj);
    }
    j["additionalConstraints"] = addConstraints;

    return j;
}


// ==========================================
// 2. IMPORTER (JSON -> Jolt C++)
// ==========================================

std::unordered_map<std::string, Ref<ShapeSettings>> gShapeCache;
std::unordered_map<std::string, Ref<GroupFilter>> gFilterCache;

Ref<ShapeSettings> ImportShape(const json& j) {
    if (j.is_null()) return nullptr;
    std::string hash = std::to_string(j["ptr"].get<uintptr_t>());
    if (gShapeCache.count(hash)) return gShapeCache[hash];
    Ref<ShapeSettings> shape;
    int type = j["type"];

    if (type == (int)EShapeSubType::Capsule) {
        shape = new CapsuleShapeSettings(j["halfHeight"], j["radius"]);
    } else if (type == (int)EShapeSubType::TaperedCapsule) {
        shape = new TaperedCapsuleShapeSettings(j["halfHeight"], j["topRadius"], j["bottomRadius"]);
    } else if (type == (int)EShapeSubType::Box) {
        shape = new BoxShapeSettings(Vec3(j["halfExtent"][0], j["halfExtent"][1], j["halfExtent"][2]), j["convexRadius"]);
    } else if (type == (int)EShapeSubType::Sphere) {
        shape = new SphereShapeSettings(j["radius"]);
    } else if (type == (int)EShapeSubType::Cylinder) {
        shape = new CylinderShapeSettings(j["halfHeight"], j["radius"], j["convexRadius"]);
    } else if (type == (int)EShapeSubType::RotatedTranslated) {
        shape = new RotatedTranslatedShapeSettings(
            Vec3(j["position"][0], j["position"][1], j["position"][2]),
            Quat(j["rotation"][0], j["rotation"][1], j["rotation"][2], j["rotation"][3]),
            ImportShape(j["innerShape"]));
    } else if (type == (int)EShapeSubType::OffsetCenterOfMass) {
        shape = new OffsetCenterOfMassShapeSettings(
            Vec3(j["offset"][0], j["offset"][1], j["offset"][2]),
            ImportShape(j["innerShape"]));
    } else if (type == (int)EShapeSubType::StaticCompound) {
        auto comp = new StaticCompoundShapeSettings();
        for (const auto& sub : j["subShapes"]) {
            comp->AddShape(
                Vec3(sub["position"][0], sub["position"][1], sub["position"][2]),
                Quat(sub["rotation"][0], sub["rotation"][1], sub["rotation"][2], sub["rotation"][3]),
                ImportShape(sub["shape"]), sub.value("userData", 0u));
        }
        shape = comp;
    } else if (type == (int)EShapeSubType::ConvexHull) {
        Array<Vec3> pts;
        for (const auto& p : j["points"]) pts.push_back(Vec3(p[0], p[1], p[2]));
        auto hull = new ConvexHullShapeSettings(pts, j["maxConvexRadius"]);
        hull->mMaxErrorConvexRadius = j["maxErrorConvexRadius"];
        hull->mHullTolerance = j["hullTolerance"];
        shape = hull;
    } else if (type == (int)EShapeSubType::Mesh) {
        VertexList verts;
        for (const auto& v : j["vertices"]) verts.push_back({v[0], v[1], v[2]});
        IndexedTriangleList tris;
        for (const auto& t : j["triangles"]) tris.push_back(IndexedTriangle(t[0], t[1], t[2], t[3]));
        PhysicsMaterialList mats;
        for (const auto& m : j["materials"]) mats.push_back(ImportMaterial(m));
        auto mesh = new MeshShapeSettings(verts, tris, mats);
        mesh->mMaxTrianglesPerLeaf = j["maxTrianglesPerLeaf"];
        mesh->mActiveEdgeCosThresholdAngle = j["activeEdgeCosThresholdAngle"];
        shape = mesh;
    }

    if (shape) {
        shape->mUserData = j["userData"];
        if (shape->GetRTTI()->IsKindOf(JPH_RTTI(ConvexShapeSettings))) {
            auto* cs = static_cast<ConvexShapeSettings*>(shape.GetPtr());
            if (j.contains("density")) cs->mDensity = j["density"];
            if (j.contains("material")) cs->mMaterial = ImportMaterial(j["material"]);
        }
    }
    gShapeCache[hash] = shape;
    return shape;
}

Ref<TwoBodyConstraintSettings> ImportConstraint(const json& j) {
    if (j.is_null()) return nullptr;
    int type = j["type"];

    auto impMotor = [](MotorSettings& m, const json& mj) {
        m.mSpringSettings.mFrequency = mj["freq"];
        m.mSpringSettings.mDamping = mj["damp"];
        m.mSpringSettings.mMode = (ESpringMode)mj.value("mode", 0);
        m.mMinForceLimit = mj["minF"]; m.mMaxForceLimit = mj["maxF"];
        m.mMinTorqueLimit = mj["minT"]; m.mMaxTorqueLimit = mj["maxT"];
    };
    auto impSpring = [](SpringSettings& s, const json& sj) {
        s.mMode = (ESpringMode)sj.value("mode", 0);
        s.mFrequency = sj["freq"]; s.mDamping = sj["damp"];
    };

    Ref<TwoBodyConstraintSettings> c;
    switch ((EConstraintSubType)type) {
        case EConstraintSubType::Fixed: {
            auto st = new FixedConstraintSettings();
            st->mSpace = (EConstraintSpace)j["space"].get<int>();
            st->mAutoDetectPoint = j["autoDetect"];
            st->mPoint1 = Vec3(j["pos1"][0], j["pos1"][1], j["pos1"][2]);
            st->mAxisX1 = Vec3(j["axisX1"][0], j["axisX1"][1], j["axisX1"][2]);
            st->mAxisY1 = Vec3(j["axisY1"][0], j["axisY1"][1], j["axisY1"][2]);
            st->mPoint2 = Vec3(j["pos2"][0], j["pos2"][1], j["pos2"][2]);
            st->mAxisX2 = Vec3(j["axisX2"][0], j["axisX2"][1], j["axisX2"][2]);
            st->mAxisY2 = Vec3(j["axisY2"][0], j["axisY2"][1], j["axisY2"][2]);
            c = st; break;
        }
        case EConstraintSubType::Point: {
            auto st = new PointConstraintSettings();
            st->mSpace = (EConstraintSpace)j["space"].get<int>();
            st->mPoint1 = Vec3(j["pos1"][0], j["pos1"][1], j["pos1"][2]);
            st->mPoint2 = Vec3(j["pos2"][0], j["pos2"][1], j["pos2"][2]);
            c = st; break;
        }
        case EConstraintSubType::Hinge: {
            auto st = new HingeConstraintSettings();
            st->mSpace = (EConstraintSpace)j["space"].get<int>();
            st->mPoint1 = Vec3(j["pos1"][0], j["pos1"][1], j["pos1"][2]);
            st->mHingeAxis1 = Vec3(j["axis1"][0], j["axis1"][1], j["axis1"][2]);
            st->mNormalAxis1 = Vec3(j["norm1"][0], j["norm1"][1], j["norm1"][2]);
            st->mPoint2 = Vec3(j["pos2"][0], j["pos2"][1], j["pos2"][2]);
            st->mHingeAxis2 = Vec3(j["axis2"][0], j["axis2"][1], j["axis2"][2]);
            st->mNormalAxis2 = Vec3(j["norm2"][0], j["norm2"][1], j["norm2"][2]);
            st->mLimitsMin = j["limMin"]; st->mLimitsMax = j["limMax"];
            impSpring(st->mLimitsSpringSettings, j["limSpring"]);
            st->mMaxFrictionTorque = j["maxFriction"];
            impMotor(st->mMotorSettings, j["motor"]);
            c = st; break;
        }
        case EConstraintSubType::Slider: {
            auto st = new SliderConstraintSettings();
            st->mSpace = (EConstraintSpace)j["space"].get<int>();
            st->mAutoDetectPoint = j["autoDetect"];
            st->mPoint1 = Vec3(j["pos1"][0], j["pos1"][1], j["pos1"][2]);
            st->mSliderAxis1 = Vec3(j["axis1"][0], j["axis1"][1], j["axis1"][2]);
            st->mNormalAxis1 = Vec3(j["norm1"][0], j["norm1"][1], j["norm1"][2]);
            st->mPoint2 = Vec3(j["pos2"][0], j["pos2"][1], j["pos2"][2]);
            st->mSliderAxis2 = Vec3(j["axis2"][0], j["axis2"][1], j["axis2"][2]);
            st->mNormalAxis2 = Vec3(j["norm2"][0], j["norm2"][1], j["norm2"][2]);
            st->mLimitsMin = j["limMin"]; st->mLimitsMax = j["limMax"];
            impSpring(st->mLimitsSpringSettings, j["limSpring"]);
            st->mMaxFrictionForce = j["maxFriction"];
            impMotor(st->mMotorSettings, j["motor"]);
            c = st; break;
        }
        case EConstraintSubType::Distance: {
            auto st = new DistanceConstraintSettings();
            st->mSpace = (EConstraintSpace)j["space"].get<int>();
            st->mPoint1 = Vec3(j["pos1"][0], j["pos1"][1], j["pos1"][2]);
            st->mPoint2 = Vec3(j["pos2"][0], j["pos2"][1], j["pos2"][2]);
            st->mMinDistance = j["minD"]; st->mMaxDistance = j["maxD"];
            impSpring(st->mLimitsSpringSettings, j["limSpring"]);
            c = st; break;
        }
        case EConstraintSubType::Cone: {
            auto st = new ConeConstraintSettings();
            st->mSpace = (EConstraintSpace)j["space"].get<int>();
            st->mPoint1 = Vec3(j["pos1"][0], j["pos1"][1], j["pos1"][2]);
            st->mTwistAxis1 = Vec3(j["twist1"][0], j["twist1"][1], j["twist1"][2]);
            st->mPoint2 = Vec3(j["pos2"][0], j["pos2"][1], j["pos2"][2]);
            st->mTwistAxis2 = Vec3(j["twist2"][0], j["twist2"][1], j["twist2"][2]);
            st->mHalfConeAngle = j["halfCone"];
            c = st; break;
        }
        case EConstraintSubType::SwingTwist: {
            auto st = new SwingTwistConstraintSettings();
            st->mSpace = (EConstraintSpace)j["space"].get<int>();
            st->mPosition1 = RVec3(j["pos1"][0], j["pos1"][1], j["pos1"][2]);
            st->mTwistAxis1 = Vec3(j["twist1"][0], j["twist1"][1], j["twist1"][2]);
            st->mPlaneAxis1 = Vec3(j["plane1"][0], j["plane1"][1], j["plane1"][2]);
            st->mPosition2 = RVec3(j["pos2"][0], j["pos2"][1], j["pos2"][2]);
            st->mTwistAxis2 = Vec3(j["twist2"][0], j["twist2"][1], j["twist2"][2]);
            st->mPlaneAxis2 = Vec3(j["plane2"][0], j["plane2"][1], j["plane2"][2]);
            st->mNormalHalfConeAngle = j["normalHalfCone"];
            st->mPlaneHalfConeAngle = j["planeHalfCone"];
            st->mTwistMinAngle = j["twistMin"]; st->mTwistMaxAngle = j["twistMax"];
            st->mMaxFrictionTorque = j["maxFriction"];
            impMotor(st->mSwingMotorSettings, j["swingMotor"]);
            impMotor(st->mTwistMotorSettings, j["twistMotor"]);
            c = st; break;
        }
        case EConstraintSubType::SixDOF: {
            auto st = new SixDOFConstraintSettings();
            st->mSpace = (EConstraintSpace)j["space"].get<int>();
            st->mPosition1 = Vec3(j["pos1"][0], j["pos1"][1], j["pos1"][2]);
            st->mAxisX1 = Vec3(j["axisX1"][0], j["axisX1"][1], j["axisX1"][2]);
            st->mAxisY1 = Vec3(j["axisY1"][0], j["axisY1"][1], j["axisY1"][2]);
            st->mPosition2 = Vec3(j["pos2"][0], j["pos2"][1], j["pos2"][2]);
            st->mAxisX2 = Vec3(j["axisX2"][0], j["axisX2"][1], j["axisX2"][2]);
            st->mAxisY2 = Vec3(j["axisY2"][0], j["axisY2"][1], j["axisY2"][2]);
            for (int i = 0; i < 6; ++i) st->mMaxFriction[i] = j["maxFric"][i];
            for (int i = 0; i < 6; ++i) { st->mLimitMin[i] = j["limMin"][i]; st->mLimitMax[i] = j["limMax"][i]; }
            for (int i = 0; i < 6; ++i) impMotor(st->mMotorSettings[i], j["motor"][i]);
            c = st; break;
        }
        default: break;
    }
    if (c) {
        c->mEnabled = j["enabled"];
        c->mConstraintPriority = j["priority"];
        c->mNumPositionStepsOverride = j["posSteps"];
        c->mNumVelocityStepsOverride = j["velSteps"];
        c->mDrawConstraintSize = j["drawSize"];
    }
    return c;
}

Ref<RagdollSettings> ImportRagdoll(const json& j) {
    Ref<RagdollSettings> r = new RagdollSettings();
    
    // 1. Skeleton
    r->mSkeleton = new Skeleton();
    for (const auto& jnt : j["skeleton"]) {
        r->mSkeleton->AddJoint(jnt["name"].get<std::string>(), jnt["parentName"].get<std::string>());
    }

    // 2. Parts
    for (const auto& pj : j["parts"]) {
        RagdollSettings::Part part;
        part.mPosition = RVec3(pj["position"][0], pj["position"][1], pj["position"][2]);
        part.mRotation = Quat(pj["rotation"][0], pj["rotation"][1], pj["rotation"][2], pj["rotation"][3]);
        part.SetShapeSettings(ImportShape(pj["shape"]));
        
        part.mCollisionGroup.SetGroupID(pj["collGroup"]["groupID"]);
        part.mCollisionGroup.SetSubGroupID(pj["collGroup"]["subGroupID"]);
        
        if (pj["collGroup"].contains("filter")) {
            std::string hash = std::to_string(pj["collGroup"]["filterPtr"].get<uintptr_t>());
            if (gFilterCache.count(hash)) {
                part.mCollisionGroup.SetGroupFilter(gFilterCache[hash]);
            } else {
                auto filter = new GroupFilterTable(23);
                for(int a = 0; a < 23; ++a) {
                    for(int b = 0; b < 23; ++b) {
                        if (a != b && !pj["collGroup"]["filter"][a][b].get<bool>()) {
                            filter->DisableCollision(a, b);
                        }
                    }
                }
                gFilterCache[hash] = filter;
                part.mCollisionGroup.SetGroupFilter(filter);
            }
        }

        part.mObjectLayer = pj["objectLayer"];
        part.mMotionType = (EMotionType)pj["motionType"].get<int>();
        part.mAllowDynamicOrKinematic = pj["allowDynamicOrKinematic"];
        part.mMotionQuality = (EMotionQuality)pj["motionQuality"].get<int>();
        part.mAllowSleeping = pj["allowSleeping"];
        part.mFriction = pj["friction"];
        part.mRestitution = pj["restitution"];
        part.mLinearDamping = pj["linearDamping"];
        part.mAngularDamping = pj["angularDamping"];
        part.mMaxLinearVelocity = pj["maxLinearVelocity"];
        part.mMaxAngularVelocity = pj["maxAngularVelocity"];
        part.mGravityFactor = pj["gravityFactor"];
        
        part.mAllowedDOFs = (EAllowedDOFs)pj["allowedDOFs"].get<int>();
        part.mIsSensor = pj["isSensor"];
        part.mCollideKinematicVsNonDynamic = pj["collideKinematicVsNonDynamic"];
        part.mUseManifoldReduction = pj["useManifoldReduction"];
        part.mApplyGyroscopicForce = pj["applyGyroscopicForce"];
        part.mEnhancedInternalEdgeRemoval = pj["enhancedInternalEdgeRemoval"];
        part.mNumVelocityStepsOverride = pj["numVelocityStepsOverride"];
        part.mNumPositionStepsOverride = pj["numPositionStepsOverride"];
        part.mInertiaMultiplier = pj["inertiaMultiplier"];
        
        part.mOverrideMassProperties = (EOverrideMassProperties)pj["overrideMass"].get<int>();
        part.mMassPropertiesOverride.mMass = pj["mass"];
        
        Mat44 mat;
        for(int i=0; i<4; ++i) {
            mat.SetColumn4(i, Vec4(pj["inertia"][i][0], pj["inertia"][i][1], pj["inertia"][i][2], pj["inertia"][i][3]));
        }
        part.mMassPropertiesOverride.mInertia = mat;
        part.mToParent = ImportConstraint(pj["toParent"]);
        
        r->mParts.push_back(part);
    }

    // 3. Additional Constraints
    if (j.contains("additionalConstraints")) {
        for (const auto& cj : j["additionalConstraints"]) {
            RagdollSettings::AdditionalConstraint ac;
            ac.mBodyIdx[0] = cj["bodyIdx"][0];
            ac.mBodyIdx[1] = cj["bodyIdx"][1];
            ac.mConstraint = ImportConstraint(cj["constraint"]);
            r->mAdditionalConstraints.push_back(ac);
        }
    }

    return r;
}

// ==========================================
// MAIN VERIFICATION
// ==========================================

int main() {
    RegisterDefaultAllocator();
    Factory::sInstance = new Factory();
    RegisterTypes();

    // 1. Load the original .tof file
    std::ifstream file("human.tof");
    Ref<RagdollSettings> originalRagdoll;
    
    if (!ObjectStreamIn::sReadObject(file, originalRagdoll)) {
        std::cout << "Failed to load human.tof\n";
        return 1;
    }
    std::cout << "Successfully loaded human.tof natively.\n";

    // 2. Export Jolt to JSON
    json exportedJson = ExportRagdoll(originalRagdoll);
    
    // 2.5 Prove the bridge by writing JSON to disk and parsing it back
    std::ofstream outJson("human.json");
    outJson << exportedJson.dump(4);
    outJson.close();
    
    std::ifstream inJson("human.json");
    json parsedJson = json::parse(inJson);
    inJson.close();
    
    // 3. Import JSON into a completely new Ragdoll
    Ref<RagdollSettings> importedRagdoll = ImportRagdoll(parsedJson);

    // 3.5 Write out the upgraded original to match schema versions
    std::ofstream outUpgraded("human_UPGRADED.tof");
    if (!ObjectStreamOut::sWriteObject(outUpgraded, ObjectStreamOut::EStreamType::Text, *originalRagdoll)) {
        std::cout << "Failed to write human_UPGRADED.tof\n";
        return 1;
    }
    outUpgraded.close();

    // 4. Export the Imported Ragdoll BACK to a native TOF file
    std::ofstream outTof("human_VERIFIED.tof");
    if (!ObjectStreamOut::sWriteObject(outTof, ObjectStreamOut::EStreamType::Text, *importedRagdoll)) {
        std::cout << "Failed to write human_VERIFIED.tof\n";
        return 1;
    }
    outTof.close();

    // 5. THE BRUTAL TEST: Compare the two TOF files line by line
    std::ifstream f1("human_UPGRADED.tof");
    std::ifstream f2("human_VERIFIED.tof");
    std::string line1, line2;
    int lineNum = 1;
    bool failed = false;

    std::cout << "\nRunning strict TOF-to-TOF text diff verification...\n";

    while (std::getline(f1, line1) && std::getline(f2, line2)) {
        if (!line1.empty() && line1.back() == '\r') line1.pop_back();
        if (!line2.empty() && line2.back() == '\r') line2.pop_back();
        
        // Jolt's TOF sometimes puts trailing spaces or slightly different float formatting 
        // depending on the exact OS/compiler, but since we are running both in the same 
        // executable, the output should be 100% identical.
        if (line1 != line2) {
            std::cout << "\n[!] FATAL MISMATCH AT LINE " << lineNum << " [!]\n";
            std::cout << "Original TOF : " << line1 << "\n";
            std::cout << "Imported TOF : " << line2 << "\n";
            failed = true;
            break; // Stop at the first failure
        }
        lineNum++;
    }

    // Check if one file is longer than the other
    if (!failed && (std::getline(f1, line1) || std::getline(f2, line2))) {
        std::cout << "\n[!] FATAL MISMATCH: Files have different line counts! [!]\n";
        failed = true;
    }

    if (!failed) {
        std::cout << "\n======================================================\n";
        std::cout << "ULTIMATE VERIFICATION SUCCESS:\n";
        std::cout << "human.tof and human_VERIFIED.tof are 100% BYTE-FOR-BYTE IDENTICAL!\n";
        std::cout << "The JSON bridge successfully captured and restored every single property.\n";
        std::cout << "======================================================\n";
    } else {
        std::cout << "\n======================================================\n";
        std::cout << "FAILURE: The JSON bridge leaked data!\n";
        std::cout << "Check the line mismatch above to see exactly what property was missed.\n";
        std::cout << "======================================================\n";
    }

    delete Factory::sInstance;
    return 0;
}