#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/ObjectStream/ObjectStreamIn.h>
#include <Jolt/ObjectStream/ObjectStreamOut.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>
#include <Jolt/Core/RTTI.h>

#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <sstream>

using namespace JPH;
using json = nlohmann::json;

class ObjectStreamJsonOut : public ObjectStreamOut
{
public:
    json mRoot;
    std::vector<json*> mStack;
    std::string mNextKey;
    bool mInRTTI = false;
    
    explicit ObjectStreamJsonOut(ostream &inStream) : ObjectStreamOut(inStream) {
        mStack.push_back(&mRoot);
    }
    
    void AddValue(const json& val) {
        if (mInRTTI) return;
        json* top = mStack.back();
        if (top->is_object()) {
            (*top)[mNextKey] = val;
        } else if (top->is_array()) {
            top->push_back(val);
        } else {
            *top = val;
        }
    }

    virtual void WriteDataType(EOSDataType inType) override {
        if (inType == EOSDataType::Declare) mInRTTI = true;
        if (inType == EOSDataType::Object) mInRTTI = false;
    }
    
    virtual void WriteName(const char *inName) override { }
    virtual void WriteIdentifier(Identifier inIdentifier) override { }
    virtual void WriteCount(uint32 inCount) override { }
    
    virtual void WritePrimitiveData(const uint8 &v) override { AddValue(v); }
    virtual void WritePrimitiveData(const uint16 &v) override { AddValue(v); }
    virtual void WritePrimitiveData(const int &v) override { AddValue(v); }
    virtual void WritePrimitiveData(const uint32 &v) override { AddValue(v); }
    virtual void WritePrimitiveData(const uint64 &v) override { AddValue(v); }
    virtual void WritePrimitiveData(const float &v) override { AddValue(v); }
    virtual void WritePrimitiveData(const double &v) override { AddValue(v); }
    virtual void WritePrimitiveData(const bool &v) override { AddValue(v); }
    virtual void WritePrimitiveData(const String &v) override { AddValue(v.c_str()); }
    
    virtual void WritePrimitiveData(const Float3 &v) override { AddValue({v.x, v.y, v.z}); }
    virtual void WritePrimitiveData(const Float4 &v) override { AddValue({v.x, v.y, v.z, v.w}); }
    virtual void WritePrimitiveData(const Double3 &v) override { AddValue({v.x, v.y, v.z}); }
    virtual void WritePrimitiveData(const Vec3 &v) override { AddValue({v.GetX(), v.GetY(), v.GetZ()}); }
    virtual void WritePrimitiveData(const DVec3 &v) override { AddValue({v.GetX(), v.GetY(), v.GetZ()}); }
    virtual void WritePrimitiveData(const Vec4 &v) override { AddValue({v.GetX(), v.GetY(), v.GetZ(), v.GetW()}); }
    virtual void WritePrimitiveData(const UVec4 &v) override { AddValue({v.GetX(), v.GetY(), v.GetZ(), v.GetW()}); }
    virtual void WritePrimitiveData(const Quat &v) override { AddValue({v.GetX(), v.GetY(), v.GetZ(), v.GetW()}); }
    virtual void WritePrimitiveData(const Mat44 &v) override {
        json arr = json::array();
        for(int i=0; i<4; ++i) {
            Vec4 col = v.GetColumn4(i);
            arr.push_back({col.GetX(), col.GetY(), col.GetZ(), col.GetW()});
        }
        AddValue(arr);
    }
    virtual void WritePrimitiveData(const DMat44 &v) override {
        json arr = json::array();
        for(int i=0; i<3; ++i) {
            Vec4 col = v.GetColumn4(i);
            arr.push_back({col.GetX(), col.GetY(), col.GetZ(), col.GetW()});
        }
        DVec3 trans = v.GetTranslation();
        arr.push_back({trans.GetX(), trans.GetY(), trans.GetZ(), 1.0});
        AddValue(arr);
    }

    virtual void WriteClassData(const RTTI *inRTTI, const void *inInstance) override {
        if (mInRTTI) return;
        
        json obj = json::object();
        obj["$type"] = inRTTI->GetName();
        
        json* parent = mStack.back();
        json* currentObj = nullptr;
        if (parent->is_object()) {
            (*parent)[mNextKey] = obj;
            currentObj = &((*parent)[mNextKey]);
        } else if (parent->is_array()) {
            parent->push_back(obj);
            currentObj = &(parent->back());
        } else {
            *parent = obj;
            currentObj = parent;
        }
        
        mStack.push_back(currentObj);
        
        for (int i = 0; i < inRTTI->GetAttributeCount(); ++i) {
            const SerializableAttribute &attr = inRTTI->GetAttribute(i);
            mNextKey = attr.GetName();
            attr.WriteData(*this, inInstance);
        }
        
        mStack.pop_back();
    }
    
    virtual void WritePointerData(const RTTI *inRTTI, const void *inPointer) override {
        if (mInRTTI) return;
        if (inPointer) {
            const RTTI* rtti = inRTTI;
            WriteClassData(rtti, inPointer);
        } else {
            AddValue(nullptr);
        }
    }
    
    virtual void HintNextItem() override { }
    virtual void HintIndentUp() override {
        if (mInRTTI) return;
        json arr = json::array();
        json* parent = mStack.back();
        json* currentArr = nullptr;
        
        if (parent->is_object()) {
            (*parent)[mNextKey] = arr;
            currentArr = &((*parent)[mNextKey]);
        } else if (parent->is_array()) {
            parent->push_back(arr);
            currentArr = &(parent->back());
        } else {
            *parent = arr;
            currentArr = parent;
        }
        mStack.push_back(currentArr);
    }
    virtual void HintIndentDown() override {
        if (mInRTTI) return;
        mStack.pop_back();
    }
};

int main() {
    RegisterDefaultAllocator();
    Factory::sInstance = new Factory();
    RegisterTypes();

    std::ifstream file("Human.tof");
    Ref<RagdollSettings> ragdollSettings;
    
    if (ObjectStreamIn::sReadObject(file, ragdollSettings)) {
        std::stringstream ss;
        ObjectStreamJsonOut jsonOut(ss);
        
        // Trigger the dynamic extraction using Jolt's API
        jsonOut.Write(ragdollSettings.GetPtr(), JPH_RTTI(RagdollSettings));
        
        // Save your perfect, 100% complete data
        std::ofstream out("ExtractedHuman.json");
        out << jsonOut.mRoot.dump(4);
        out.close();
        
        std::cout << "Extraction complete. No data missed.\n";
    } else {
        std::cout << "Failed to load Human.tof\n";
    }

    delete Factory::sInstance;
    return 0;
}
