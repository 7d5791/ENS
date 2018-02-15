//
// Generated file, do not edit! Created by nedtool 4.6 from veins/modules/messages/VideoStreamMessage.msg.
//

// Disable warnings about unused variables, empty switch stmts, etc:
#ifdef _MSC_VER
#  pragma warning(disable:4101)
#  pragma warning(disable:4065)
#endif

#include <iostream>
#include <sstream>
#include "VideoStreamMessage_m.h"

USING_NAMESPACE


// Another default rule (prevents compiler from choosing base class' doPacking())
template<typename T>
void doPacking(cCommBuffer *, T& t) {
    throw cRuntimeError("Parsim error: no doPacking() function for type %s or its base class (check .msg and _m.cc/h files!)",opp_typename(typeid(t)));
}

template<typename T>
void doUnpacking(cCommBuffer *, T& t) {
    throw cRuntimeError("Parsim error: no doUnpacking() function for type %s or its base class (check .msg and _m.cc/h files!)",opp_typename(typeid(t)));
}




// Template rule for outputting std::vector<T> types
template<typename T, typename A>
inline std::ostream& operator<<(std::ostream& out, const std::vector<T,A>& vec)
{
    out.put('{');
    for(typename std::vector<T,A>::const_iterator it = vec.begin(); it != vec.end(); ++it)
    {
        if (it != vec.begin()) {
            out.put(','); out.put(' ');
        }
        out << *it;
    }
    out.put('}');
    
    char buf[32];
    sprintf(buf, " (size=%u)", (unsigned int)vec.size());
    out.write(buf, strlen(buf));
    return out;
}

// Template rule which fires if a struct or class doesn't have operator<<
template<typename T>
inline std::ostream& operator<<(std::ostream& out,const T&) {return out;}

Register_Class(VideoStreamMessage);

VideoStreamMessage::VideoStreamMessage(const char *name, int kind) : ::WaveShortMessage(name,kind)
{
    this->Marker_var = 0;
    this->sequenceNumber_var = 0;
    this->timestamp_var = 0;
    this->fragmentStart_var = 0;
    this->fragmentEnd_var = 0;
    this->frameNumber_var = 0;
    this->frameTime_var = 0;
    this->frameType_var = 0;
}

VideoStreamMessage::VideoStreamMessage(const VideoStreamMessage& other) : ::WaveShortMessage(other)
{
    copy(other);
}

VideoStreamMessage::~VideoStreamMessage()
{
}

VideoStreamMessage& VideoStreamMessage::operator=(const VideoStreamMessage& other)
{
    if (this==&other) return *this;
    ::WaveShortMessage::operator=(other);
    copy(other);
    return *this;
}

void VideoStreamMessage::copy(const VideoStreamMessage& other)
{
    this->Marker_var = other.Marker_var;
    this->sequenceNumber_var = other.sequenceNumber_var;
    this->timestamp_var = other.timestamp_var;
    this->fragmentStart_var = other.fragmentStart_var;
    this->fragmentEnd_var = other.fragmentEnd_var;
    this->frameNumber_var = other.frameNumber_var;
    this->frameTime_var = other.frameTime_var;
    this->frameType_var = other.frameType_var;
}

void VideoStreamMessage::parsimPack(cCommBuffer *b)
{
    ::WaveShortMessage::parsimPack(b);
    doPacking(b,this->Marker_var);
    doPacking(b,this->sequenceNumber_var);
    doPacking(b,this->timestamp_var);
    doPacking(b,this->fragmentStart_var);
    doPacking(b,this->fragmentEnd_var);
    doPacking(b,this->frameNumber_var);
    doPacking(b,this->frameTime_var);
    doPacking(b,this->frameType_var);
}

void VideoStreamMessage::parsimUnpack(cCommBuffer *b)
{
    ::WaveShortMessage::parsimUnpack(b);
    doUnpacking(b,this->Marker_var);
    doUnpacking(b,this->sequenceNumber_var);
    doUnpacking(b,this->timestamp_var);
    doUnpacking(b,this->fragmentStart_var);
    doUnpacking(b,this->fragmentEnd_var);
    doUnpacking(b,this->frameNumber_var);
    doUnpacking(b,this->frameTime_var);
    doUnpacking(b,this->frameType_var);
}

bool VideoStreamMessage::getMarker() const
{
    return Marker_var;
}

void VideoStreamMessage::setMarker(bool Marker)
{
    this->Marker_var = Marker;
}

uint16_t VideoStreamMessage::getSequenceNumber() const
{
    return sequenceNumber_var;
}

void VideoStreamMessage::setSequenceNumber(uint16_t sequenceNumber)
{
    this->sequenceNumber_var = sequenceNumber;
}

uint32_t VideoStreamMessage::getTimestamp() const
{
    return timestamp_var;
}

void VideoStreamMessage::setTimestamp(uint32_t timestamp)
{
    this->timestamp_var = timestamp;
}

bool VideoStreamMessage::getFragmentStart() const
{
    return fragmentStart_var;
}

void VideoStreamMessage::setFragmentStart(bool fragmentStart)
{
    this->fragmentStart_var = fragmentStart;
}

bool VideoStreamMessage::getFragmentEnd() const
{
    return fragmentEnd_var;
}

void VideoStreamMessage::setFragmentEnd(bool fragmentEnd)
{
    this->fragmentEnd_var = fragmentEnd;
}

long VideoStreamMessage::getFrameNumber() const
{
    return frameNumber_var;
}

void VideoStreamMessage::setFrameNumber(long frameNumber)
{
    this->frameNumber_var = frameNumber;
}

double VideoStreamMessage::getFrameTime() const
{
    return frameTime_var;
}

void VideoStreamMessage::setFrameTime(double frameTime)
{
    this->frameTime_var = frameTime;
}

int VideoStreamMessage::getFrameType() const
{
    return frameType_var;
}

void VideoStreamMessage::setFrameType(int frameType)
{
    this->frameType_var = frameType;
}

class VideoStreamMessageDescriptor : public cClassDescriptor
{
  public:
    VideoStreamMessageDescriptor();
    virtual ~VideoStreamMessageDescriptor();

    virtual bool doesSupport(cObject *obj) const;
    virtual const char *getProperty(const char *propertyname) const;
    virtual int getFieldCount(void *object) const;
    virtual const char *getFieldName(void *object, int field) const;
    virtual int findField(void *object, const char *fieldName) const;
    virtual unsigned int getFieldTypeFlags(void *object, int field) const;
    virtual const char *getFieldTypeString(void *object, int field) const;
    virtual const char *getFieldProperty(void *object, int field, const char *propertyname) const;
    virtual int getArraySize(void *object, int field) const;

    virtual std::string getFieldAsString(void *object, int field, int i) const;
    virtual bool setFieldAsString(void *object, int field, int i, const char *value) const;

    virtual const char *getFieldStructName(void *object, int field) const;
    virtual void *getFieldStructPointer(void *object, int field, int i) const;
};

Register_ClassDescriptor(VideoStreamMessageDescriptor);

VideoStreamMessageDescriptor::VideoStreamMessageDescriptor() : cClassDescriptor("VideoStreamMessage", "WaveShortMessage")
{
}

VideoStreamMessageDescriptor::~VideoStreamMessageDescriptor()
{
}

bool VideoStreamMessageDescriptor::doesSupport(cObject *obj) const
{
    return dynamic_cast<VideoStreamMessage *>(obj)!=NULL;
}

const char *VideoStreamMessageDescriptor::getProperty(const char *propertyname) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? basedesc->getProperty(propertyname) : NULL;
}

int VideoStreamMessageDescriptor::getFieldCount(void *object) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? 8+basedesc->getFieldCount(object) : 8;
}

unsigned int VideoStreamMessageDescriptor::getFieldTypeFlags(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldTypeFlags(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static unsigned int fieldTypeFlags[] = {
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
    };
    return (field>=0 && field<8) ? fieldTypeFlags[field] : 0;
}

const char *VideoStreamMessageDescriptor::getFieldName(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldName(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static const char *fieldNames[] = {
        "Marker",
        "sequenceNumber",
        "timestamp",
        "fragmentStart",
        "fragmentEnd",
        "frameNumber",
        "frameTime",
        "frameType",
    };
    return (field>=0 && field<8) ? fieldNames[field] : NULL;
}

int VideoStreamMessageDescriptor::findField(void *object, const char *fieldName) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    int base = basedesc ? basedesc->getFieldCount(object) : 0;
    if (fieldName[0]=='M' && strcmp(fieldName, "Marker")==0) return base+0;
    if (fieldName[0]=='s' && strcmp(fieldName, "sequenceNumber")==0) return base+1;
    if (fieldName[0]=='t' && strcmp(fieldName, "timestamp")==0) return base+2;
    if (fieldName[0]=='f' && strcmp(fieldName, "fragmentStart")==0) return base+3;
    if (fieldName[0]=='f' && strcmp(fieldName, "fragmentEnd")==0) return base+4;
    if (fieldName[0]=='f' && strcmp(fieldName, "frameNumber")==0) return base+5;
    if (fieldName[0]=='f' && strcmp(fieldName, "frameTime")==0) return base+6;
    if (fieldName[0]=='f' && strcmp(fieldName, "frameType")==0) return base+7;
    return basedesc ? basedesc->findField(object, fieldName) : -1;
}

const char *VideoStreamMessageDescriptor::getFieldTypeString(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldTypeString(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static const char *fieldTypeStrings[] = {
        "bool",
        "uint16_t",
        "uint32_t",
        "bool",
        "bool",
        "long",
        "double",
        "int",
    };
    return (field>=0 && field<8) ? fieldTypeStrings[field] : NULL;
}

const char *VideoStreamMessageDescriptor::getFieldProperty(void *object, int field, const char *propertyname) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldProperty(object, field, propertyname);
        field -= basedesc->getFieldCount(object);
    }
    switch (field) {
        case 7:
            if (!strcmp(propertyname,"enum")) return "FrameType";
            return NULL;
        default: return NULL;
    }
}

int VideoStreamMessageDescriptor::getArraySize(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getArraySize(object, field);
        field -= basedesc->getFieldCount(object);
    }
    VideoStreamMessage *pp = (VideoStreamMessage *)object; (void)pp;
    switch (field) {
        default: return 0;
    }
}

std::string VideoStreamMessageDescriptor::getFieldAsString(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldAsString(object,field,i);
        field -= basedesc->getFieldCount(object);
    }
    VideoStreamMessage *pp = (VideoStreamMessage *)object; (void)pp;
    switch (field) {
        case 0: return bool2string(pp->getMarker());
        case 1: return ulong2string(pp->getSequenceNumber());
        case 2: return ulong2string(pp->getTimestamp());
        case 3: return bool2string(pp->getFragmentStart());
        case 4: return bool2string(pp->getFragmentEnd());
        case 5: return long2string(pp->getFrameNumber());
        case 6: return double2string(pp->getFrameTime());
        case 7: return long2string(pp->getFrameType());
        default: return "";
    }
}

bool VideoStreamMessageDescriptor::setFieldAsString(void *object, int field, int i, const char *value) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->setFieldAsString(object,field,i,value);
        field -= basedesc->getFieldCount(object);
    }
    VideoStreamMessage *pp = (VideoStreamMessage *)object; (void)pp;
    switch (field) {
        case 0: pp->setMarker(string2bool(value)); return true;
        case 1: pp->setSequenceNumber(string2ulong(value)); return true;
        case 2: pp->setTimestamp(string2ulong(value)); return true;
        case 3: pp->setFragmentStart(string2bool(value)); return true;
        case 4: pp->setFragmentEnd(string2bool(value)); return true;
        case 5: pp->setFrameNumber(string2long(value)); return true;
        case 6: pp->setFrameTime(string2double(value)); return true;
        case 7: pp->setFrameType(string2long(value)); return true;
        default: return false;
    }
}

const char *VideoStreamMessageDescriptor::getFieldStructName(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldStructName(object, field);
        field -= basedesc->getFieldCount(object);
    }
    switch (field) {
        default: return NULL;
    };
}

void *VideoStreamMessageDescriptor::getFieldStructPointer(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldStructPointer(object, field, i);
        field -= basedesc->getFieldCount(object);
    }
    VideoStreamMessage *pp = (VideoStreamMessage *)object; (void)pp;
    switch (field) {
        default: return NULL;
    }
}


