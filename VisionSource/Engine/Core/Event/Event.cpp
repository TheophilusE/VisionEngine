
#include "Event.h"

namespace Vision
{
Event::Event(const Type& type, const Variant& data)
    : m_Type(type)
    , m_Data(data)
{
}

}