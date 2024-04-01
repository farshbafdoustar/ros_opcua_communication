//
// DO NOT EDIT THIS FILE!
// It is automatically generated from opcfoundation.org schemas.
//

#include <sstream>
#include <string>

#include "opc/ua/protocol/attribute_ids.h"

namespace OpcUa
{

std::string ToString(const AttributeId & value)
{
  switch (value)
  {
    case AttributeId::NodeId:
      return "NodeId";
    case AttributeId::NodeClass:
      return "NodeClass";
    case AttributeId::BrowseName:
      return "BrowseName";
    case AttributeId::DisplayName:
      return "DisplayName";
    case AttributeId::Description:
      return "Description";
    case AttributeId::WriteMask:
      return "WriteMask";
    case AttributeId::UserWriteMask:
      return "UserWriteMask";
    case AttributeId::IsAbstract:
      return "IsAbstract";
    case AttributeId::Symmetric:
      return "Symmetric";
    case AttributeId::InverseName:
      return "InverseName";
    case AttributeId::ContainsNoLoops:
      return "ContainsNoLoops";
    case AttributeId::EventNotifier:
      return "EventNotifier";
    case AttributeId::Value:
      return "Value";
    case AttributeId::DataType:
      return "DataType";
    case AttributeId::ValueRank:
      return "ValueRank";
    case AttributeId::ArrayDimensions:
      return "ArrayDimensions";
    case AttributeId::AccessLevel:
      return "AccessLevel";
    case AttributeId::UserAccessLevel:
      return "UserAccessLevel";
    case AttributeId::MinimumSamplingInterval:
      return "MinimumSamplingInterval";
    case AttributeId::Historizing:
      return "Historizing";
    case AttributeId::Executable:
      return "Executable";
    case AttributeId::UserExecutable:
      return "UserExecutable";
    default:
      {
        std::stringstream result;
        result << "unknown(" << static_cast<int>(value) << ")";
        return result.str();
      }
  }
}

} // namespace OpcUa

