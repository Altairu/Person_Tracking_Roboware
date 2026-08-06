// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from altair_interfaces:srv/ConnectCan.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "altair_interfaces/srv/detail/connect_can__struct.h"
#include "altair_interfaces/srv/detail/connect_can__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace altair_interfaces
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _ConnectCan_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ConnectCan_Request_type_support_ids_t;

static const _ConnectCan_Request_type_support_ids_t _ConnectCan_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _ConnectCan_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ConnectCan_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ConnectCan_Request_type_support_symbol_names_t _ConnectCan_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, altair_interfaces, srv, ConnectCan_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, altair_interfaces, srv, ConnectCan_Request)),
  }
};

typedef struct _ConnectCan_Request_type_support_data_t
{
  void * data[2];
} _ConnectCan_Request_type_support_data_t;

static _ConnectCan_Request_type_support_data_t _ConnectCan_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ConnectCan_Request_message_typesupport_map = {
  2,
  "altair_interfaces",
  &_ConnectCan_Request_message_typesupport_ids.typesupport_identifier[0],
  &_ConnectCan_Request_message_typesupport_symbol_names.symbol_name[0],
  &_ConnectCan_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ConnectCan_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ConnectCan_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace altair_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, altair_interfaces, srv, ConnectCan_Request)() {
  return &::altair_interfaces::srv::rosidl_typesupport_c::ConnectCan_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "altair_interfaces/srv/detail/connect_can__struct.h"
// already included above
// #include "altair_interfaces/srv/detail/connect_can__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace altair_interfaces
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _ConnectCan_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ConnectCan_Response_type_support_ids_t;

static const _ConnectCan_Response_type_support_ids_t _ConnectCan_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _ConnectCan_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ConnectCan_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ConnectCan_Response_type_support_symbol_names_t _ConnectCan_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, altair_interfaces, srv, ConnectCan_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, altair_interfaces, srv, ConnectCan_Response)),
  }
};

typedef struct _ConnectCan_Response_type_support_data_t
{
  void * data[2];
} _ConnectCan_Response_type_support_data_t;

static _ConnectCan_Response_type_support_data_t _ConnectCan_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ConnectCan_Response_message_typesupport_map = {
  2,
  "altair_interfaces",
  &_ConnectCan_Response_message_typesupport_ids.typesupport_identifier[0],
  &_ConnectCan_Response_message_typesupport_symbol_names.symbol_name[0],
  &_ConnectCan_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ConnectCan_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ConnectCan_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace altair_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, altair_interfaces, srv, ConnectCan_Response)() {
  return &::altair_interfaces::srv::rosidl_typesupport_c::ConnectCan_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "altair_interfaces/srv/detail/connect_can__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace altair_interfaces
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _ConnectCan_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ConnectCan_type_support_ids_t;

static const _ConnectCan_type_support_ids_t _ConnectCan_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _ConnectCan_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ConnectCan_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ConnectCan_type_support_symbol_names_t _ConnectCan_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, altair_interfaces, srv, ConnectCan)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, altair_interfaces, srv, ConnectCan)),
  }
};

typedef struct _ConnectCan_type_support_data_t
{
  void * data[2];
} _ConnectCan_type_support_data_t;

static _ConnectCan_type_support_data_t _ConnectCan_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ConnectCan_service_typesupport_map = {
  2,
  "altair_interfaces",
  &_ConnectCan_service_typesupport_ids.typesupport_identifier[0],
  &_ConnectCan_service_typesupport_symbol_names.symbol_name[0],
  &_ConnectCan_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t ConnectCan_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ConnectCan_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace altair_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, altair_interfaces, srv, ConnectCan)() {
  return &::altair_interfaces::srv::rosidl_typesupport_c::ConnectCan_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
