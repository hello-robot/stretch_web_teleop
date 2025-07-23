// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from stretch_web_teleop:msg/TextToSpeech.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "stretch_web_teleop/msg/detail/text_to_speech__struct.h"
#include "stretch_web_teleop/msg/detail/text_to_speech__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool stretch_web_teleop__msg__text_to_speech__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[52];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("stretch_web_teleop.msg._text_to_speech.TextToSpeech", full_classname_dest, 51) == 0);
  }
  stretch_web_teleop__msg__TextToSpeech * ros_message = _ros_message;
  {  // text
    PyObject * field = PyObject_GetAttrString(_pymsg, "text");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->text, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // voice
    PyObject * field = PyObject_GetAttrString(_pymsg, "voice");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->voice, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // is_slow
    PyObject * field = PyObject_GetAttrString(_pymsg, "is_slow");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->is_slow = (Py_True == field);
    Py_DECREF(field);
  }
  {  // override_behavior
    PyObject * field = PyObject_GetAttrString(_pymsg, "override_behavior");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->override_behavior = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * stretch_web_teleop__msg__text_to_speech__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of TextToSpeech */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("stretch_web_teleop.msg._text_to_speech");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "TextToSpeech");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  stretch_web_teleop__msg__TextToSpeech * ros_message = (stretch_web_teleop__msg__TextToSpeech *)raw_ros_message;
  {  // text
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->text.data,
      strlen(ros_message->text.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "text", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // voice
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->voice.data,
      strlen(ros_message->voice.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "voice", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // is_slow
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->is_slow ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "is_slow", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // override_behavior
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->override_behavior);
    {
      int rc = PyObject_SetAttrString(_pymessage, "override_behavior", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
