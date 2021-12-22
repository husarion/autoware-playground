// Copyright 2020 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
/// \file
/// \brief Visibility contorl
#ifndef panther_interface__VISIBILITY_CONTROL_HPP_
#define panther_interface__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define panther_interface_EXPORT __attribute__ ((dllexport))
    #define panther_interface_IMPORT __attribute__ ((dllimport))
  #else
    #define panther_interface_EXPORT __declspec(dllexport)
    #define panther_interface_IMPORT __declspec(dllimport)
  #endif
  #ifdef panther_interface_BUILDING_LIBRARY
    #define panther_interface_PUBLIC panther_interface_EXPORT
  #else
    #define panther_interface_PUBLIC panther_interface_IMPORT
  #endif
  #define panther_interface_PUBLIC_TYPE panther_interface_PUBLIC
  #define panther_interface_LOCAL
#else
  #define panther_interface_EXPORT __attribute__ ((visibility("default")))
  #define panther_interface_IMPORT
  #if __GNUC__ >= 4
    #define panther_interface_PUBLIC __attribute__ ((visibility("default")))
    #define panther_interface_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define panther_interface_PUBLIC
    #define panther_interface_LOCAL
  #endif
  #define panther_interface_PUBLIC_TYPE
#endif

#endif  // panther_interface__VISIBILITY_CONTROL_HPP_
