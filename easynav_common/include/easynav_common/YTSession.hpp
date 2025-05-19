// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.
//
// Easy Navigation program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.


#ifndef EASYNAV_COMMON_TYPES__YTSESSION_HPP_
#define EASYNAV_COMMON_TYPES__YTSESSION_HPP_

#include "easynav_common/Singleton.hpp"

#include "yaets/tracing.hpp"

namespace easynav
{

/**
 * @class YTSession
 * @brief A Yaets Tracing Session
 */
class YTSession : public yaets::TraceSession, public Singleton<YTSession>
{
public:
  explicit YTSession()
  : yaets::TraceSession("/tmp/easynav.log")
  {}

  ~YTSession()
  {
    stop();
  }

  SINGLETON_DEFINITIONS(YTSession)
};

#ifdef EASYNAV_DEBUG_WITH_YAETS
  #define EASYNAV_TRACE_EVENT TRACE_EVENT(YTSession::get())
  #define EASYNAV_TRACE_NAMED_EVENT(name) yaets::TraceGuard guard(YTSession::get(), name);
#else
  #define EASYNAV_TRACE_EVENT ((void)0)
  #define EASYNAV_TRACE_NAMED_EVENT(name) ((void)0)
#endif

}  // namespace easynav


#endif  // EASYNAV_COMMON_TYPES__YTSESSION_HPP_
