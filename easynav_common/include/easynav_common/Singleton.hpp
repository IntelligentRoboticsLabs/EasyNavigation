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


#ifndef EASYNAV_COMMON__SINGLETON_H_
#define EASYNAV_COMMON__SINGLETON_H_

#include <memory>
#include <mutex>
#include <utility>

namespace easynav
{

template<class C>
class Singleton
{
public:
  template<typename ... Args>
  static C * getInstance(Args &&... args)
  {
    std::call_once(init_flag_, [&]() {
        instance_ = std::make_unique<C>(std::forward<Args>(args)...);
    });
    return instance_.get();
  }

  static void removeInstance()
  {
    instance_.reset();
    init_flag_ = std::once_flag();
  }

  template<typename ... Args>
  static C & get(Args &&... args)
  {
    getInstance(std::forward<Args>(args)...);
    return *instance_;
  }

protected:
  Singleton() = default;
  ~Singleton() = default;

  Singleton(const Singleton &) = delete;
  Singleton & operator=(const Singleton &) = delete;

private:
  static std::unique_ptr<C> instance_;
  static std::once_flag init_flag_;
};

template<class C>
std::unique_ptr<C> Singleton<C>::instance_ = nullptr;

template<class C>
std::once_flag Singleton<C>::init_flag_;

#define SINGLETON_DEFINITIONS(ClassName) \
public: \
  static ClassName * getInstance() \
  { \
    return ::easynav::Singleton<ClassName>::getInstance(); \
  } \
  template<typename ... Args> \
  static ClassName * getInstance(Args && ... args) \
  { \
    return ::easynav::Singleton<ClassName>::getInstance( \
      std::forward<Args>(args)...); \
  }

}  // namespace easynav

#endif  // EASYNAV_COMMON__SINGLETON_H_
