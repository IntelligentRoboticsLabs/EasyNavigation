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

/// \file
/// \brief Implementation of the ConfigurationValue class.


#include <string>
#include <vector>
#include <variant>
#include <stdexcept>

#include "easynav_common/Configuration.hpp"

namespace easynav_common
{

ConfigurationValue::ConfigurationValue()
{
}

bool
ConfigurationValue::is_string() const
{
  return std::holds_alternative<std::string>(value_);
}

bool
ConfigurationValue::is_string_vector() const
{
  return std::holds_alternative<std::vector<std::string>>(value_);
}

bool
ConfigurationValue::is_double() const
{
  return std::holds_alternative<double>(value_);
}

bool
ConfigurationValue::is_double_vector() const
{
  return std::holds_alternative<std::vector<double>>(value_);
}

const
std::string & ConfigurationValue::as_string() const
{
  return std::get<std::string>(value_);
}

const
std::vector<std::string> & ConfigurationValue::as_string_vector() const
{
  return std::get<std::vector<std::string>>(value_);
}

double
ConfigurationValue::as_double() const
{
  return std::get<double>(value_);
}

const
std::vector<double> & ConfigurationValue::as_double_vector() const
{
  return std::get<std::vector<double>>(value_);
}

const
std::string * ConfigurationValue::try_string() const
{
  return std::get_if<std::string>(&value_);
}

const
std::vector<std::string> * ConfigurationValue::try_string_vector() const
{
  return std::get_if<std::vector<std::string>>(&value_);
}

const double *
ConfigurationValue::try_double() const
{
  return std::get_if<double>(&value_);
}

const std::vector<double> *
ConfigurationValue::try_double_vector() const
{
  return std::get_if<std::vector<double>>(&value_);
}

}  // namespace easynav_common
