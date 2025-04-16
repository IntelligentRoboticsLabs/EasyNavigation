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
/// \brief Definition of the ConfigurationValue class used to store multiple types of configuration data.

#ifndef EASYNAV_CORE__CONFIGURATION_HPP_
#define EASYNAV_CORE__CONFIGURATION_HPP_

#include <string>
#include <vector>
#include <variant>
#include <stdexcept>

namespace easynav_common
{

/**
 * @class ConfigurationValue
 * @brief Holds a configuration value that can be a string, vector of strings, double, or vector of doubles.
 *
 * This class provides type-safe access to a variant configuration value, allowing runtime
 * inspection and extraction based on the stored type.
 */
class ConfigurationValue {
public:
  /// @brief The variant type that holds the actual value.
  using ValueType = std::variant<
    std::string,
    std::vector<std::string>,
    double,
    std::vector<double>
  >;

  /// @brief Default constructor.
  ConfigurationValue();

  /**
   * @brief Construct a ConfigurationValue from any supported type.
   * @tparam T One of the supported types: std::string, std::vector<std::string>, double, std::vector<double>.
   * @param value The value to store.
   */
  template<typename T>
  ConfigurationValue(T value)
  : value_(std::move(value)) {}

  /**
   * @brief Get the internal std::variant storing the value.
   * @return A const reference to the underlying variant.
   */
  const ValueType & get_variant() const {return value_;}

  /// @brief Check if the stored value is a string.
  /// @return true if it is a string.
  bool is_string() const;

  /// @brief Check if the stored value is a vector of strings.
  /// @return true if it is a vector of strings.
  bool is_string_vector() const;

  /// @brief Check if the stored value is a double.
  /// @return true if it is a double.
  bool is_double() const;

  /// @brief Check if the stored value is a vector of doubles.
  /// @return true if it is a vector of doubles.
  bool is_double_vector() const;

  /// @brief Get the stored value as a string.
  /// @return A const reference to the string.
  const std::string & as_string() const;

  /// @brief Get the stored value as a vector of strings.
  /// @return A const reference to the vector of strings.
  const std::vector<std::string> & as_string_vector() const;

  /// @brief Get the stored value as a double.
  /// @return The double value.
  double as_double() const;

  /// @brief Get the stored value as a vector of doubles.
  /// @return A const reference to the vector of doubles.
  const std::vector<double> & as_double_vector() const;

  /// @brief Safely try to access the value as a string.
  /// @return A pointer to the string if the type matches, otherwise nullptr.
  const std::string * try_string() const;

  /// @brief Safely try to access the value as a vector of strings.
  /// @return A pointer to the vector if the type matches, otherwise nullptr.
  const std::vector<std::string> * try_string_vector() const;

  /// @brief Safely try to access the value as a double.
  /// @return A pointer to the double if the type matches, otherwise nullptr.
  const double * try_double() const;

  /// @brief Safely try to access the value as a vector of doubles.
  /// @return A pointer to the vector if the type matches, otherwise nullptr.
  const std::vector<double> * try_double_vector() const;

private:
  ValueType value_;
};

}  // namespace easynav_common

#endif  // EASYNAV_CORE__CONFIGURATION_HPP_
