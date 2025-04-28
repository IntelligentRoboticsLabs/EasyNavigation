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
/// \brief Definition of the Result class, a generic outcome container inspired by Rust's Result<T, E>.

#ifndef EASYNAV_COMMON__RESULT_HPP_
#define EASYNAV_COMMON__RESULT_HPP_

#include <variant>
#include <functional>
#include <stdexcept>

namespace easynav
{

/**
 * @brief Generic result type that holds either a value (T) or an error (E).
 *
 * This class is a type-safe alternative to exceptions and error codes, providing a functional
 * style for composing and handling success or failure states in C++.
 *
 * @tparam T Success value type.
 * @tparam E Error value type.
 */
template<typename T, typename E>
class Result {
public:
  /// @brief Variant type that stores either a success value or an error.
  using ValueType = std::variant<T, E>;

  /**
   * @brief Creates a successful result.
   * @param value The value to store.
   * @return A Result<T, E> containing the value.
   */
  static Result Ok(T value)
  {
    return Result(std::move(value));
  }

  /**
   * @brief Creates a failed result.
   * @param error The error to store.
   * @return A Result<T, E> containing the error.
   */
  static Result Err(E error)
  {
    return Result(std::move(error));
  }

  /**
   * @brief Checks whether the result contains a valid value.
   * @return true if the result is Ok.
   */
  bool has_value() const
  {
    return std::holds_alternative<T>(value_);
  }

  /**
   * @brief Checks whether the result contains an error.
   * @return true if the result is Err.
   */
  bool has_error() const
  {
    return std::holds_alternative<E>(value_);
  }

  /**
   * @brief Returns the value if the result is Ok.
   * @throws std::logic_error if the result contains an error.
   * @return A const reference to the stored value.
   */
  const T & value() const
  {
    if (!has_value()) {
      throw std::logic_error("Called value() on Err");
    }
    return std::get<T>(value_);
  }

  /**
   * @brief Returns the error if the result is Err.
   * @throws std::logic_error if the result contains a value.
   * @return A const reference to the stored error.
   */
  const E & error() const
  {
    if (!has_error()) {
      throw std::logic_error("Called error() on Ok");
    }
    return std::get<E>(value_);
  }

  /**
   * @brief Returns the stored value or a fallback if the result is Err.
   * @param fallback The fallback value to return if result is Err.
   * @return The stored value or fallback.
   */
  T value_or(const T & fallback) const
  {
    return has_value() ? std::get<T>(value_) : fallback;
  }

  /**
   * @brief Transforms the result if it is Ok, otherwise propagates the error.
   *
   * @tparam Func A callable that takes a T and returns U.
   * @param f The function to apply to the value.
   * @return Result<U, E> with transformed value or propagated error.
   */
  template<typename Func>
  auto map(Func f) const -> Result<decltype(f(std::declval<T>())), E>
  {
    if (has_value()) {
      return Result<decltype(f(std::declval<T>())), E>::Ok(f(std::get<T>(value_)));
    } else {
      return Result<decltype(f(std::declval<T>())), E>::Err(std::get<E>(value_));
    }
  }

  /**
   * @brief Chains operations that return another Result if the current one is Ok.
   *
   * @tparam Func A callable that takes a T and returns Result<U, E>.
   * @param f The function to apply to the value.
   * @return Result<U, E> from the function or the current error.
   */
  template<typename Func>
  auto and_then(Func f) const -> decltype(f(std::declval<T>()))
  {
    if (has_value()) {
      return f(std::get<T>(value_));
    } else {
      return decltype(f(std::declval<T>()))::Err(std::get<E>(value_));
    }
  }

private:
  /// @brief Constructs an Ok result.
  explicit Result(T value)
  : value_(std::move(value)) {}

  /// @brief Constructs an Err result.
  explicit Result(E error)
  : value_(std::move(error)) {}

  ValueType value_;
};

}  // namespace easynav

#endif  // EASYNAV_COMMON__RESULT_HPP_
