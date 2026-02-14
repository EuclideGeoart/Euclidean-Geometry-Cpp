#pragma once

#include <variant>
#include <boost/variant.hpp>

// --- Helper: Overload 1 (Handles Boost Variant - Linux) ---
template <typename TargetType, typename... Args>
const TargetType* safe_get_point(const boost::variant<Args...>* v) {
    return boost::get<TargetType>(v);
}

// --- Helper: Overload 2 (Handles Std Variant - Windows) ---
template <typename TargetType, typename... Args>
const TargetType* safe_get_point(const std::variant<Args...>* v) {
    return std::get_if<TargetType>(v);
}