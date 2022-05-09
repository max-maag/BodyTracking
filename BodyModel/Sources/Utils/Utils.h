#pragma once

#include <type_traits>

namespace utils {
	void runtimeAssert(bool condition, const char* const message);

	// Returns next higher power of two, or 0 if datatype can't hold the result
	template <typename T>
	T nextPowerOfTwo(T value) {
		static_assert(std::is_integral_v<T>, "Type of value must be integral.");

		T powerOfTwo = static_cast<T>(1);

		while (value > 0) {
			powerOfTwo <<= 1;
			value >>= 1;
		}

		return powerOfTwo;
	}

	// Returns value, if it is a power of two, otherwise the next higher power of two, or 0 if datatype can't hold the result
	template<typename T>
	T nextOrEqualPowerOfTwo(T value) {
		if (value == 0)
			return 0;

		return nextPowerOfTwo(value - 1);
	}
}