#include "Utils.h"

#include <cassert>

namespace utils {
	void runtimeAssert(bool condition, const char const* message) {
		assert(((void*)message, condition));
	}
}