#pragma once

#include <string>
#include <array>
#include <filesystem>

#include "../Utils/Utils.h"

namespace text {
	struct AsciiCharacter {
		unsigned int x, y, width, height;
		int originX, originY;
	};


	struct AsciiFont {
		std::filesystem::path texturePath;
		unsigned int size, width, height;
		std::array<const AsciiCharacter, 95> characters;

		const AsciiCharacter& operator[](char c) const {
			utils::runtimeAssert(c >= ' ' && c <= '~', "Not a text character");
			return characters[c - ' '];
		}
	};

}