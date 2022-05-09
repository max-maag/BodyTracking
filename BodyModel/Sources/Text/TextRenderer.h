#pragma once

#include <string>
#include <Kore/Math/Matrix.h>

#include "Font.h"

namespace text {
	class TextRenderer {
	public:
		void init();
		void begin(AsciiFont font);
		void drawText(std::string text, Kore::mat4 transformLocal);
		void drawTextBillboard(std::string text, Kore::vec3 position, unsigned int size);
	};
}