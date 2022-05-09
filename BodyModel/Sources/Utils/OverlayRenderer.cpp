#include "OverlayRenderer.h"

namespace utils {
	using namespace Kore;

	OverlayRenderer::OverlayRenderer(unsigned int screenWidth, unsigned int screenHeight):
		widthScreen(screenWidth),
		heightScreen(screenHeight),
		g2ProjectionInverse(mat4::orthogonalProjection(0, screenWidth, screenHeight, 0, 0.1, 1000).Invert()) {}


	void OverlayRenderer::init() {
		graphics2 = std::make_unique<Graphics2::Graphics2>(widthScreen, heightScreen, true);
	}


	void OverlayRenderer::begin(mat4 matrixProjection3d, mat4 matrixView3d) {
		this->matrixProjection3d = matrixProjection3d;
		this->matrixView3d = matrixView3d;

		graphics2->begin(true, -1, -1, false);
	}


	void OverlayRenderer::end() {
		graphics2->end();
	}


	void OverlayRenderer::drawSquare(vec4 position3d, float size, uint color) {
		vec2 positionScreen = worldToScreen(position3d);
		
		// Graphics2 assumes position is top left corner
		positionScreen.x() -= size / 2;
		positionScreen.y() -= size / 2;

		graphics2->setColor(color);

		graphics2->fillRect(positionScreen.x(), positionScreen.y(), size, size);
	}


	void OverlayRenderer::drawLine(vec4 positionStart3d, vec4 positionEnd3d, float width, uint color) {
		vec2 positionStartScreen = worldToScreen(positionStart3d);
		vec2 positionEndScreen = worldToScreen(positionEnd3d);

		graphics2->setColor(color);

		graphics2->drawLine(positionStartScreen.x(), positionStartScreen.y(), positionEndScreen.x(), positionEndScreen.y(), width);
	}


	vec2 OverlayRenderer::worldToScreen(vec4 positionWorld3d) const {
		vec4 v = g2ProjectionInverse * matrixProjection3d * matrixView3d * vec4(positionWorld3d, 1);

		return (v.xyz() / v.w()).xy();
	}

	void OverlayRenderer::setFont(Kore::Kravur* font) {
		graphics2->setFont(font);
	}

	void OverlayRenderer::drawText(std::string_view text, Kore::vec4 position3d, Kore::vec2 offset2D, Kore::uint color) {
		graphics2->setFontColor(color);

		vec2 positionScreen = worldToScreen(position3d) + offset2D;

		graphics2->drawString(text.data(), text.size(), positionScreen.x(), positionScreen.y());
	}
}