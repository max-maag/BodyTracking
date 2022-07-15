#pragma once

#include <memory>
#include <string_view>
#include <Kore/Graphics2/Graphics.h>
#include <Kore/Math/Vector.h>
#include <Kore/Math/Matrix.h>

namespace utils {

	/* Renders 2D shapes over a 3D scene.
	 *
	 * Typical lifecycle:
	 *
	 *		OverlayRenderer overlayRenderer;
	 *
	 *		initKoreAndWindow();
	 *
	 *		overlayRenderer.init();
	 *
	 *		while(!stopRendering) {
	 *			mat4 projection3d = ...;
	 *			mat4 view3d = ...;
	 *
	 *			draw3dScence(projection3d, view3d);
	 *
	 *			overlayRenderer.begin(projection3d, view3d);
	 *
	 *			overlayRenderer.drawX(...);
	 *			overlayRenderer.drawY(...);
	 *
	 *			overlayRenderer.end();
	 *
	 *			swapRenderBuffers();
	 *		}
	 */
	class OverlayRenderer
	{
	private:
		// Inverse of the projection matrix used by Graphics2.
		const Kore::mat4 g2ProjectionInverse;

		const unsigned int widthScreen;
		const unsigned int heightScreen;

		std::unique_ptr<Kore::Graphics2::Graphics2> graphics2;

		Kore::mat4 matrixProjection3d;
		Kore::mat4 matrixView3d;

		// Converts a 3D coordinate to the screen coordinate system used by Graphics2.
		Kore::vec2 worldToScreen(Kore::vec4 positionWorld3d) const;

	public:

		// width and height in pixels
		OverlayRenderer(unsigned int widthScreen, unsigned int heightScreen);


		// Must be called once before any call to other methods can be made.
		void init();


		// Begin a render pass.
		void begin(Kore::mat4 matrixProjection3d, Kore::mat4 matrixView3d);


		// End a render pass.
		void end();


		/* Draw a square at the given position with the given size and color.
		 *
		 * Arguments:
		 *		position3d:	The square's center in 3D space.
		 *		size:		The square's edge length in pixels.
		 *		color:		The square's color in ARGB.
		 */
		void drawSquare(Kore::vec4 position3d, float size, Kore::uint color = Kore::Graphics2::Color::White);


		/* Draw a line between the given positions with the given width and color.
		 *
		 *		┌────────────────────────┐	─────
		 *		│                        │	  ↑
		 *		x position1    position2 x	width
		 *		│                        │    ↓
		 *		└────────────────────────┘	─────
		 *
		 *
		 * Arguments:
		 *		positionStart3d:	The line's start in 3D space.
		 *		positionEnd3d:		The line's end in 3D space.
		 *		width:				The line's size.
		 *		color:				The lines's color in ARGB.
		 */
		void drawLine(Kore::vec4 positionStart3d, Kore::vec4 positionEnd3d, float width, Kore::uint color = Kore::Graphics2::Color::White);


		// Set font for text rendering
		void setFont(Kore::Kravur* font);


		/* Draw a text.
		 * 
		 * Arguments:
		 *		text:		The text to draw.
		 *		position3d:	Start of the text in 3D space.
		 *		color:		The text's color.
		 */
		void drawText(std::string_view text, Kore::vec4 position3d, Kore::vec2 offset2D = Kore::vec2(0, 0), Kore::uint color = Kore::Graphics2::Color::White);

		/* Draw a text with screen coordinates.
		 *
		 * Arguments:
		 *		text:		The text to draw.
		 *		position2d:	Start of the text in pixel space.
		 *		color:		The text's color.
		 */
		void drawText2D(std::string_view text, Kore::vec2 position2d, Kore::uint color = Kore::Graphics2::Color::White);
	};
}