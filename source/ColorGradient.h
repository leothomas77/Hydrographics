#include <vector> 
#include <algorithm>
/**
http://www.andrewnoske.com/wiki/Code_-_heatmaps_and_color_gradients
*/
class ColorGradient
{
private:
	struct ColorPoint  // Internal class used to store colors at different points in the gradient.
	{
		float r, g, b;      // Red, green and blue values of our color.
		float val;        // Position of our color along the gradient (between 0 and 1).
		ColorPoint(float red, float green, float blue, float value)
			: r(red), g(green), b(blue), val(value) {}
	};
	std::vector<ColorPoint> color;      // An array of color points in ascending value.

public:
	//-- Default constructor:
	ColorGradient() { createDefaultHeatMapGradient(); }

	//-- Inserts a new color point into its correct position:
	void addColorPoint(float red, float green, float blue, float value)
	{
		for (int i = 0; i<color.size(); i++) {
			if (value < color[i].val) {
				color.insert(color.begin() + i, ColorPoint(red, green, blue, value));
				return;
			}
		}
		color.push_back(ColorPoint(red, green, blue, value));
	}

	//-- Inserts a new color point into its correct position:
	void clearGradient() { color.clear(); }

	//-- Places a 5 color heapmap gradient into the "color" vector:
	void createDefaultHeatMapGradient()
	{
    float rangeValues[5] = { 0.0f, 0.25f, 0.5f, 0.75f, 1.0f };
    float factor = 1.0f;

    color.clear();
		color.push_back(ColorPoint(0, 0, 1, rangeValues[0] * factor));      // Blue.
		color.push_back(ColorPoint(0, 1, 1, rangeValues[1] * factor));      // Cyan.
		color.push_back(ColorPoint(0, 1, 0, rangeValues[2] * factor));      // Green.
		color.push_back(ColorPoint(1, 1, 0, rangeValues[3] * factor));      // Yellow.
		color.push_back(ColorPoint(1, 0, 0, rangeValues[4] * factor));      // Red.
	}

	//-- Inputs a (value) between 0 and 1 and outputs the (red), (green) and (blue)
	//-- values representing that position in the gradient.
	Vec4 getColorAtValue(const float value)
	{
		float red, green, blue;
		if (color.size() == 0)
			return Vec4(0.0f);

		for (int i = 0; i<color.size(); i++)
		{
			ColorPoint &currC = color[i];
			if (value < currC.val)
			{
				ColorPoint &prevC = color[std::max(0, i - 1)];
				float valueDiff = (prevC.val - currC.val);
				float fractBetween = (valueDiff == 0) ? 0 : (value - currC.val) / valueDiff;
				red = (prevC.r - currC.r)*fractBetween + currC.r;
				green = (prevC.g - currC.g)*fractBetween + currC.g;
				blue = (prevC.b - currC.b)*fractBetween + currC.b;
				return Vec4(red, green, blue, 1.0f);
			}
		}
		red = color.back().r;
		green = color.back().g;
		blue = color.back().b;
		return Vec4(red, green, blue, 1.0f);
	}
};