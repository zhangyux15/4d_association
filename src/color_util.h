#pragma once
#include <opencv2/core.hpp>
#include <map>
#include <string>
#include <vector>
#include <unordered_map>
#include <Eigen/Core>


namespace ColorUtil
{
	inline const cv::Scalar& GetColor(const std::string& name)
	{
		// refer to https://www.rapidtables.com/web/color/RGB_Color.html
		static const std::map<std::string, cv::Scalar> colorDict = {
			{ "alice_blue", { 255, 248, 240 } },
			{ "antique_white", { 215, 235, 250 } },
			{ "aqua", { 255, 255, 0 } },
			{ "aqua_marine", { 212, 255, 127 } },
			{ "azure", { 255, 255, 240 } },
			{ "beige", { 220, 245, 245 } },
			{ "bisque", { 196, 228, 255 } },
			{ "black", { 0, 0, 0 } },
			{ "blanched_almond", { 205, 235, 255 } },
			{ "blue", { 255, 0, 0 } },
			{ "blue_violet", { 226, 43, 138 } },
			{ "brown", { 42, 42, 165 } },
			{ "burly_wood", { 135, 184, 222 } },
			{ "cadet_blue", { 160, 158, 95 } },
			{ "chart_reuse", { 0, 255, 127 } },
			{ "chocolate", { 30, 105, 210 } },
			{ "coral", { 80, 127, 255 } },
			{ "corn_flower_blue", { 237, 149, 100 } },
			{ "corn_silk", { 220, 248, 255 } },
			{ "crimson", { 60, 20, 220 } },
			{ "cyan", { 255, 255, 0 } },
			{ "dark_blue", { 139, 0, 0 } },
			{ "dark_cyan", { 139, 139, 0 } },
			{ "dark_golden_rod", { 11, 134, 184 } },
			{ "dark_gray", { 169, 169, 169 } },
			{ "dark_green", { 0, 100, 0 } },
			{ "dark_khaki", { 107, 183, 189 } },
			{ "dark_magenta", { 139, 0, 139 } },
			{ "dark_olive_green", { 47, 107, 85 } },
			{ "dark_orange", { 0, 140, 255 } },
			{ "dark_orchid", { 204, 50, 153 } },
			{ "dark_red", { 0, 0, 139 } },
			{ "dark_salmon", { 122, 150, 233 } },
			{ "dark_sea_green", { 143, 188, 143 } },
			{ "dark_slate_blue", { 139, 61, 72 } },
			{ "dark_slate_gray", { 79, 79, 47 } },
			{ "dark_turquoise", { 209, 206, 0 } },
			{ "dark_violet", { 211, 0, 148 } },
			{ "deep_pink", { 147, 20, 255 } },
			{ "deep_sky_blue", { 255, 191, 0 } },
			{ "dim_gray", { 105, 105, 105 } },
			{ "dodger_blue", { 255, 144, 30 } },
			{ "firebrick", { 34, 34, 178 } },
			{ "floral_white", { 240, 250, 255 } },
			{ "forest_green", { 34, 139, 34 } },
			{ "gainsboro", { 220, 220, 220 } },
			{ "ghost_white", { 255, 248, 248 } },
			{ "gold", { 0, 215, 255 } },
			{ "golden_rod", { 32, 165, 218 } },
			{ "gray", { 128, 128, 128 } },
			{ "green", { 0, 128, 0 } },
			{ "green_yellow", { 47, 255, 173 } },
			{ "honeydew", { 240, 255, 240 } },
			{ "hot_pink", { 180, 105, 255 } },
			{ "indian_red", { 92, 92, 205 } },
			{ "indigo", { 130, 0, 75 } },
			{ "ivory", { 240, 255, 255 } },
			{ "khaki", { 140, 230, 240 } },
			{ "lavender", { 250, 230, 230 } },
			{ "lavender_blush", { 245, 240, 255 } },
			{ "lawn_green", { 0, 252, 124 } },
			{ "lemon_chiffon", { 205, 250, 255 } },
			{ "light_blue", { 230, 216, 173 } },
			{ "light_coral", { 128, 128, 240 } },
			{ "light_cyan", { 255, 255, 224 } },
			{ "light_golden", { 210, 250, 250 } },
			{ "light_gray", { 211, 211, 211 } },
			{ "light_green", { 144, 238, 144 } },
			{ "light_pink", { 193, 182, 255 } },
			{ "light_salmon", { 122, 160, 255 } },
			{ "light_sea_green", { 170, 178, 32 } },
			{ "light_sky_blue", { 250, 206, 135 } },
			{ "light_slate_gray", { 153, 136, 119 } },
			{ "light_steel_blue", { 222, 196, 176 } },
			{ "light_yellow", { 224, 255, 255 } },
			{ "lime", { 0, 255, 0 } },
			{ "lime_green", { 50, 205, 50 } },
			{ "linen", { 230, 240, 250 } },
			{ "magenta", { 255, 0, 255 } },
			{ "maroon", { 0, 0, 128 } },
			{ "medium_aqua_marine", { 170, 205, 102 } },
			{ "medium_blue", { 205, 0, 0 } },
			{ "medium_orchid", { 211, 85, 186 } },
			{ "medium_purple", { 219, 112, 147 } },
			{ "medium_sea_green", { 113, 179, 60 } },
			{ "medium_slate_blue", { 238, 104, 123 } },
			{ "medium_spring_green", { 154, 250, 0 } },
			{ "medium_turquoise", { 204, 209, 72 } },
			{ "medium_violet_red", { 133, 21, 199 } },
			{ "midnight_blue", { 112, 25, 25 } },
			{ "mint_cream", { 250, 255, 245 } },
			{ "misty_rose", { 225, 228, 255 } },
			{ "moccasin", { 181, 228, 255 } },
			{ "navajo_white", { 173, 222, 255 } },
			{ "navy", { 128, 0, 0 } },
			{ "old_lace", { 230, 245, 253 } },
			{ "olive", { 0, 128, 128 } },
			{ "olive_drab", { 35, 142, 107 } },
			{ "orange", { 0, 165, 255 } },
			{ "orange_red", { 0, 69, 255 } },
			{ "orchid", { 214, 112, 218 } },
			{ "pale_golden_rod", { 170, 232, 238 } },
			{ "pale_green", { 152, 251, 152 } },
			{ "pale_turquoise", { 238, 238, 175 } },
			{ "pale_violet_red", { 147, 112, 219 } },
			{ "papaya_whip", { 213, 239, 255 } },
			{ "peach_puff", { 185, 218, 255 } },
			{ "peru", { 63, 133, 205 } },
			{ "pink", { 203, 192, 255 } },
			{ "plum", { 221, 160, 221 } },
			{ "powder_blue", { 230, 224, 176 } },
			{ "purple", { 128, 0, 128 } },
			{ "red", { 0, 0, 255 } },
			{ "rosy_brown", { 143, 143, 188 } },
			{ "royal_blue", { 225, 105, 65 } },
			{ "saddle_brown", { 19, 69, 139 } },
			{ "salmon", { 114, 128, 250 } },
			{ "sandy_brown", { 96, 164, 244 } },
			{ "sea_green", { 87, 139, 46 } },
			{ "sea_shell", { 238, 245, 255 } },
			{ "sienna", { 45, 82, 160 } },
			{ "silver", { 192, 192, 192 } },
			{ "sky_blue", { 235, 206, 135 } },
			{ "slate_blue", { 205, 90, 106 } },
			{ "slate_gray", { 144, 128, 112 } },
			{ "snow", { 250, 250, 255 } },
			{ "spring_green", { 127, 255, 0 } },
			{ "steel_blue", { 180, 130, 70 } },
			{ "tan", { 140, 180, 210 } },
			{ "teal", { 128, 128, 0 } },
			{ "thistle", { 216, 191, 216 } },
			{ "tomato", { 71, 99, 255 } },
			{ "turquoise", { 208, 224, 64 } },
			{ "violet", { 238, 130, 238 } },
			{ "wheat", { 179, 222, 245 } },
			{ "white", { 255, 255, 255 } },
			{ "white_smoke", { 245, 245, 245 } },
			{ "yellow", { 0, 255, 255 } },
			{ "yellow_green", { 50, 205, 154 } }
		};
		return colorDict.find(name)->second;
	}

	inline const cv::Scalar& GetColor(const int& idx)
	{
		static const std::vector<cv::Scalar> colorVec = {
			{ 230, 216, 173 },					// light blue
			{ 210, 250, 250 },					// light_golden
			{ 193, 182, 255 },					// light_pink
			{ 128, 128, 240 },					// light_coral
			{ 144, 238, 144 },					// light_green
			{ 255, 191, 0 },					// deep_sky_blue
			{ 113, 179, 60 },					// medium_sea_green
			{ 122, 160, 255 },					// light_salmon	
			{ 221, 160, 221 },					// plum
			{ 255, 255, 0 },					// cyan
			{ 212, 255, 127 },					// aqua_marine
			{ 250, 206, 135 },					// light_sky_blue
			{92, 92, 205},						// indian red
			{ 230, 245, 253 },					// old_lace
			{ 180, 105, 255 },					// hot_pink
			{71, 99, 255},						// tomato
			{250, 230, 230}						// lavender
		};
		return colorVec[(idx + colorVec.size()) % colorVec.size()];
	}

	inline cv::Scalar Reverse(const cv::Scalar& color)
	{
		return cv::Scalar(color(2), color(1), color(0));
	}


	inline Eigen::Vector3f Rerverse(const Eigen::Vector3f& color)
	{
		return Eigen::Vector3f(color.z(), color.y(), color.x());
	}


	inline Eigen::Vector3f Clip(const cv::Scalar& color)
	{
		return 1.f / 255.f *Eigen::Vector3f(float(color(0)), float(color(1)), float(color(2)));
	}


	inline cv::Scalar Clip(const Eigen::Vector3f& color)
	{
		return cv::Scalar(std::round(255.f * color.x()), std::round(255.f * color.y()), std::round(255.f*color.z()));
	}
}


