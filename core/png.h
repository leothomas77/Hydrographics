#pragma once

#include "types.h"

struct PngImage
{
	uint16_t m_width = 0;
	uint16_t m_height = 0;

	// pixels are always assumed to be 32 bit
	uint32_t* m_data = nullptr;
};

bool PngLoad(const char* filename, PngImage& image);
void PngFree(PngImage& image);

struct HdrImage
{
	uint16_t m_width;
	uint16_t m_height;

	float* m_data;
};

bool HdrLoad(const char* filename, HdrImage& image);
void HdrFree(HdrImage& image);

