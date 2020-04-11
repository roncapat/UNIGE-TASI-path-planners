#pragma once
#include <fstream>
#include <vector>
#include <stdexcept>
#include <iostream>
#include <boost/multi_array.hpp>

#pragma pack(push, 1)
struct BMPFileHeader {
  uint16_t file_type{0x4D42};          // File type always BM which is 0x4D42 (stored as hex uint16_t in little endian)
  uint32_t file_size{0};               // Size of the file (in bytes)
  uint16_t reserved1{0};               // Reserved, always 0
  uint16_t reserved2{0};               // Reserved, always 0
  uint32_t offset_data{0};             // Start position of pixel data (bytes from the beginning of the file)
};

struct BMPInfoHeader {
  uint32_t size{0};                      // Size of this header (in bytes)
  int32_t width{0};                      // width of bitmap in pixels
  int32_t height{0};                     // width of bitmap in pixels
  //       (if positive, bottom-up, with origin in lower left corner)
  //       (if negative, top-down, with origin in upper left corner)
  uint16_t planes{1};                    // No. of planes for the target device, this is always 1
  uint16_t bit_count{0};                 // No. of bits per pixel
  uint32_t compression{0};               // 0 or 3 - uncompressed. THIS PROGRAM CONSIDERS ONLY UNCOMPRESSED BMP images
  uint32_t size_image{0};                // 0 - for uncompressed images
  int32_t x_pixels_per_meter{0};
  int32_t y_pixels_per_meter{0};
  uint32_t colors_used
      {0};               // No. color indexes in the color table. Use 0 for the max number of colors allowed by bit_count
  uint32_t colors_important{0};          // No. of colors used for displaying the bitmap. If 0 all colors are required
};

struct BMPColorHeader {
  uint32_t red_mask{0x00ff0000};         // Bit mask for the red channel
  uint32_t green_mask{0x0000ff00};       // Bit mask for the green channel
  uint32_t blue_mask{0x000000ff};        // Bit mask for the blue channel
  uint32_t alpha_mask{0xff000000};       // Bit mask for the alpha channel
  uint32_t color_space_type{0x73524742}; // Default "sRGB" (0x73524742)
  uint32_t unused[16]{0};                // Unused data for sRGB color space
};
#pragma pack(pop)

struct BMP {
  BMPFileHeader file_header;
  BMPInfoHeader bmp_info_header;
  BMPColorHeader bmp_color_header;
  typedef boost::multi_array<uint8_t, 2> array_type;
  array_type::extent_gen extents;
  array_type data;

  BMP(const char *fname) {
      read(fname);
  }

  void read(const char *fname) {
      std::ifstream input{fname, std::ios_base::binary};
      if (input) {
          input.read((char *) &file_header, sizeof(file_header));
          if (file_header.file_type != 0x4D42) {
              throw std::runtime_error("Error! Unrecognized file format.");
          }
          input.read((char *) &bmp_info_header, sizeof(bmp_info_header));

          // The BMPColorHeader is used only for transparent images
          if (bmp_info_header.bit_count == 32) {
              // Check if the file has bit mask color information
              if (bmp_info_header.size >= (sizeof(BMPInfoHeader) + sizeof(BMPColorHeader))) {
                  input.read((char *) &bmp_color_header, sizeof(bmp_color_header));
                  // Check if the pixel data is stored as BGRA and if the color space type is sRGB
                  check_color_header(bmp_color_header);
              } else {
                  std::cerr << "Error! The file \"" << fname << "\" does not seem to contain bit mask information\n";
                  throw std::runtime_error("Error! Unrecognized file format.");
              }
          }

          // BMP pixels rows are 4-bytes aligned, 0 padded at the end of each line
          int padding = (bmp_info_header.width % 4 == 0) ? 0 : 4 - bmp_info_header.width % 4;

          // Jump to the pixel data location
          input.seekg(file_header.offset_data, std::ifstream::beg);

          if (bmp_info_header.height < 0) {
              throw std::runtime_error("The program can treat only BMP images with origin in the bottom left corner!");
          }

          if (bmp_info_header.bit_count != 0x8)
              throw std::runtime_error("The program can treat only 8-bit BMP images");

          input.seekg(file_header.offset_data, std::ifstream::beg);
          data.resize(extents[bmp_info_header.height][bmp_info_header.width + padding]);
          for (int y = bmp_info_header.height - 1; y >= 0; y--)
              input.read((char *) data.data() + y * (bmp_info_header.width + padding), bmp_info_header.width + padding);
          data.resize(extents[bmp_info_header.height][bmp_info_header.width]);
          //BMP images pixels are specified from LOWER LEFT corner, we need TOP LEFT
      } else {
          throw std::runtime_error("Unable to open the input image file.");
      }
  }

 private:

  // Check if the pixel data is stored as BGRA and if the color space type is sRGB
  void check_color_header(BMPColorHeader &bmp_color_header) {
      BMPColorHeader expected_color_header;
      if (expected_color_header.red_mask != bmp_color_header.red_mask ||
          expected_color_header.blue_mask != bmp_color_header.blue_mask ||
          expected_color_header.green_mask != bmp_color_header.green_mask ||
          expected_color_header.alpha_mask != bmp_color_header.alpha_mask) {
          throw std::runtime_error(
              "Unexpected color mask format! The program expects the pixel data to be in the BGRA format");
      }
      if (expected_color_header.color_space_type != bmp_color_header.color_space_type) {
          throw std::runtime_error("Unexpected color space type! The program expects sRGB values");
      }
  }
};
