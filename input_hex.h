/** @file
 * @brief Parse HEX files
 */
#ifndef INPUT_HEX_H__
#define INPUT_HEX_H__

#include <string>
#include <vector>
#include <cstdint>


/// Parse a HEX file into a binary string
std::vector<uint8_t> parse_hex_file(const std::string& filename);


#endif
