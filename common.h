#ifndef COMMON_H
#define COMMON_H

#include <limits>
#include <cstdint>
#include <cstddef>


/// Type to store data memory offset
typedef uint_fast32_t memptr_t;
/// Type to store I/O memory offset
typedef uint_fast16_t ioptr_t;
/// Type to store flash memory offset
typedef uint_fast32_t flashptr_t;
/// Interrupt vector number
typedef uint_fast16_t ivnum_t;


/** @brief Interrupt levels
 *
 * Bit index in PMIC.STATUS can be obtained decrementing the value by 1.
 */
enum IntLvl {
  INTLVL_NONE = 0,
  INTLVL_LO = 1,
  INTLVL_MED = 2,
  INTLVL_HI = 3,
  INTLVL_NMI = 8,
};


/// Map bit size to an appropriated unsigned integer type
template <unsigned N> class nbits_to_utype;
template <> struct nbits_to_utype<8> { typedef uint8_t type; };
template <> struct nbits_to_utype<16> { typedef uint16_t type; };
template <> struct nbits_to_utype<24> { typedef uint32_t type; };
template <> struct nbits_to_utype<32> { typedef uint32_t type; };

/// Read a multibyte little-endian value from an 8-bit array-compatible type
template <unsigned N> typename nbits_to_utype<N>::type register_get(const uint8_t* p);
template <> inline uint8_t register_get<8>(const uint8_t* p) { return p[0]; }
template <> inline uint16_t register_get<16>(const uint8_t* p) { return p[0] + (p[1] << 8); }
template <> inline uint32_t register_get<24>(const uint8_t* p) { return p[0] + (p[1] << 8) + (p[2] << 16); }
template <> inline uint32_t register_get<32>(const uint8_t* p) { return p[0] + (p[1] << 8) + (p[2] << 16) + (p[3] << 24); }

/// Write a multibyte little-endian value to an 8-bit array-compatible type
template <unsigned N> void register_set(uint8_t* p, typename nbits_to_utype<N>::type v);
template <> inline void register_set<8>(uint8_t* p, uint8_t v) { p[0] = v; }
template <> inline void register_set<16>(uint8_t* p, uint16_t v) { p[0] = v & 0xff; p[1] = (v >> 8) & 0xff; }
template <> inline void register_set<24>(uint8_t* p, uint32_t v) { p[0] = v & 0xff; p[1] = (v >> 8) & 0xff; p[2] = (v >> 16) & 0xff; }
template <> inline void register_set<32>(uint8_t* p, uint32_t v) { p[0] = v & 0xff; p[1] = (v >> 8) & 0xff; p[2] = (v >> 16) & 0xff; p[3] = (v >> 24) & 0xff; }

/// Like register_get(), but for stack access (assume pre-increment)
template <unsigned N> typename nbits_to_utype<N>::type stack_get(const uint8_t* p);
template <> inline uint8_t stack_get<8>(const uint8_t* p) { return p[0]; }
template <> inline uint16_t stack_get<16>(const uint8_t* p) { return p[0] + (p[-1] << 8); }
template <> inline uint32_t stack_get<24>(const uint8_t* p) { return p[0] + (p[-1] << 8) + (p[-2] << 16); }
template <> inline uint32_t stack_get<32>(const uint8_t* p) { return p[0] + (p[-1] << 8) + (p[-2] << 16) + (p[-3] << 24); }

/// Like register_set(), but for stack access (assume post-decrement)
template <unsigned N> void stack_set(uint8_t* p, typename nbits_to_utype<N>::type v);
template <> inline void stack_set<8>(uint8_t* p, uint8_t v) { p[0] = v; }
template <> inline void stack_set<16>(uint8_t* p, uint16_t v) { p[0] = v & 0xff; p[-1] = (v >> 8) & 0xff; }
template <> inline void stack_set<24>(uint8_t* p, uint32_t v) { p[0] = v & 0xff; p[-1] = (v >> 8) & 0xff; p[-2] = (v >> 16) & 0xff; }
template <> inline void stack_set<32>(uint8_t* p, uint32_t v) { p[0] = v & 0xff; p[-1] = (v >> 8) & 0xff; p[-2] = (v >> 16) & 0xff; p[-3] = (v >> 24) & 0xff; }


/// Wrap memory into 8/16/32-bit register
template <unsigned N>
class Register
{
  static_assert(N % 8 == 0, "invalid bit count");
  uint8_t data_[N/8];
 public:
  typedef typename nbits_to_utype<N>::type value_type;
  Register& operator=(value_type v) { register_set<N>(data_, v); return *this; }
  operator value_type() const { return register_get<N>(data_); }
  Register& operator++() { register_set<N>(data_, register_get<N>(data_)+1); return *this; }
  Register& operator--() { register_set<N>(data_, register_get<N>(data_)-1); return *this; }
};

static_assert(sizeof(Register<32>) == 4, "Register class not compatible with your compiler");


/// Portable bitfields
template <unsigned B, unsigned N=1, typename T=uint8_t>
struct BitField
{
  static_assert(!std::numeric_limits<T>::is_signed, "T is not unsigned");
  static_assert(std::numeric_limits<T>::digits >= (int)(B+N), "T is too short");
  static constexpr T mask = (1 << N)-1;
  T data;
  BitField& operator=(T v) { data = (data & ~(mask << B)) | ((N == 1 ? !!v : (v & mask)) << B); return *this; }
  operator T() const { return (data >> B) & mask; }
};


/// Convert unsigned value to signed, the portable way
template <typename S, typename U, unsigned nbits=std::numeric_limits<U>::digits>
S unsigned_to_signed(U v) {
  static_assert(std::numeric_limits<S>::is_signed, "S is not signed");
  static_assert(!std::numeric_limits<U>::is_signed, "U is not unsigned");
  static_assert(std::numeric_limits<S>::digits >= (int)nbits-1, "S non-sign part is shorter than nbits");
  static_assert(std::numeric_limits<U>::digits >= (int)nbits-1, "U is shorter than nbits");
  return (v & (1 << (nbits-1))) ? (S)v - (1 << nbits) : v;
}

template <unsigned nbits> int16_t u8_to_s8(uint8_t v) {
  return unsigned_to_signed<int8_t, uint8_t, nbits>(v);
};
template <unsigned nbits> int16_t u16_to_s16(uint16_t v) {
  return unsigned_to_signed<int16_t, uint16_t, nbits>(v);
};
template <unsigned nbits=8> int16_t u8_to_s16(uint8_t v) {
  return unsigned_to_signed<int16_t, uint8_t, nbits>(v);
};


#endif
