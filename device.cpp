#include <cassert>
#include <climits>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/exceptions.hpp>
#include "device.h"
#include "log.h"


static Log::StaticLogger logger("device");
static Log::StaticLogger logger_asm("asm");


#define LOG_OPCODE(f, ...)  SPDLOG_DEBUG(logger_asm, "PC:{:05X} SP:{:04X} OP:{:04X} " f, cpu_.pc_, cpu_.sp_, opcode, ## __VA_ARGS__);

struct OpcodeDetail
{
  /// Return true for 2-word instructions
  static constexpr bool opcode_is_32b(uint16_t opcode)
  {
    // 2-word instructions are JMP, CALL, LDS, STS
    return (opcode & 0xFE0C) == 0x940C || (opcode & 0xFC0F) == 0x9000;
  }


  // NOPE
  static unsigned int op_nope(Device& device, uint16_t) {
    LOG_OPCODE("NOPE");
    device.cpu_.pc_++;
    return 1;
  }

  // BSET, SE{C,Z,N,V,S,H,T,I}
  static unsigned int op_bset(Device& device, uint16_t opcode) {
    const uint8_t s = (opcode >> 4) & 7;
    LOG_OPCODE("BSET {}", s);
    device.cpu_.sreg_.data |= (1 << s);
    device.cpu_.pc_++;
    return 1;
  }

  // BCLR, CL{C,Z,N,V,S,H,T,I}
  static unsigned int op_bclr(Device& device, uint16_t opcode) {
    const uint8_t s = (opcode >> 4) & 7;
    LOG_OPCODE("BCLR {}", s);
    device.cpu_.sreg_.data &= ~(1 << s);
    device.cpu_.pc_++;
    return 1;
  }

  // SBI
  static unsigned int op_sbi(Device& device, uint16_t opcode) {
    const uint8_t a = (opcode >> 3) & 0x1F;
    const uint8_t b = opcode & 7;
    LOG_OPCODE("SBI 0x{},{}", a, b);
    //TODO this changes the whole byte, not just one bit
    device.setIoMem(a, device.getIoMem(a) | (1 <<b));
    device.cpu_.pc_++;
    return 1;
  }

  // CBI
  static unsigned int op_cbi(Device& device, uint16_t opcode) {
    const uint8_t a = (opcode >> 3) & 0x1F;
    const uint8_t b = opcode & 7;
    LOG_OPCODE("CBI 0x{},{}", a, b);
    //TODO this changes the whole byte, not just one bit
    device.setIoMem(a, device.getIoMem(a) & ~(1 <<b));
    device.cpu_.pc_++;
    return 1;
  }

  // COM
  static unsigned int op_com(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint8_t Rd = device.regfile_[d];
    const uint8_t R = device.regfile_[d] = 0xFF - Rd;
    LOG_OPCODE("COM r{}", d);
    device.cpu_.sreg_.C = 1;
    device.cpu_.sreg_.Z = R == 0;
    device.cpu_.sreg_.N = R & 0x80;
    device.cpu_.sreg_.V = 0;
    device.cpu_.sreg_.S = device.cpu_.sreg_.N ^ device.cpu_.sreg_.V;
    device.cpu_.pc_++;
    return 1;
  }

  // NEG
  static unsigned int op_neg(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint8_t Rd = device.regfile_[d];
    const uint8_t R = device.regfile_[d] = (0x100 - Rd) & 0xFF;
    LOG_OPCODE("NEG r{}", d);
    device.cpu_.sreg_.C = R != 0;
    device.cpu_.sreg_.Z = R == 0;
    device.cpu_.sreg_.N = R & 0x80;
    device.cpu_.sreg_.V = R == 0x80;
    device.cpu_.sreg_.S = device.cpu_.sreg_.N ^ device.cpu_.sreg_.V;
    device.cpu_.sreg_.H = (R & Rd) & 0x08;
    device.cpu_.pc_++;
    return 1;
  }

  // SWAP
  static unsigned int op_swap(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint8_t Rd = device.regfile_[d];
    LOG_OPCODE("SWAP r{}", d);
    device.regfile_[d] = ((Rd & 0x0F) << 4) | ((Rd & 0xF0) >> 4);
    device.cpu_.pc_++;
    return 1;
  }

  // INC
  static unsigned int op_inc(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint8_t R = ++device.regfile_[d];
    LOG_OPCODE("INC r{}", d);
    device.cpu_.sreg_.Z = R == 0;
    device.cpu_.sreg_.N = R & 0x80;
    device.cpu_.sreg_.V = R == 0x80;
    device.cpu_.sreg_.S = device.cpu_.sreg_.N ^ device.cpu_.sreg_.V;
    device.cpu_.pc_++;
    return 1;
  }

  // ASR
  static unsigned int op_asr(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint8_t Rd = device.regfile_[d];
    const uint8_t R = device.regfile_[d] = (Rd >> 1) | (Rd & 0x80);
    LOG_OPCODE("ASR r{}", d);
    device.cpu_.sreg_.C = Rd & 1;
    device.cpu_.sreg_.Z = R == 0;
    device.cpu_.sreg_.N = R & 0x80;
    device.cpu_.sreg_.V = device.cpu_.sreg_.N ^ device.cpu_.sreg_.C;
    device.cpu_.sreg_.S = device.cpu_.sreg_.N ^ device.cpu_.sreg_.V;
    device.cpu_.pc_++;
    return 1;
  }

  // LSR
  static unsigned int op_lsr(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint8_t Rd = device.regfile_[d];
    const uint8_t R = device.regfile_[d] = (Rd >> 1);
    LOG_OPCODE("LSR r{}", d);
    device.cpu_.sreg_.C = Rd & 1;
    device.cpu_.sreg_.Z = R == 0;
    device.cpu_.sreg_.N = 0;
    device.cpu_.sreg_.V = device.cpu_.sreg_.N ^ device.cpu_.sreg_.C;
    device.cpu_.sreg_.S = device.cpu_.sreg_.N ^ device.cpu_.sreg_.V;
    device.cpu_.pc_++;
    return 1;
  }

  // ROR
  static unsigned int op_ror(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint8_t Rd = device.regfile_[d];
    const uint8_t R = device.regfile_[d] = (Rd >> 1) | (device.cpu_.sreg_.C << 7);
    LOG_OPCODE("ROR r{}", d);
    device.cpu_.sreg_.C = Rd & 1;
    device.cpu_.sreg_.Z = R == 0;
    device.cpu_.sreg_.N = R & 0x80;
    device.cpu_.sreg_.V = device.cpu_.sreg_.N ^ device.cpu_.sreg_.C;
    device.cpu_.sreg_.S = device.cpu_.sreg_.N ^ device.cpu_.sreg_.V;
    device.cpu_.pc_++;
    return 1;
  }

  // DEC
  static unsigned int op_dec(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint8_t R = --device.regfile_[d];
    LOG_OPCODE("DEC r{}", d);
    device.cpu_.sreg_.Z = R == 0;
    device.cpu_.sreg_.N = R & 0x80;
    device.cpu_.sreg_.V = R == 0x80;
    device.cpu_.sreg_.S = device.cpu_.sreg_.N ^ device.cpu_.sreg_.V;
    device.cpu_.pc_++;
    return 1;
  }

  // CP
  static unsigned int op_cp(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint8_t r = (opcode & 0xF) | ((opcode >> 5) & 0x10);
    const uint8_t Rd = device.regfile_[d];
    const uint8_t Rr = device.regfile_[r];
    const uint8_t R = Rd - Rr;
    LOG_OPCODE("CP r{},r{}", d, r);
    device.cpu_.sreg_.C = ((~Rd & Rr) | (Rr & R) | (R & ~Rd)) & 0x80;
    device.cpu_.sreg_.Z = R == 0;
    device.cpu_.sreg_.N = R & 0x80;
    device.cpu_.sreg_.V = ((Rd & ~Rr & ~R) | (~Rd & Rr & R)) & 0x80;
    device.cpu_.sreg_.S = device.cpu_.sreg_.N ^ device.cpu_.sreg_.V;
    device.cpu_.sreg_.H = ((~Rd & Rr) | (Rr & R) | (R & ~Rd)) & 0x08;
    device.cpu_.pc_++;
    return 1;
  }

  // CPC
  static unsigned int op_cpc(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint8_t r = (opcode & 0xF) | ((opcode >> 5) & 0x10);
    const uint8_t Rd = device.regfile_[d];
    const uint8_t Rr = device.regfile_[r];
    const uint8_t R = Rd - Rr - device.cpu_.sreg_.C;
    LOG_OPCODE("CPC r{},r{}", d, r);
    device.cpu_.sreg_.C = ((~Rd & Rr) | (Rr & R) | (R & ~Rd)) & 0x80;
    device.cpu_.sreg_.Z = R == 0 && device.cpu_.sreg_.Z;
    device.cpu_.sreg_.N = R & 0x80;
    device.cpu_.sreg_.V = ((Rd & ~Rr & ~R) | (~Rd & Rr & R)) & 0x80;
    device.cpu_.sreg_.S = device.cpu_.sreg_.N ^ device.cpu_.sreg_.V;
    device.cpu_.sreg_.H = ((~Rd & Rr) | (Rr & R) | (R & ~Rd)) & 0x08;
    device.cpu_.pc_++;
    return 1;
  }

  // ADD, LSL
  static unsigned int op_add_lsl(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint8_t r = (opcode & 0xF) | ((opcode >> 5) & 0x10);
    const uint8_t Rd = device.regfile_[d];
    const uint8_t Rr = device.regfile_[r];
    const uint8_t R = device.regfile_[d] = Rd + Rr;
    LOG_OPCODE("ADD r{},r{}", d, r);
    device.cpu_.sreg_.C = ((Rd & Rr) | (Rr & ~R) | (~R & Rd)) & 0x80;
    device.cpu_.sreg_.Z = R == 0;
    device.cpu_.sreg_.N = R & 0x80;
    device.cpu_.sreg_.V = ((Rd & Rr & ~R) | (~Rd & ~Rr & R)) & 0x80;
    device.cpu_.sreg_.S = device.cpu_.sreg_.N ^ device.cpu_.sreg_.V;
    device.cpu_.sreg_.H = ((Rd & Rr) | (Rr & ~R) | (~R & Rd)) & 0x08;
    device.cpu_.pc_++;
    return 1;
  }

  // ADC, ROL
  static unsigned int op_adc_rol(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint8_t r = (opcode & 0xF) | ((opcode >> 5) & 0x10);
    const uint8_t Rd = device.regfile_[d];
    const uint8_t Rr = device.regfile_[r];
    const uint8_t R = device.regfile_[d] = Rd + Rr + device.cpu_.sreg_.C;
    LOG_OPCODE("ADC r{},r{}", d, r);
    device.cpu_.sreg_.C = ((Rd & Rr) | (Rr & ~R) | (~R & Rd)) & 0x80;
    device.cpu_.sreg_.Z = R == 0;
    device.cpu_.sreg_.N = R & 0x80;
    device.cpu_.sreg_.V = ((Rd & Rr & ~R) | (~Rd & ~Rr & R)) & 0x80;
    device.cpu_.sreg_.S = device.cpu_.sreg_.N ^ device.cpu_.sreg_.V;
    device.cpu_.sreg_.H = ((Rd & Rr) | (Rr & ~R) | (~R & Rd)) & 0x08;
    device.cpu_.pc_++;
    return 1;
  }

  // SUB
  static unsigned int op_sub(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint8_t r = (opcode & 0xF) | ((opcode >> 5) & 0x10);
    const uint8_t Rd = device.regfile_[d];
    const uint8_t Rr = device.regfile_[r];
    const uint8_t R = device.regfile_[d] = Rd - Rr;
    LOG_OPCODE("SUB r{},r{}", d, r);
    device.cpu_.sreg_.C = ((~Rd & Rr) | (Rr & R) | (R & ~Rd)) & 0x80;
    device.cpu_.sreg_.Z = R == 0;
    device.cpu_.sreg_.N = R & 0x80;
    device.cpu_.sreg_.V = ((Rd & ~Rr & ~R) | (~Rd & Rr & R)) & 0x80;
    device.cpu_.sreg_.S = device.cpu_.sreg_.N ^ device.cpu_.sreg_.V;
    device.cpu_.sreg_.H = ((~Rd & Rr) | (Rr & R) | (R & ~Rd)) & 0x08;
    device.cpu_.pc_++;
    return 1;
  }

  // SBC
  static unsigned int op_sbc(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint8_t r = (opcode & 0xF) | ((opcode >> 5) & 0x10);
    const uint8_t Rd = device.regfile_[d];
    const uint8_t Rr = device.regfile_[r];
    const uint8_t R = device.regfile_[d] = Rd - Rr - device.cpu_.sreg_.C;
    LOG_OPCODE("SBC r{},r{}", d, r);
    device.cpu_.sreg_.C = ((~Rd & Rr) | (Rr & R) | (R & ~Rd)) & 0x80;
    device.cpu_.sreg_.Z = R == 0;
    device.cpu_.sreg_.N = R & 0x80;
    device.cpu_.sreg_.V = ((Rd & ~Rr & ~R) | (~Rd & Rr & R)) & 0x80;
    device.cpu_.sreg_.S = device.cpu_.sreg_.N ^ device.cpu_.sreg_.V;
    device.cpu_.sreg_.H = ((~Rd & Rr) | (Rr & R) | (R & ~Rd)) & 0x08;
    device.cpu_.pc_++;
    return 1;
  }

  // MUL
  static unsigned int op_mul(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint8_t r = (opcode & 0xF) | ((opcode >> 5) & 0x10);
    const uint8_t Rd = device.regfile_[d];
    const uint8_t Rr = device.regfile_[r];
    const uint16_t R = Rd * Rr;
    LOG_OPCODE("MUL r{},r{}", d, r);
    device.reg01_ = R;
    device.cpu_.sreg_.C = R & 0x8000;
    device.cpu_.sreg_.Z = R == 0;
    device.cpu_.pc_++;
    return 2;
  }

  // MULS
  static unsigned int op_muls(Device& device, uint16_t opcode) {
    const uint8_t d = ((opcode >> 4) & 0xF) | 0x10; // 16 <= d <= 31
    const uint8_t r = (opcode & 0xF) | 0x10; // 16 <= r <= 31
    const int16_t Rd = u8_to_s16(device.regfile_[d]);
    const int16_t Rr = u8_to_s16(device.regfile_[r]);
    const uint16_t R = Rd * Rr;
    LOG_OPCODE("MULS r{},r{}", d, r);
    device.reg01_ = R;
    device.cpu_.sreg_.C = R & 0x8000;
    device.cpu_.sreg_.Z = R == 0;
    device.cpu_.pc_++;
    return 2;
  }

  // MULSU
  static unsigned int op_mulsu(Device& device, uint16_t opcode) {
    const uint8_t d = ((opcode >> 4) & 0x7) | 0x10; // 16 <= d <= 23
    const uint8_t r = (opcode & 0x7) | 0x10; // 16 <= r <= 23
    const int16_t Rd = u8_to_s16(device.regfile_[d]);
    const uint8_t Rr = device.regfile_[r];
    const uint16_t R = Rd * Rr;
    LOG_OPCODE("MULSU r{},r{}", d, r);
    device.reg01_ = R;
    device.cpu_.sreg_.C = R & 0x8000;
    device.cpu_.sreg_.Z = R == 0;
    device.cpu_.pc_++;
    return 2;
  }

  // FMUL
  static unsigned int op_fmul(Device& device, uint16_t opcode) {
    const uint8_t d = ((opcode >> 4) & 0x7) | 0x10; // 16 <= d <= 23
    const uint8_t r = (opcode & 0x7) | 0x10; // 16 <= r <= 23
    const uint8_t Rd = device.regfile_[d];
    const uint8_t Rr = device.regfile_[r];
    uint16_t R = Rd * Rr;
    LOG_OPCODE("FMUL r{},r{}", d, r);
    device.cpu_.sreg_.C = R & 0x8000; // before left shift
    device.reg01_ = (R <<= 1);
    device.cpu_.sreg_.Z = R == 0;
    device.cpu_.pc_++;
    return 2;
  }

  // FMULS
  static unsigned int op_fmuls(Device& device, uint16_t opcode) {
    const uint8_t d = ((opcode >> 4) & 0x7) | 0x10; // 16 <= d <= 23
    const uint8_t r = (opcode & 0x7) | 0x10; // 16 <= r <= 23
    const int16_t Rd = u8_to_s16(device.regfile_[d]);
    const int16_t Rr = u8_to_s16(device.regfile_[r]);
    uint16_t R = Rd * Rr;
    LOG_OPCODE("FMULS r{},r{}", d, r);
    device.cpu_.sreg_.C = R & 0x8000; // before left shift
    device.reg01_ = (R <<= 1);
    device.cpu_.sreg_.Z = R == 0;
    device.cpu_.pc_++;
    return 2;
  }

  // FMULSU
  static unsigned int op_fmulsu(Device& device, uint16_t opcode) {
    const uint8_t d = ((opcode >> 4) & 0x7) | 0x10; // 16 <= d <= 23
    const uint8_t r = (opcode & 0x7) | 0x10; // 16 <= r <= 23
    const int16_t Rd = u8_to_s16(device.regfile_[d]);
    const uint8_t Rr = device.regfile_[r];
    uint16_t R = Rd * Rr;
    LOG_OPCODE("FMULSU r{},r{}", d, r);
    device.cpu_.sreg_.C = R & 0x8000; // before left shift
    device.reg01_ = (R <<= 1);
    device.cpu_.sreg_.Z = R == 0;
    device.cpu_.pc_++;
    return 2;
  }

  // AND, TST
  static unsigned int op_and_tst(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint8_t r = (opcode & 0xF) | ((opcode >> 5) & 0x10);
    const uint8_t Rd = device.regfile_[d];
    const uint8_t Rr = device.regfile_[r];
    const uint8_t R = device.regfile_[d] = Rd & Rr;
    LOG_OPCODE("AND r{},r{}", d, r);
    device.cpu_.sreg_.Z = R == 0;
    device.cpu_.sreg_.N = R & 0x80;
    device.cpu_.sreg_.V = 0;
    device.cpu_.sreg_.S = device.cpu_.sreg_.N ^ device.cpu_.sreg_.V;
    device.cpu_.pc_++;
    return 1;
  }

  // EOR, CLR
  static unsigned int op_eor_clr(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint8_t r = (opcode & 0xF) | ((opcode >> 5) & 0x10);
    const uint8_t Rd = device.regfile_[d];
    const uint8_t Rr = device.regfile_[r];
    const uint8_t R = device.regfile_[d] = Rd ^ Rr;
    LOG_OPCODE("EOR r{},r{}", d, r);
    device.cpu_.sreg_.Z = R == 0;
    device.cpu_.sreg_.N = R & 0x80;
    device.cpu_.sreg_.V = 0;
    device.cpu_.sreg_.S = device.cpu_.sreg_.N ^ device.cpu_.sreg_.V;
    device.cpu_.pc_++;
    return 1;
  }

  // OR
  static unsigned int op_or(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint8_t r = (opcode & 0xF) | ((opcode >> 5) & 0x10);
    const uint8_t Rd = device.regfile_[d];
    const uint8_t Rr = device.regfile_[r];
    const uint8_t R = device.regfile_[d] = Rd | Rr;
    LOG_OPCODE("OR r{},r{}", d, r);
    device.cpu_.sreg_.Z = R == 0;
    device.cpu_.sreg_.N = R & 0x80;
    device.cpu_.sreg_.V = 0;
    device.cpu_.sreg_.S = device.cpu_.sreg_.N ^ device.cpu_.sreg_.V;
    device.cpu_.pc_++;
    return 1;
  }

  // MOV
  static unsigned int op_mov(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint8_t r = (opcode & 0xF) | ((opcode >> 5) & 0x10);
    LOG_OPCODE("MOV r{},r{}", d, r);
    device.regfile_[d] = device.regfile_[r];
    device.cpu_.pc_++;
    return 1;
  }

  // CPI
  static unsigned int op_cpi(Device& device, uint16_t opcode) {
    const uint8_t d = ((opcode >> 4) & 0xF) | 0x10; // 16 <= d <= 31
    const uint8_t K = (opcode & 0xF) | ((opcode >> 4) & 0xF0);
    const uint8_t Rd = device.regfile_[d];
    const uint8_t R = Rd - K;
    LOG_OPCODE("CPI r{},0x{:02X}", d, K);
    device.cpu_.sreg_.C = ((~Rd & K) | (K & R) | (R & ~Rd)) & 0x80;
    device.cpu_.sreg_.Z = R == 0;
    device.cpu_.sreg_.N = R & 0x80;
    device.cpu_.sreg_.V = ((Rd & ~K & ~R) | (~Rd & K & R)) & 0x80;
    device.cpu_.sreg_.S = device.cpu_.sreg_.N ^ device.cpu_.sreg_.V;
    device.cpu_.sreg_.H = ((~Rd & K) | (K & R) | (R & ~Rd)) & 0x08;
    device.cpu_.pc_++;
    return 1;
  }

  // SUBI
  static unsigned int op_subi(Device& device, uint16_t opcode) {
    const uint8_t d = ((opcode >> 4) & 0xF) | 0x10; // 16 <= d <= 31
    const uint8_t K = (opcode & 0xF) | ((opcode >> 4) & 0xF0);
    const uint8_t Rd = device.regfile_[d];
    const uint8_t R = device.regfile_[d] = Rd - K;
    LOG_OPCODE("SUBI r{},0x{:02X}", d, K);
    device.cpu_.sreg_.C = ((~Rd & K) | (K & R) | (R & ~Rd)) & 0x80;
    device.cpu_.sreg_.Z = R == 0;
    device.cpu_.sreg_.N = R & 0x80;
    device.cpu_.sreg_.V = ((Rd & ~K & ~R) | (~Rd & K & R)) & 0x80;
    device.cpu_.sreg_.S = device.cpu_.sreg_.N ^ device.cpu_.sreg_.V;
    device.cpu_.sreg_.H = ((~Rd & K) | (K & R) | (R & ~Rd)) & 0x08;
    device.cpu_.pc_++;
    return 1;
  }

  // SBCI
  static unsigned int op_sbci(Device& device, uint16_t opcode) {
    const uint8_t d = ((opcode >> 4) & 0xF) | 0x10; // 16 <= d <= 31
    const uint8_t K = (opcode & 0xF) | ((opcode >> 4) & 0xF0);
    const uint8_t Rd = device.regfile_[d];
    const uint8_t R = device.regfile_[d] = Rd - K - device.cpu_.sreg_.C;
    LOG_OPCODE("SBCI r{},0x{:02X}", d, K);
    device.cpu_.sreg_.C = ((~Rd & K) | (K & R) | (R & ~Rd)) & 0x80;
    device.cpu_.sreg_.Z = R == 0;
    device.cpu_.sreg_.N = R & 0x80;
    device.cpu_.sreg_.V = ((Rd & ~K & ~R) | (~Rd & K & R)) & 0x80;
    device.cpu_.sreg_.S = device.cpu_.sreg_.N ^ device.cpu_.sreg_.V;
    device.cpu_.sreg_.H = ((~Rd & K) | (K & R) | (R & ~Rd)) & 0x08;
    device.cpu_.pc_++;
    return 1;
  }

  // ANDI, CBR
  static unsigned int op_andi_cbr(Device& device, uint16_t opcode) {
    const uint8_t d = ((opcode >> 4) & 0xF) | 0x10; // 16 <= d <= 31
    const uint8_t K = (opcode & 0xF) | ((opcode >> 4) & 0xF0);
    const uint8_t Rd = device.regfile_[d];
    const uint8_t R = device.regfile_[d] = Rd & K;
    LOG_OPCODE("ANDI r{},0x{:02X}", d, K);
    device.cpu_.sreg_.Z = R == 0;
    device.cpu_.sreg_.N = R & 0x80;
    device.cpu_.sreg_.V = 0;
    device.cpu_.sreg_.S = device.cpu_.sreg_.N ^ device.cpu_.sreg_.V;
    device.cpu_.pc_++;
    return 1;
  }

  // ORI, SBR
  static unsigned int op_ori_sbr(Device& device, uint16_t opcode) {
    const uint8_t d = ((opcode >> 4) & 0xF) | 0x10; // 16 <= d <= 31
    const uint8_t K = (opcode & 0xF) | ((opcode >> 4) & 0xF0);
    const uint8_t Rd = device.regfile_[d];
    const uint8_t R = device.regfile_[d] = Rd | K;
    LOG_OPCODE("ORI r{},0x{:02X}", d, K);
    device.cpu_.sreg_.Z = R == 0;
    device.cpu_.sreg_.N = R & 0x80;
    device.cpu_.sreg_.V = 0;
    device.cpu_.sreg_.S = device.cpu_.sreg_.N ^ device.cpu_.sreg_.V;
    device.cpu_.pc_++;
    return 1;
  }

  // MOVW
  static unsigned int op_movw(Device& device, uint16_t opcode) {
    const uint8_t d = ((opcode >> 3) & 0x1E);
    const uint8_t r = (opcode & 0xF) << 1;
    LOG_OPCODE("MOVW r{}:r{},r{}:r{}", d, d+1, r, r+1);
    register_set<16>(&device.regfile_[d], register_get<16>(&device.regfile_[r]));
    device.cpu_.pc_++;
    return 1;
  }

  // ADIW
  static unsigned int op_adiw(Device& device, uint16_t opcode) {
    const uint8_t d = ((opcode >> 3) & 0x6) + 24;
    const uint8_t K = (opcode & 0xF) | ((opcode >> 2) & 0x30);
    const uint16_t Rd = register_get<16>(&device.regfile_[d]);
    const uint16_t R = Rd + K;
    LOG_OPCODE("ADIW r{}:r{},0x{:02X}", d, d+1, K);
    register_set<16>(&device.regfile_[d], R);
    device.cpu_.sreg_.C = (~R & Rd) & 0x8000;
    device.cpu_.sreg_.Z = R == 0;
    device.cpu_.sreg_.N = R & 0x8000;
    device.cpu_.sreg_.V = (R & ~Rd) & 0x8000;
    device.cpu_.sreg_.S = device.cpu_.sreg_.N ^ device.cpu_.sreg_.V;
    device.cpu_.pc_++;
    return 1;
  }

  // SBIW
  static unsigned int op_sbiw(Device& device, uint16_t opcode) {
    const uint8_t d = ((opcode >> 3) & 0x6) + 24;
    const uint8_t K = (opcode & 0xF) | ((opcode >> 2) & 0x30);
    const uint16_t Rd = register_get<16>(&device.regfile_[d]);
    const uint16_t R = Rd - K;
    LOG_OPCODE("SBIW r{},0x{:02X}", d, K);
    register_set<16>(&device.regfile_[d], R);
    device.cpu_.sreg_.C = (R & ~Rd) & 0x8000;
    device.cpu_.sreg_.Z = R == 0;
    device.cpu_.sreg_.N = R & 0x8000;
    device.cpu_.sreg_.V = (R & ~Rd) & 0x8000;
    device.cpu_.sreg_.S = device.cpu_.sreg_.N ^ device.cpu_.sreg_.V;
    device.cpu_.pc_++;
    return 1;
  }

  // BLD
  static unsigned int op_bld(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint8_t b = opcode & 7;
    LOG_OPCODE("BLD r{},{}", d, b);
    device.regfile_[d] = (device.regfile_[d] & ~(1 << b)) | (device.cpu_.sreg_.T << b);
    device.cpu_.pc_++;
    return 1;
  }

  // BST
  static unsigned int op_bst(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint8_t b = opcode & 7;
    LOG_OPCODE("BST r{},{}", d, b);
    device.cpu_.sreg_.T = device.regfile_[d] & (1 << b);
    device.cpu_.pc_++;
    return 1;
  }

  // LDI, SER
  static unsigned int op_ldi_ser(Device& device, uint16_t opcode) {
    const uint8_t d = ((opcode >> 4) & 0xF) | 0x10; // 16 <= d <= 31
    const uint8_t K = (opcode & 0xF) | ((opcode >> 4) & 0xF0);
    LOG_OPCODE("LDI r{},0x{:02X}", d, K);
    device.regfile_[d] = K;
    device.cpu_.pc_++;
    return 1;
  }

  // LDS (16-bit)
  static unsigned int op_lds(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint16_t k = device.flash_data_[device.cpu_.pc_+1];
    LOG_OPCODE("LDS r{},0x{:04X}", d, k);
    const memptr_t addr = k | (device.cpu_.rampd_ << 16);
    device.regfile_[d] = device.getDataMem(addr);
    device.cpu_.pc_ += 2;
    // assume the same for internal and external SRAM
    return addr >= Device::MEM_SRAM_START ? 3 : 2;
  }

  // LD (X)
  static unsigned int op_ld(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const memptr_t addr = device.regx_ | (device.cpu_.rampx_ << 16);
    device.regfile_[d] = device.getDataMem(addr);
    LOG_OPCODE("LD r{},X  @{:05X} = {:02X}", d, addr, device.regfile_[d]);
    device.cpu_.pc_++;
    // assume the same for internal and external SRAM
    return addr >= Device::MEM_SRAM_START ? 2 : 1;
  }

  // LD (X+)
  static unsigned int op_ld_x_inc(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    if(d == 26 || d == 27) {
      logger_asm->critical("undefined opcode behavior: LD r{},X+", d);
    }
    const memptr_t addr = device.regx_ | (device.cpu_.rampx_ << 16);
    device.regfile_[d] = device.getDataMem(addr);
    LOG_OPCODE("LD r{},X+  @{:05X} = {:02X}", d, addr, device.regfile_[d]);
    ++device.regx_;
    if(device.regx_ == 0) { // X overflow, update RAMPX
      device.cpu_.rampx_++;
      device.cpu_.rampx_ &= device.cpu_.ramp_mask_;
    }
    device.cpu_.pc_++;
    // assume the same for internal and external SRAM
    return addr >= Device::MEM_SRAM_START ? 2 : 1;
  }

  // LD (-X)
  static unsigned int op_ld_dec_x(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    if(d == 26 || d == 27) {
      logger_asm->critical("undefined opcode behavior: LD r{},-X", d);
    }
    --device.regx_;
    if(device.regx_ == 0xFFFF) { // X underflow, update RAMPX
      device.cpu_.rampx_--;
      device.cpu_.rampx_ &= device.cpu_.ramp_mask_;
    }
    const memptr_t addr = device.regx_ | (device.cpu_.rampx_ << 16);
    device.regfile_[d] = device.getDataMem(addr);
    LOG_OPCODE("LD r{},-X  @{:05X} = {:02X}", d, addr, device.regfile_[d]);
    device.cpu_.pc_++;
    // assume the same for internal and external SRAM
    return addr >= Device::MEM_SRAM_START ? 3 : 2;
  }

  // LD (Y i), LDD (Y iv)
  static unsigned int op_ld_y(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint8_t q = (opcode & 0x7) | ((opcode >> 7) & 0x18) | ((opcode >> 8) & 0x20);
    const memptr_t addr = (device.regy_ | (device.cpu_.rampy_ << 16)) + q;
    device.regfile_[d] = device.getDataMem(addr);
    LOG_OPCODE("LDD r{},Y+{}  @{:05X} = {:02X}", d, q, addr, device.regfile_[d]);
    device.cpu_.pc_++;
    // assume the same for internal and external SRAM
    return addr >= (q == 0 ? 2 : 1) + (Device::MEM_SRAM_START ? 1 : 0);
  }

  // LD (Y ii)
  static unsigned int op_ld_y_inc(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    if(d == 28 || d == 29) {
      logger_asm->critical("undefined opcode behavior: LD r{},Y+", d);
    }
    const memptr_t addr = device.regy_ | (device.cpu_.rampy_ << 16);
    device.regfile_[d] = device.getDataMem(addr);
    LOG_OPCODE("LD r{},Y+  @{:05X} = {:02X}", d, addr, device.regfile_[d]);
    ++device.regy_;
    if(device.regy_ == 0) { // Y overflow, update RAMPY
      device.cpu_.rampy_++;
      device.cpu_.rampy_ &= device.cpu_.ramp_mask_;
    }
    device.cpu_.pc_++;
    // assume the same for internal and external SRAM
    return addr >= Device::MEM_SRAM_START ? 2 : 1;
  }

  // LD (Y iii)
  static unsigned int op_ld_dec_y(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    if(d == 28 || d == 29) {
      logger_asm->critical("undefined opcode behavior: LD r{},-Y", d);
    }
    --device.regy_;
    if(device.regy_ == 0xFFFF) { // Y underflow, update RAMPY
      device.cpu_.rampy_--;
      device.cpu_.rampy_ &= device.cpu_.ramp_mask_;
    }
    const memptr_t addr = device.regy_ | (device.cpu_.rampy_ << 16);
    device.regfile_[d] = device.getDataMem(addr);
    LOG_OPCODE("LD r{},-Y  @{:05X} = {:02X}", d, addr, device.regfile_[d]);
    device.cpu_.pc_++;
    // assume the same for internal and external SRAM
    return addr >= Device::MEM_SRAM_START ? 3 : 2;
  }

  // LD (Z i), LDD (Z iv)
  static unsigned int op_ld_z(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint8_t q = (opcode & 0x7) | ((opcode >> 7) & 0x18) | ((opcode >> 8) & 0x20);
    const memptr_t addr = (device.regz_ | (device.cpu_.rampz_ << 16)) + q;
    device.regfile_[d] = device.getDataMem(addr);
    LOG_OPCODE("LDD r{},Z+{}  @{:05X} = {:02X}", d, q, addr, device.regfile_[d]);
    device.cpu_.pc_++;
    // assume the same for internal and external SRAM
    return addr >= (q == 0 ? 2 : 1) + (Device::MEM_SRAM_START ? 1 : 0);
  }

  // LD (Z ii)
  static unsigned int op_ld_z_inc(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    if(d == 30 || d == 31) {
      logger_asm->critical("undefined opcode behavior: LD r{},Z+", d);
    }
    const memptr_t addr = device.regz_ | (device.cpu_.rampz_ << 16);
    device.regfile_[d] = device.getDataMem(addr);
    LOG_OPCODE("LD r{},Z+  @{:05X} = {:02X}", d, addr, device.regfile_[d]);
    ++device.regz_;
    if(device.regz_ == 0) { // Z overflow, update RAMPZ
      device.cpu_.rampz_++;
      device.cpu_.rampz_ &= device.cpu_.ramp_mask_;
    }
    device.cpu_.pc_++;
    // assume the same for internal and external SRAM
    return addr >= Device::MEM_SRAM_START ? 2 : 1;
  }

  // LD (Z iii)
  static unsigned int op_ld_dec_z(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    if(d == 30 || d == 31) {
      logger_asm->critical("undefined opcode behavior: LD r{},-Z", d);
    }
    --device.regz_;
    if(device.regz_ == 0xFFFF) { // Z underflow, update RAMPZ
      device.cpu_.rampz_--;
      device.cpu_.rampz_ &= device.cpu_.ramp_mask_;
    }
    const memptr_t addr = device.regz_ | (device.cpu_.rampz_ << 16);
    device.regfile_[d] = device.getDataMem(addr);
    LOG_OPCODE("LD r{},-Z  @{:05X} = {:02X}", d, addr, device.regfile_[d]);
    device.cpu_.pc_++;
    // assume the same for internal and external SRAM
    return addr >= Device::MEM_SRAM_START ? 3 : 2;
  }

  // STS (16-bit)
  static unsigned int op_sts(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint16_t k = device.flash_data_[device.cpu_.pc_+1];
    device.setDataMem(k | (device.cpu_.rampd_ << 16), device.regfile_[d]);
    LOG_OPCODE("STS 0x{:04X},r{}", k, d);
    device.cpu_.pc_ += 2;
    return 2;
  }

  // ST (X i)
  static unsigned int op_st_x(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const memptr_t addr = device.regx_ | (device.cpu_.rampx_ << 16);
    device.setDataMem(addr, device.regfile_[d]);
    LOG_OPCODE("ST X,r{}  @{:05X} = {:02X}", d, addr, device.regfile_[d]);
    device.cpu_.pc_++;
    return 1;
  }

  // ST (X ii)
  static unsigned int op_st_x_inc(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    if(d == 26 || d == 27) {
      logger_asm->critical("undefined opcode behavior: ST X+,r{}", d);
    }
    const memptr_t addr = device.regx_ | (device.cpu_.rampx_ << 16);
    device.setDataMem(addr, device.regfile_[d]);
    LOG_OPCODE("ST X+,r{}  @{:05X} = {:02X}", d, addr, device.regfile_[d]);
    ++device.regx_;
    if(device.regx_ == 0) { // X overflow, update RAMPX
      device.cpu_.rampx_++;
      device.cpu_.rampx_ &= device.cpu_.ramp_mask_;
    }
    device.cpu_.pc_++;
    return 1;
  }

  // ST (X iii)
  static unsigned int op_st_dec_x(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    if(d == 26 || d == 27) {
      logger_asm->critical("undefined opcode behavior: ST -X,r{}", d);
    }
    --device.regx_;
    if(device.regx_ == 0xFFFF) { // X underflow, update RAMPX
      device.cpu_.rampx_--;
      device.cpu_.rampx_ &= device.cpu_.ramp_mask_;
    }
    const memptr_t addr = device.regx_ | (device.cpu_.rampx_ << 16);
    device.setDataMem(addr, device.regfile_[d]);
    LOG_OPCODE("ST -X,r{}  @{:05X} = {:02X}", d, addr, device.regfile_[d]);
    device.cpu_.pc_++;
    return 2;
  }

  // ST (Y i), STD (Y iv)
  static unsigned int op_st_y(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint8_t q = (opcode & 0x7) | ((opcode >> 7) & 0x18) | ((opcode >> 8) & 0x20);
    const memptr_t addr = (device.regy_ | (device.cpu_.rampy_ << 16)) + q;
    device.setDataMem(addr, device.regfile_[d]);
    LOG_OPCODE("STD Y+{},r{}  @{:05X} = {:02X}", q, d, addr, device.regfile_[d]);
    device.cpu_.pc_++;
    return q == 0 ? 1 : 2;
  }

  // ST (Y ii)
  static unsigned int op_st_y_inc(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    if(d == 28 || d == 29) {
      logger_asm->critical("undefined opcode behavior: ST Y+,r{}", d);
    }
    const memptr_t addr = device.regy_ | (device.cpu_.rampy_ << 16);
    device.setDataMem(addr, device.regfile_[d]);
    LOG_OPCODE("ST Y+,r{}  @{:05X} = {:02X}", d, addr, device.regfile_[d]);
    ++device.regy_;
    if(device.regy_ == 0) { // Y overflow, update RAMPY
      device.cpu_.rampy_++;
      device.cpu_.rampy_ &= device.cpu_.ramp_mask_;
    }
    device.cpu_.pc_++;
    return 1;
  }

  // ST (Y iii)
  static unsigned int op_st_dec_y(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    if(d == 28 || d == 29) {
      logger_asm->critical("undefined opcode behavior: ST -Y,r{}", d);
    }
    --device.regy_;
    if(device.regy_ == 0xFFFF) { // Y underflow, update RAMPY
      device.cpu_.rampy_--;
      device.cpu_.rampy_ &= device.cpu_.ramp_mask_;
    }
    const memptr_t addr = device.regy_ | (device.cpu_.rampy_ << 16);
    device.setDataMem(addr, device.regfile_[d]);
    LOG_OPCODE("ST -Y,r{}  @{:05X} = {:02X}", d, addr, device.regfile_[d]);
    device.cpu_.pc_++;
    return 2;
  }

  // ST (Z i), STD (Z iv)
  static unsigned int op_st_z(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint8_t q = (opcode & 0x7) | ((opcode >> 7) & 0x18) | ((opcode >> 8) & 0x20);
    const memptr_t addr = (device.regz_ | (device.cpu_.rampz_ << 16)) + q;
    device.setDataMem(addr, device.regfile_[d]);
    LOG_OPCODE("STD Z+{},r{}  @{:05X} = {:02X}", q, d, addr, device.regfile_[d]);
    device.cpu_.pc_++;
    return q == 0 ? 1 : 2;
  }

  // ST (Z ii)
  static unsigned int op_st_z_inc(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    if(d == 30 || d == 31) {
      logger_asm->critical("undefined opcode behavior: ST Z+,r{}", d);
    }
    const memptr_t addr = device.regz_ | (device.cpu_.rampz_ << 16);
    device.setDataMem(addr, device.regfile_[d]);
    LOG_OPCODE("ST Z+,r{}  @{:05X} = {:02X}", d, addr, device.regfile_[d]);
    ++device.regz_;
    if(device.regz_ == 0) { // Z overflow, update RAMPZ
      device.cpu_.rampz_++;
      device.cpu_.rampz_ &= device.cpu_.ramp_mask_;
    }
    device.cpu_.pc_++;
    return 1;
  }

  // ST (Z iii)
  static unsigned int op_st_dec_z(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    if(d == 30 || d == 31) {
      logger_asm->critical("undefined opcode behavior: ST -Z,r{}", d);
    }
    --device.regz_;
    if(device.regz_ == 0xFFFF) { // Z underflow, update RAMPZ
      device.cpu_.rampz_--;
      device.cpu_.rampz_ &= device.cpu_.ramp_mask_;
    }
    const memptr_t addr = device.regz_ | (device.cpu_.rampz_ << 16);
    device.setDataMem(addr, device.regfile_[d]);
    LOG_OPCODE("ST -Z,r{}  @{:05X} = {:02X}", d, addr, device.regfile_[d]);
    device.cpu_.pc_++;
    return 2;
  }

  // LPM (i)
  static unsigned int op_lpm(Device& device, uint16_t) {
    const uint16_t z = device.regz_;
    const uint16_t v = device.flash_data_[z>>1];
    LOG_OPCODE("LPM r0,Z");
    device.regfile_[0] = (z & 1) ? (v >> 8) & 0xFF : (v & 0xFF);
    device.cpu_.pc_++;
    return 3;
  }

  // LPM (ii), (iii)
  static unsigned int op_lpm_z(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint16_t z = device.regz_;
    const uint16_t v = device.flash_data_[z>>1];
    LOG_OPCODE("LPM r{},Z{}", d, (opcode & 1) ? "+" : "");
    device.regfile_[d] = (z & 1) ? (v >> 8) & 0xFF : (v & 0xFF);
    if(opcode & 1) {
      ++device.regz_;
    }
    device.cpu_.pc_++;
    return 3;
  }

  // ELPM (i)
  static unsigned int op_elpm(Device& device, uint16_t) {
    const uint16_t z = device.regz_;
    const uint16_t v = device.flash_data_[(z>>1) | (device.cpu_.rampz_ << 7)];
    LOG_OPCODE("ELPM r0,Z");
    device.regfile_[0] = (z & 1) ? (v >> 8) & 0xFF : (v & 0xFF);
    device.cpu_.pc_++;
    return 3;
  }

  // ELPM (ii), (iii)
  static unsigned int op_elpm_z(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint16_t z = device.regz_;
    const uint16_t v = device.flash_data_[(z>>1) | (device.cpu_.rampz_ << 7)];
    LOG_OPCODE("ELPM r{},Z{}", d, (opcode & 1) ? "+" : "");
    device.regfile_[d] = (z & 1) ? (v >> 8) & 0xFF : (v & 0xFF);
    if(opcode & 1) {
      ++device.regz_;
      if(device.regz_ == 0) { // Z overflow, update RAMPZ
        device.cpu_.rampz_++;
        device.cpu_.rampz_ &= device.cpu_.ramp_mask_;
      }
    }
    device.cpu_.pc_++;
    return 3;
  }


  // SPM, SPM#2
  static unsigned int op_spm(Device& device, uint16_t) {
    LOG_OPCODE("SPM");
    //TODO
    device.cpu_.pc_++;
    return 1;
  }

  // XCH
  static unsigned int op_xch(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint8_t Rd = device.regfile_[d];
    const uint8_t z = device.getDataMem(device.regz_);
    LOG_OPCODE("XCH r{}", d);
    device.setDataMem(device.regz_, Rd);
    device.regfile_[d] = z;
    device.cpu_.pc_++;
    return 1;
  }

  // LAC
  static unsigned int op_lac(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint8_t Rd = device.regfile_[d];
    const uint8_t z = device.getDataMem(device.regz_);
    LOG_OPCODE("LAC r{}", d);
    device.setDataMem(device.regz_, Rd & ~z);
    device.cpu_.pc_++;
    return 1;
  }

  // LAS
  static unsigned int op_las(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint8_t Rd = device.regfile_[d];
    const uint8_t z = device.getDataMem(device.regz_);
    LOG_OPCODE("LAS r{}", d);
    device.setDataMem(device.regz_, Rd | z);
    device.regfile_[d] = z;
    device.cpu_.pc_++;
    return 1;
  }

  // LAT
  static unsigned int op_lat(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint8_t Rd = device.regfile_[d];
    const uint8_t z = device.getDataMem(device.regz_);
    LOG_OPCODE("LAT r{}", d);
    device.setDataMem(device.regz_, Rd ^ z);
    device.regfile_[d] = z;
    device.cpu_.pc_++;
    return 1;
  }

  // JMP
  static unsigned int op_jmp(Device& device, uint16_t opcode) {
    const uint16_t opcode2 = device.flash_data_[device.cpu_.pc_+1];
    flashptr_t k = ((flashptr_t)(((opcode >> 3) & 0x3E) | (opcode & 1)) << 16) | opcode2;
    LOG_OPCODE("JMP 0x{:X}", k);
    device.cpu_.pc_ = k;
    return 3;
  }

  // RJMP
  static unsigned int op_rjmp(Device& device, uint16_t opcode) {
    const int16_t k = u16_to_s16<12>(opcode & 0xFFF);
    LOG_OPCODE("RJMP {}", k);
    device.cpu_.pc_ += k + 1;
    return 2;
  }

  // IJMP
  static unsigned int op_ijmp(Device& device, uint16_t) {
    LOG_OPCODE("IJMP");
    device.cpu_.pc_ = device.regz_;
    return 2;
  }

  // EIJMP
  template <bool small_flash>
  static unsigned int op_eijmp(Device& device, uint16_t) {
    LOG_OPCODE("EIJMP");
    if constexpr(small_flash) {
      logger_asm->critical("EIJMP not available: flash size is less than 128K");
      device.cpu_.pc_++;
      return 1;
    } else {
      device.cpu_.pc_ = device.regz_ | ((flashptr_t)device.cpu_.eind_ << 16);
      return 2;
    }
  }

  // BRBC, BR{CC,SH,NE,PL,VC,GE,HC,TC,ID}
  // BRBS, BR{CS,LO,EQ,MI,VS,LT,HS,TS,IE}
  static unsigned int op_brbc_brbs(Device& device, uint16_t opcode) {
    const uint8_t s = opcode & 7;
    const int8_t k = u8_to_s8<7>((opcode >> 3) & 0x7F);
    LOG_OPCODE("BRB{} {},{}", (opcode & 0x400) ? 'C' : 'S', s, (int)k);
    if( (((device.cpu_.sreg_.data >> s) ^ (opcode >> 10)) & 1) ) {
      device.cpu_.pc_ += k + 1;
      return 2;
    } else {
      device.cpu_.pc_++;
      return 1;
    }
  }

  // SBRC, SBRS
  static unsigned int op_sbrc_sbrs(Device& device, uint16_t opcode) {
    const uint8_t r = (opcode >> 4) & 0x1F;
    const uint8_t b = opcode & 7;
    LOG_OPCODE("SBR{} r{},{}", (opcode & 0x200) ? 'S' : 'C', r, b);
    if( !(((device.regfile_[r] >> b) ^ (opcode >> 9)) & 1) ) {
      const uint16_t opcode2 = device.flash_data_[device.cpu_.pc_+1];
      if(opcode_is_32b(opcode2)) {
        device.cpu_.pc_ += 3;
        return 3;
      } else {
        device.cpu_.pc_ += 2;
        return 2;
      }
    } else {
      device.cpu_.pc_++;
      return 1;
    }
  }

  // SBIC, SBIS
  static unsigned int op_sbic_sbis(Device& device, uint16_t opcode) {
    const uint8_t a = (opcode >> 3) & 0x1F;
    const uint8_t b = opcode & 7;
    LOG_OPCODE("SBI{} 0x{:X},{}", (opcode & 0x200) ? 'S' : 'C', a, b);
    if( !(((device.getIoMem(a) >> b) ^ (opcode >> 9)) & 1) ) {
      const uint16_t opcode2 = device.flash_data_[device.cpu_.pc_+1];
      if(opcode_is_32b(opcode2)) {
        device.cpu_.pc_ += 3;
        return 3;
      } else {
        device.cpu_.pc_ += 2;
        return 2;
      }
    } else {
      device.cpu_.pc_++;
      return 1;
    }
  }

  // CPSE
  static unsigned int op_cpse(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint8_t r = (opcode & 0xF) | ((opcode >> 5) & 0x10);
    const uint8_t Rd = device.regfile_[d];
    const uint8_t Rr = device.regfile_[r];
    LOG_OPCODE("CPSE r{},r{}", d, r);
    if( Rd == Rr ) {
      const uint16_t opcode2 = device.flash_data_[device.cpu_.pc_+1];
      if(opcode_is_32b(opcode2)) {
        device.cpu_.pc_ += 3;
        return 3;
      } else {
        device.cpu_.pc_ += 2;
        return 2;
      }
    } else {
      device.cpu_.pc_++;
      return 1;
    }
  }

  // CALL
  template <bool small_flash>
  static unsigned int op_call(Device& device, uint16_t opcode) {
    const uint16_t opcode2 = device.flash_data_[device.cpu_.pc_+1];
    const flashptr_t k = ((flashptr_t)(((opcode >> 3) & 0x3E) | (opcode & 1)) << 16) | opcode2;
    LOG_OPCODE("CALL 0x{:X}", k);
    //TODO detect stack overflows
    unsigned int cycles;
    if constexpr(small_flash) {
      stack_set<16>(device.stack(), device.cpu_.pc_+2);
      device.cpu_.sp_ -= 2;
      cycles = 3;
    } else {
      stack_set<24>(device.stack(), device.cpu_.pc_+2);
      device.cpu_.sp_ -= 3;
      cycles = 4;
    }
    device.cpu_.pc_ = k;
    return cycles;
  }

  // RCALL
  template <bool small_flash>
  static unsigned int op_rcall(Device& device, uint16_t opcode) {
    const int16_t k = u16_to_s16<12>(opcode & 0xFFF);
    LOG_OPCODE("RCALL {}", k);
    //TODO detect stack overflows
    unsigned int cycles;
    if constexpr(small_flash) {
      stack_set<16>(device.stack(), device.cpu_.pc_+1);
      device.cpu_.sp_ -= 2;
      cycles = 2;
    } else {
      stack_set<24>(device.stack(), device.cpu_.pc_+1);
      device.cpu_.sp_ -= 3;
      cycles = 3;
    }
    device.cpu_.pc_ += k + 1;
    return cycles;
  }

  // ICALL
  template <bool small_flash>
  static unsigned int op_icall(Device& device, uint16_t) {
    LOG_OPCODE("ICALL");
    //TODO detect stack overflows
    unsigned int cycles;
    if constexpr(small_flash) {
      stack_set<16>(device.stack(), device.cpu_.pc_+1);
      device.cpu_.sp_ -= 2;
      cycles = 2;
    } else {
      stack_set<24>(device.stack(), device.cpu_.pc_+1);
      device.cpu_.sp_ -= 3;
      cycles = 3;
    }
    device.cpu_.pc_ = device.regz_;
    return cycles;
  }

  // EICALL
  template <bool small_flash>
  static unsigned int op_eicall(Device& device, uint16_t) {
    LOG_OPCODE("EICALL");
    //TODO detect stack overflows
    unsigned int cycles;
    if constexpr(small_flash) {
      logger_asm->critical("EICALL not available: flash size is less than 128K");
      device.cpu_.pc_++;
      cycles = 1;
    } else {
      stack_set<24>(device.stack(), device.cpu_.pc_+1);
      device.cpu_.sp_ -= 3;
      device.cpu_.pc_ = device.regz_ | ((flashptr_t)device.cpu_.eind_ << 16);
      cycles = 3;
    }
    return cycles;
  }

  // RET
  template <bool small_flash>
  static unsigned int op_ret(Device& device, uint16_t) {
    LOG_OPCODE("RET");
    unsigned int cycles;
    if constexpr(small_flash) {
      device.cpu_.sp_ += 2;
      device.cpu_.pc_ = stack_get<16>(device.stack());
      cycles = 2;
    } else {
      device.cpu_.sp_ += 3;
      device.cpu_.pc_ = stack_get<24>(device.stack());
      cycles = 3;
    }
    return cycles;
  }

  // RETI
  template <bool small_flash>
  static unsigned int op_reti(Device& device, uint16_t) {
    LOG_OPCODE("RETI");
    // Clear the "interrupt executing" flag of highest level.
    // The ASM specs wrongly states that the SREG's I bit is set
    // which is not true on XMEGA devices.
    if(device.pmic_.status_.nmiex) {
      device.pmic_.status_.nmiex = 0;
    } else if(device.pmic_.status_.hilvlex) {
      device.pmic_.status_.hilvlex = 0;
    } else if(device.pmic_.status_.medlvlex) {
      device.pmic_.status_.medlvlex = 0;
    } else if(device.pmic_.status_.lolvlex) {
      device.pmic_.status_.lolvlex = 0;
    } else {
      logger_asm->critical("RETI but no active interruption");
    }

    unsigned int cycles;
    if constexpr(small_flash) {
      device.cpu_.sp_ += 2;
      device.cpu_.pc_ = stack_get<16>(device.stack());
      cycles = 2;
    } else {
      device.cpu_.sp_ += 3;
      device.cpu_.pc_ = stack_get<24>(device.stack());
      cycles = 3;
    }
    return cycles;
  }

  // POP
  static unsigned int op_pop(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    LOG_OPCODE("POP r{}", d);
    device.cpu_.sp_++;
    device.regfile_[d] = *device.stack();
    device.cpu_.pc_++;
    return 1;
  }

  // PUSH
  static unsigned int op_push(Device& device, uint16_t opcode) {
    const uint8_t r = (opcode >> 4) & 0x1F;
    LOG_OPCODE("PUSH r{}", r);
    *device.stack() = device.regfile_[r];
    device.cpu_.sp_--;
    device.cpu_.pc_++;
    return 1;
  }

  // IN
  static unsigned int op_in(Device& device, uint16_t opcode) {
    const uint8_t d = (opcode >> 4) & 0x1F;
    const uint8_t a = (opcode & 0xF) | ((opcode >> 5) & 0x30);
    LOG_OPCODE("IN r{},0x{:X}", d, a);
    device.regfile_[d] = device.getIoMem(a);
    device.cpu_.pc_++;
    return 1;
  }

  // OUT
  static unsigned int op_out(Device& device, uint16_t opcode) {
    const uint8_t r = (opcode >> 4) & 0x1F;
    const uint8_t a = (opcode & 0xF) | ((opcode >> 5) & 0x30);
    LOG_OPCODE("OUT 0x{:X},r{}", a, r);
    device.setIoMem(a, device.regfile_[r]);
    device.cpu_.pc_++;
    return 1;
  }

  // WDR
  static unsigned int op_wdr(Device& device, uint16_t) {
    LOG_OPCODE("WDR");
    //TODO
    device.cpu_.pc_++;
    return 1;
  }

  // SLEEP
  static unsigned int op_sleep(Device& device, uint16_t) {
    LOG_OPCODE("SLEEP");
    //TODO
    device.cpu_.pc_++;
    return 1;
  }

  // BREAK
  static unsigned int op_break(Device& device, uint16_t) {
    LOG_OPCODE("BREAK");
    device.breaked_ = true;
    device.cpu_.pc_++;
    return 1;
  }

  // DES
  static unsigned int op_des(Device& device, uint16_t) {
    LOG_OPCODE("DES");
    //TODO
    device.cpu_.pc_++;
    return 1;
  }

  // invalid opcode
  static unsigned int op_invalid(Device& device, uint16_t opcode) {
    logger_asm->critical("PC:{:05X} SP:{:04X} OP:{:04X} unknown opcode", device.cpu_.pc_, device.cpu_.sp_, opcode);
    device.cpu_.pc_++;
    return 1;
  }


  static Device::OpcodeFunc getFunc(const Device& device, uint16_t opcode)
  {
    const bool small_flash = device.flash_size_ <= 0x20000;

    if(opcode == 0) { return &op_nope; }
    else if((opcode & 0xFF8F) == 0x9408) { return &op_bset; }
    else if((opcode & 0xFF8F) == 0x9488) { return &op_bclr; }
    else if((opcode & 0xFF00) == 0x9A00) { return &op_sbi; }
    else if((opcode & 0xFF00) == 0x9800) { return &op_cbi; }
    else if((opcode & 0xFE0F) == 0x9400) { return &op_com; }
    else if((opcode & 0xFE0F) == 0x9401) { return &op_neg; }
    else if((opcode & 0xFE0F) == 0x9402) { return &op_swap; }
    else if((opcode & 0xFE0F) == 0x9403) { return &op_inc; }
    else if((opcode & 0xFE0F) == 0x9405) { return &op_asr; }
    else if((opcode & 0xFE0F) == 0x9406) { return &op_lsr; }
    else if((opcode & 0xFE0F) == 0x9407) { return &op_ror; }
    else if((opcode & 0xFE0F) == 0x940A) { return &op_dec; }
    else if((opcode & 0xFC00) == 0x1400) { return &op_cp; }
    else if((opcode & 0xFC00) == 0x0400) { return &op_cpc; }
    else if((opcode & 0xFC00) == 0x0C00) { return &op_add_lsl; }
    else if((opcode & 0xFC00) == 0x1C00) { return &op_adc_rol; }
    else if((opcode & 0xFC00) == 0x1800) { return &op_sub; }
    else if((opcode & 0xFC00) == 0x0800) { return &op_sbc; }
    else if((opcode & 0xFC00) == 0x9C00) { return &op_mul; }
    else if((opcode & 0xFF00) == 0x0200) { return &op_muls; }
    else if((opcode & 0xFF88) == 0x0300) { return &op_mulsu; }
    else if((opcode & 0xFF88) == 0x0308) { return &op_fmul; }
    else if((opcode & 0xFF88) == 0x0380) { return &op_fmuls; }
    else if((opcode & 0xFF88) == 0x0380) { return &op_fmulsu; }
    else if((opcode & 0xFC00) == 0x2000) { return &op_and_tst; }
    else if((opcode & 0xFC00) == 0x2400) { return &op_eor_clr; }
    else if((opcode & 0xFC00) == 0x2800) { return &op_or; }
    else if((opcode & 0xFC00) == 0x2C00) { return &op_mov; }
    else if((opcode & 0xF000) == 0x3000) { return &op_cpi; }
    else if((opcode & 0xF000) == 0x5000) { return &op_subi; }
    else if((opcode & 0xF000) == 0x4000) { return &op_sbci; }
    else if((opcode & 0xF000) == 0x7000) { return &op_andi_cbr; }
    else if((opcode & 0xF000) == 0x6000) { return &op_ori_sbr; }
    else if((opcode & 0xFF00) == 0x0100) { return &op_movw; }
    else if((opcode & 0xFF00) == 0x9600) { return &op_adiw; }
    else if((opcode & 0xFF00) == 0x9700) { return &op_sbiw; }
    else if((opcode & 0xFE08) == 0xF800) { return &op_bld; }
    else if((opcode & 0xFE08) == 0xFA00) { return &op_bst; }
    else if((opcode & 0xF000) == 0xE000) { return &op_ldi_ser; }
    else if((opcode & 0xFE0F) == 0x9000) { return &op_lds; }
    else if((opcode & 0xFE0F) == 0x900C) { return &op_ld; }
    else if((opcode & 0xFE0F) == 0x900D) { return &op_ld_x_inc; }
    else if((opcode & 0xFE0F) == 0x900E) { return &op_ld_dec_x; }
    else if((opcode & 0xD208) == 0x8008) { return &op_ld_y; }
    else if((opcode & 0xFE0F) == 0x9009) { return &op_ld_y_inc; }
    else if((opcode & 0xFE0F) == 0x900A) { return &op_ld_dec_y; }
    else if((opcode & 0xD208) == 0x8000) { return &op_ld_z; }
    else if((opcode & 0xFE0F) == 0x9001) { return &op_ld_z_inc; }
    else if((opcode & 0xFE0F) == 0x9002) { return &op_ld_dec_z; }
    else if((opcode & 0xFE0F) == 0x9200) { return &op_sts; }
    else if((opcode & 0xFE0F) == 0x920C) { return &op_st_x; }
    else if((opcode & 0xFE0F) == 0x920D) { return &op_st_x_inc; }
    else if((opcode & 0xFE0F) == 0x920E) { return &op_st_dec_x; }
    else if((opcode & 0xD208) == 0x8208) { return &op_st_y; }
    else if((opcode & 0xFE0F) == 0x9209) { return &op_st_y_inc; }
    else if((opcode & 0xFE0F) == 0x920A) { return &op_st_dec_y; }
    else if((opcode & 0xD208) == 0x8200) { return &op_st_z; }
    else if((opcode & 0xFE0F) == 0x9201) { return &op_st_z_inc; }
    else if((opcode & 0xFE0F) == 0x9202) { return &op_st_dec_z; }
    else if(opcode == 0x95C8) { return &op_lpm; }
    else if((opcode & 0xFE0E) == 0x9004) { return &op_lpm_z; }
    else if(opcode == 0x95D8) { return &op_elpm; }
    else if((opcode & 0xFE0E) == 0x9006) { return &op_elpm_z; }
    else if(opcode == 0x95E8 || opcode == 0x95F8) { return &op_spm; }
    else if((opcode & 0xFE0F) == 0x9204) { return &op_xch; }
    else if((opcode & 0xFE0F) == 0x9206) { return &op_lac; }
    else if((opcode & 0xFE0F) == 0x9205) { return &op_las; }
    else if((opcode & 0xFE0F) == 0x9207) { return &op_lat; }
    else if((opcode & 0xFE0E) == 0x940C) { return &op_jmp; }
    else if((opcode & 0xF000) == 0xC000) { return &op_rjmp; }
    else if(opcode == 0x9409) { return &op_ijmp; }
    else if(opcode == 0x9419) { return small_flash ? &op_eijmp<true> : &op_eijmp<false>; }
    else if((opcode & 0xF800) == 0xF000) { return &op_brbc_brbs; }
    else if((opcode & 0xFC00) == 0xFC00) { return &op_sbrc_sbrs; }
    else if((opcode & 0xFD00) == 0x9900) { return &op_sbic_sbis; }
    else if((opcode & 0xFC00) == 0x1000) { return &op_cpse; }
    else if((opcode & 0xFE0E) == 0x940E) { return small_flash ? &op_call<true> : &op_call<false>; }
    else if((opcode & 0xF000) == 0xD000) { return small_flash ? &op_rcall<true> : &op_rcall<false>; }
    else if(opcode == 0x9509) { return small_flash ? &op_icall<true> : &op_icall<false>; }
    else if(opcode == 0x9519) { return small_flash ? &op_eicall<true> : &op_eicall<false>; }
    else if(opcode == 0x9508) { return small_flash ? &op_ret<true> : &op_ret<false>; }
    else if(opcode == 0x9518) { return small_flash ? &op_reti<true> : &op_reti<false>; }
    else if((opcode & 0xFE0F) == 0x900F) { return &op_pop; }
    else if((opcode & 0xFE0F) == 0x920F) { return &op_push; }
    else if((opcode & 0xF800) == 0xB000) { return &op_in; }
    else if((opcode & 0xF800) == 0xB800) { return &op_out; }
    else if(opcode == 0x95A8) { return &op_wdr; }
    else if(opcode == 0x9588) { return &op_sleep; }
    else if(opcode == 0x9598) { return &op_break; }
    else if((opcode & 0xFF0F) == 0x940B) { return &op_des; }
    else { return &op_invalid; }
  }
};

#undef LOG_OPCODE


Device::Device(const ModelConf& model, ConfTree& conf):
    // memory map
    flash_size_(model.flash_size),
    flash_page_size_(model.flash_page_size),
    flash_app_size_(flash_size_ - model.flash_boot_size),
    flash_app_table_start_(flash_app_size_ - model.flash_boot_size),
    flash_app_table_size_(model.flash_boot_size),
    flash_boot_start_(flash_size_ - model.flash_boot_size),
    flash_boot_size_(model.flash_boot_size),
    mem_eeprom_size_(model.mem_eeprom_size),
    mem_sram_size_(model.mem_sram_size),
    mem_exsram_start_(MEM_SRAM_START + mem_sram_size_),
    mem_exsram_size_(model.has_exsram ? MEM_MAX_SIZE - mem_exsram_start_ : 0),
    // conf
    conf_(conf),
    // flash and memory data
    flash_data_(flash_size_/2, 0xFFFF),
    sram_data_(mem_sram_size_),
    // blocks
    cpu_(*this),
    osc_(*this),
    clk_(*this, osc_),
    pmic_(*this),
    gpior_(*this)
{
  // check memory map values
  // order and which values are checked are important to detect possible
  // overflows and to ensure everything is checked
#define DEVICE_CHECK(cond,msg) \
  if(!(cond)) throw DeviceConfigurationError(*this, msg);

  DEVICE_CHECK(flash_page_size_ % 2 == 0, "flash page size not aligned on words");
  DEVICE_CHECK(flash_size_ % flash_page_size_ == 0, "flash size not aligned on page size");
  DEVICE_CHECK(flash_boot_size_ % flash_page_size_ == 0, "flash bootloader size not aligned on page size");
  DEVICE_CHECK(flash_boot_size_ < flash_size_, "flash bootloader larger than total flash");
  DEVICE_CHECK(flash_boot_size_ < flash_app_size_, "flash bootloader larger than flash application");

  DEVICE_CHECK(mem_eeprom_size_ <= 0x1000, "memory mapped EEPROM is too large");
  DEVICE_CHECK(mem_sram_size_ < MEM_MAX_SIZE - MEM_SRAM_START, "internal SRAM is too large");

#undef DEVICE_CHECK

  // connect blocks
  connectBlock(&cpu_);
  connectBlock(&clk_);
  connectBlock(&osc_);
  connectBlock(&pmic_);
  connectBlock(&gpior_);

  // prepare opcodes
  for(unsigned i = 0; i < opcode_map_.size(); ++i) {
    opcode_map_[i] = OpcodeDetail::getFunc(*this, i);
  }
}

Device::~Device()
{
}

const Device::ConfTree& Device::conf(const std::string& path)
{
  try {
    return conf_.get_child(path);
  } catch(const boost::property_tree::ptree_bad_path&) {
    return conf_.put_child(path, ConfTree());
  }
}


void Device::reset()
{
  // reset state
  instruction_cycles_ = 0;
  interrupt_wait_instruction_ = true;
  clk_sys_tick_ = 0;
  clk_sys_events_.clear();

  // reset CLK first for schedule()
  clk_.reset();
  // CPU step MUST be executed first
  assert(clk_sys_events_.empty());
  schedule(ClockType::CPU, [&]() { return stepCPU(); });

  // reset blocks
  for(auto block: blocks_) {
    block->reset();
  }

  // clear memory
  // SRAM is not cleared on reset
  regfile_.fill(0);
}


void Device::step()
{
  // When an event is unscheduled, it is removed from clk_sys_events_ and a
  // "large" part of the vector can be copied. However, after some time, events
  // likely to be unscheduled will be "pushed" to the end of the vector and
  // such, faster to be removed.
  ++clk_sys_tick_;
  // don't use iterator since callbacks can schedule new events
  for(size_t i = 0; i < clk_sys_events_.size(); ) {
    if(!clk_sys_events_[i]) {
      // event unscheduled using unschedule()
      clk_sys_events_.erase(clk_sys_events_.begin() + i);
      continue;
    }
    auto& ev = *clk_sys_events_[i];
    if(ev.tick <= clk_sys_tick_) {
      const unsigned int next = ev.callback();
      if(next) {
        ev.tick += next * ev.scale;
      } else {
        clk_sys_events_.erase(clk_sys_events_.begin() + i);
        continue;  // don't iterate
      }
    }
    ++i;
  }
}


unsigned int Device::stepCPU()
{
  //TODO handle halted CPU

  // Step order is important to ensure an instruction is executed before any
  // pending interruption is served.
  breaked_ = false;

  // check for pending interruptions
  if(instruction_cycles_ == 0 && !interrupt_wait_instruction_ && cpu_.sreg_.I && !ccpState()) {
    if(processPendingInterrupts()) {
      instruction_cycles_ = 5;
      interrupt_wait_instruction_ = true;
    }
  }

  //TODO:check Before clock handling blocks were stepped here

  // execute instruction
  while(instruction_cycles_ == 0) {
    instruction_cycles_ = executeNextInstruction();
    interrupt_wait_instruction_ = false;
  }
  instruction_cycles_--;
  return 1;
}


void Device::setIvLvl(ivnum_t iv, IntLvl lvl)
{
  switch(lvl) {
    case INTLVL_NONE:
      if(iv_pending_.lo.erase(iv)) break;
      if(iv_pending_.med.erase(iv)) break;
      if(iv_pending_.hi.erase(iv)) break;
      if(iv_pending_.nmi.erase(iv)) break;
      break;
    case INTLVL_LO:
      if(!iv_pending_.lo.insert(iv).second) break;
      if(iv_pending_.med.erase(iv)) break;
      if(iv_pending_.hi.erase(iv)) break;
      if(iv_pending_.nmi.erase(iv)) break;
      break;
    case INTLVL_MED:
      if(!iv_pending_.med.insert(iv).second) break;
      if(iv_pending_.lo.erase(iv)) break;
      if(iv_pending_.hi.erase(iv)) break;
      if(iv_pending_.nmi.erase(iv)) break;
      break;
    case INTLVL_HI:
      if(!iv_pending_.hi.insert(iv).second) break;
      if(iv_pending_.lo.erase(iv)) break;
      if(iv_pending_.med.erase(iv)) break;
      if(iv_pending_.nmi.erase(iv)) break;
      break;
    case INTLVL_NMI:
      if(!iv_pending_.nmi.insert(iv).second) break;
      if(iv_pending_.lo.erase(iv)) break;
      if(iv_pending_.med.erase(iv)) break;
      if(iv_pending_.hi.erase(iv)) break;
      break;
    default:
      logger->critical("invalid INTLVL: {}", lvl);
  }
}


IntLvl Device::currentIntLvl() const
{
  uint8_t intlvlex = pmic_.status_.data;
  if(intlvlex == 0) {
    return INTLVL_NONE; // common case
  }
  IntLvl levels[] = {INTLVL_NMI, INTLVL_HI, INTLVL_MED, INTLVL_LO};
  for(IntLvl lvl: levels) {
    if(intlvlex & (1 << (lvl-1))) {
      return lvl;
    }
  }
  return INTLVL_NONE; // should not happen
}


void Device::loadFlash(const std::vector<uint8_t>& data)
{
  if(data.size() > 2*flash_data_.size()) {
    throw DeviceConfigurationError(*this, "flash data to load is too large");
  }
  if(data.size() % 2 != 0) {
    throw DeviceConfigurationError(*this, "flash data not aligned on words");
  }
  auto itsrc = data.begin();
  auto itdst = flash_data_.begin();
  while(itsrc != data.end()) {
    *itdst++ = *itsrc + (*(itsrc+1) << 8);
    itsrc +=2;
  }
}


bool Device::processPendingInterrupts()
{
  const IntLvl intlvl = currentIntLvl();
  IntLvl intlvl_new = INTLVL_NONE;
  InterruptQueue* iv_queue = nullptr;
  if(intlvl >= INTLVL_NMI) {
    return false;
  } else if(!iv_pending_.nmi.empty()) {
    intlvl_new = INTLVL_NMI;
    iv_queue = &iv_pending_.nmi;
  } else if(intlvl >= INTLVL_HI) {
    return false;
  } else if(pmic_.ctrl_.hilvlen && !iv_pending_.hi.empty()) {
    intlvl_new = INTLVL_HI;
    iv_queue = &iv_pending_.hi;
  } else if(intlvl >= INTLVL_MED) {
    return false;
  } else if(pmic_.ctrl_.medlvlen && !iv_pending_.med.empty()) {
    intlvl_new = INTLVL_MED;
    iv_queue = &iv_pending_.med;
  } else if(intlvl >= INTLVL_LO) {
    return false;
  } else if(pmic_.ctrl_.lolvlen && !iv_pending_.lo.empty()) {
    intlvl_new = INTLVL_LO;
    iv_queue = &iv_pending_.lo;
  } else {
    return false;
  }

  const ivnum_t iv_num = *iv_queue->begin();
  iv_queue->erase(iv_queue->begin());
  // update PMIC status
  pmic_.status_.data |= 1 << (intlvl_new - 1);
  // get IV address (each IV is 2-word long), don't forget IVSEL
  flashptr_t iv_addr = 2*iv_num;
  if(pmic_.ctrl_.ivsel) {
    iv_addr += flash_boot_start_;
  }
  // execute the IV
  Block* const block = iv_blocks_[iv_num];
  assert(block);
  block->executeIv(iv_num-block->iv_base());
  if(flash_size_ <= 0x20000) {
    stack_set<16>(stack(), cpu_.pc_);
    cpu_.sp_ -= 2;
  } else {
    stack_set<24>(stack(), cpu_.pc_);
    cpu_.sp_ -= 3;
  }
  cpu_.pc_ = iv_addr;
  logger->debug("acknowledge interrupt {}, level {}, PC:{:05X}", iv_num, intlvl_new, iv_addr);
  return true;
}


unsigned int Device::executeNextInstruction()
{
  const uint16_t opcode = flash_data_[cpu_.pc_];
  //TODO check for cpu_.pc_ oveflow? (check other accesses to flash_data_ below)
  //TODO check availability of some opcodes on specific devices
  //TODO check use of RAMP[XYZD]

  // Instructions which longs longer than 1 cycle are executed immediately but
  // will delay the execution of the next one.
  return opcode_map_[opcode](*this, opcode);
}


void Device::connectBlock(Block* block)
{
  logger->info("connecting block {}", block->name());

  if(block->io_addr() + block->io_size() > MEM_IO_SIZE) {
    throw BlockError(*block, "I/O memory out of range");
  }
  for(auto addr = block->io_addr(); addr < block->io_addr() + block->io_size(); ++addr) {
    if(io_blocks_[addr]) {
      throw BlockError(*block, "I/O memory space overlaps with block " + io_blocks_[addr]->name());
    }
    io_blocks_[addr] = block;
  }

  if(block->iv_base()) {
    if(block->iv_base() + block->iv_count() > IV_MAX_COUNT) {
      throw BlockError(*block, "IVs out of range");
    }
    for(auto iv = block->iv_base(); iv < block->iv_base() + block->iv_count(); ++iv) {
      if(iv_blocks_[iv]) {
        throw BlockError(*block, "interrupt vectors overlap with block " + iv_blocks_[iv]->name());
      }
      iv_blocks_[iv] = block;
    }
  }

  blocks_.push_back(block);
}



void Device::setSP(uint16_t sp)
{
  if(sp >= MEM_SRAM_START + MEM_SRAM_START) {
    //XXX exsram should be allowed
    logger->critical("invalid SP value (overflow): 0x{:02X}", sp);
  } else {
    cpu_.sp_ = sp;
  }
}

void Device::setPC(flashptr_t pc)
{
  if(pc >= flash_size_) {
    logger->critical("invalid PC value (overflow): 0x{:X}", pc);
    throw std::runtime_error("PC overflow");
  } else {
    cpu_.pc_ = pc;
  }
}


uint8_t Device::getDataMem(memptr_t addr) const
{
  if(addr < MEM_IO_SIZE) {
    return Device::getIoMem(addr);
  } else if(addr >= MEM_EEPROM_START && addr < MEM_EEPROM_START+mem_eeprom_size_) {
    logger->warn("EEPROM read access not supported at 0x{:x}", addr);
    return 0; //TODO
  } else if(addr >= MEM_SRAM_START && addr < MEM_SRAM_START+mem_sram_size_) {
    return sram_data_[addr-MEM_SRAM_START];
  } else if(addr >= MEM_EMULATOR_START && addr < MEM_EMULATOR_START+MEM_EMULATOR_SIZE) {
    return Device::getEmulatorMem(addr);
  } else if(mem_exsram_size_ && addr >= mem_exsram_start_ && addr < mem_exsram_start_+mem_exsram_size_) {
    logger->warn("external SRAM read access not supported: 0x{:X}", addr);
    return 0; //TODO
  } else {
    logger->error("invalid data memory address to read: 0x{:X}", addr);
    return 0;
  }
}

void Device::setDataMem(memptr_t addr, uint8_t v)
{
  if(addr < MEM_IO_SIZE) {
    Device::setIoMem(addr, v);
  } else if(addr >= MEM_EEPROM_START && addr < MEM_EEPROM_START+mem_eeprom_size_) {
    logger->warn("EEPROM write access not supported at 0x{:x}", addr);
  } else if(addr >= MEM_SRAM_START && addr < MEM_SRAM_START+mem_sram_size_) {
    sram_data_[addr-MEM_SRAM_START] = v;
  } else if(addr >= MEM_EMULATOR_START && addr < MEM_EMULATOR_START+MEM_EMULATOR_SIZE) {
    Device::setEmulatorMem(addr, v);
  } else if(mem_exsram_size_ && addr >= mem_exsram_start_ && addr < mem_exsram_start_+mem_exsram_size_) {
    logger->warn("external SRAM write access not supported at 0x{:x}", addr);
  } else {
    logger->error("invalid data memory address to write: 0x{:X}", addr);
  }
}

uint8_t Device::getIoMem(ioptr_t addr) const
{
  Block* const block = io_blocks_[addr];
  if(block == NULL) {
    logger->error("invalid I/O address to read: 0x{:X} (no block)", addr);
    return 0;
  }
  return block->getIo(addr - block->io_addr());
}

void Device::setIoMem(ioptr_t addr, uint8_t v)
{
  Block* const block = io_blocks_[addr];
  if(block == NULL) {
    logger->error("invalid I/O address to write: 0x{:X} (no block)", addr);
    return;
  }
  block->setIo(addr - block->io_addr(), v);
}

uint8_t Device::getEmulatorMem(memptr_t addr) const
{
  uint16_t offset = addr - MEM_EMULATOR_START;
  switch(offset) {
    case 0x00: // clk_sys_tick_
    case 0x01:
    case 0x02:
    case 0x03:
      return (clk_sys_tick_ >> ((offset - 0x00) * 8)) & 0xff;
    default:
      logger->warn("emulator memory ready 0x{:06X}: reserved address", addr);
      return 0;
  }
}

void Device::setEmulatorMem(memptr_t addr, uint8_t)
{
  logger->error("emulator memory write 0x{:06X}: not writable", addr);
}


const ClockEvent* Device::schedule(ClockType clock, ClockCallback cb, unsigned int ticks)
{
  assert(cb);
  unsigned int scale = getClockScale(clock);
  clk_sys_events_.push_back(std::make_unique<ClockEvent>(clock, cb, (clk_sys_tick_/scale+ticks) * scale, scale));
  return clk_sys_events_.rbegin()->get();
}

void Device::unschedule(const ClockEvent* ev)
{
  // clear the element, null ptr will be reset on next step() loop
  //TODO check if events can be unscheduled by CPU on their due tick
  for(auto it=clk_sys_events_.begin(); it!=clk_sys_events_.end(); ++it) {
    if(it->get() == ev) {
      it->reset();
      return;
    }
  }
  logger->error("cannot unschedule event: not found");
}


void Device::onClockConfigChange()
{
  for(auto& ev: clk_sys_events_) {
    unsigned int scale = getClockScale(ev->clock);
    if(ev->scale == scale) {
      continue; // nothing to do
    }
    // this method should only be called on slowest clock tick
    assert((ev->tick - clk_sys_tick_) % ev->scale == 0);
    unsigned int dt = (ev->tick - clk_sys_tick_ + ev->scale-1) / ev->scale;
    ev->tick = clk_sys_tick_ + dt * scale;
    ev->scale = scale;
  }
}


unsigned int Device::getClockScale(ClockType clock) const
{
  switch(clock) {
    case ClockType::SYS:
      return 1;
    case ClockType::CPU:
    case ClockType::PER:
      return clk_.prescaler_a_ * clk_.prescaler_b_ * clk_.prescaler_c_;
    case ClockType::PER2:
      return clk_.prescaler_a_ * clk_.prescaler_b_;
    case ClockType::PER4:
      return clk_.prescaler_a_;
    case ClockType::ASY:
      logger->warn("ASY clock not supported");
      return 1;
    default:
      throw std::runtime_error("invalid clock type");
  }
}

