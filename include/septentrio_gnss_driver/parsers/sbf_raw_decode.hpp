// =============================================================================
// sbf_raw_decode.hpp
//
// Private header for septentrio_gnss_driver.
// Decodes GPS/GAL/BDS raw SBF nav subframe blocks into GpsNavMsg / GalNavMsg /
// BdsNavMsg so the driver can publish ephemeris from raw blocks when decoded
// blocks (5891 / 4002 / 4081) are not present in the SBF file.
//
// Self-contained C++17.  No dependency on super_gps or external GNSS libraries.
//
// Buffer layouts (match eph_decoder.h in super_gps / RTKLIB conventions):
//   GPS  : 90 bytes – SF1 @ [0:30), SF2 @ [30:60), SF3 @ [60:90)
//             each subframe packed as 10 × 24 data bits (6 parity bits stripped)
//   GAL I/NAV : 112 bytes – word-type N @ [N*16 : N*16+16) for N = 0..6
//   GAL F/NAV : 186 bytes – page-type N @ [(N-1)*31 : N*31) for N = 1..6
//   BDS D1    : 114 bytes – SF1 @ [0:38), SF2 @ [38:76), SF3 @ [76:114)
//               (38 bytes = 304 bits, covers the 300-bit BDS subframe)
//
// Angular parameters in GpsNavMsg / GalNavMsg / BdsNavMsg are in semi-circles.
// All scale factors therefore omit the SC2RAD (= π) factor used in eph_t.
// =============================================================================
#pragma once

#include <array>
#include <cstdint>
#include <cstring>

// ROS 2 message types (driver-internal)
#include <septentrio_gnss_driver/msg/gps_nav.hpp>
#include <septentrio_gnss_driver/msg/gal_nav.hpp>
#include <septentrio_gnss_driver/msg/bds_nav.hpp>

using GpsNavMsg = septentrio_gnss_driver::msg::GpsNav;
using GalNavMsg = septentrio_gnss_driver::msg::GalNav;
using BdsNavMsg = septentrio_gnss_driver::msg::BdsNav;

namespace sbf_raw {

// ---------------------------------------------------------------------------
// Bit-extraction helpers  (MSB-first byte-array convention, same as RTKLIB)
// ---------------------------------------------------------------------------
inline uint32_t getbitu(const uint8_t* buf, int pos, int len)
{
    uint32_t bits = 0;
    for (int i = pos; i < pos + len; i++)
        bits = (bits << 1) | ((buf[i / 8] >> (7 - i % 8)) & 1u);
    return bits;
}
inline int32_t getbits(const uint8_t* buf, int pos, int len)
{
    uint32_t bits = getbitu(buf, pos, len);
    if (len > 0 && 32 > len && (bits & (1u << (len - 1))))
        bits |= ~((1u << len) - 1);
    return static_cast<int32_t>(bits);
}
inline uint32_t getbitu2(const uint8_t* buf, int p1, int l1, int p2, int l2)
{
    return (getbitu(buf, p1, l1) << l2) | getbitu(buf, p2, l2);
}
inline int32_t getbits2(const uint8_t* buf, int p1, int l1, int p2, int l2)
{
    if (getbitu(buf, p1, 1))
        return static_cast<int32_t>((getbits(buf, p1, l1) << l2) | getbitu(buf, p2, l2));
    return static_cast<int32_t>(getbitu2(buf, p1, l1, p2, l2));
}
inline uint32_t merge_two_u(uint32_t a, uint32_t b, int n)
{
    return (a << n) | b;
}

// Write 'len' bits of 'data' (MSB-first) starting at bit 'pos' in 'buf'.
inline void setbitu(uint8_t* buf, int pos, int len, uint32_t data)
{
    uint32_t mask = 1u << (len - 1);
    for (int i = pos; i < pos + len; i++, mask >>= 1)
    {
        if (data & mask) buf[i / 8] |=  (1u << (7 - i % 8));
        else             buf[i / 8] &= ~(1u << (7 - i % 8));
    }
}

// ---------------------------------------------------------------------------
// Power-of-2 scale factors
// ---------------------------------------------------------------------------
static constexpr double P2_5  = 1.0 / (1 << 5);
static constexpr double P2_6  = 1.0 / (1 << 6);
static constexpr double P2_19 = 1.0 / (1 << 19);
static constexpr double P2_29 = 1.0 / (1LL << 29);
static constexpr double P2_31 = 1.0 / (1LL << 31);
static constexpr double P2_32 = 1.0 / (1LL << 32);
static constexpr double P2_33 = 1.0 / (1LL << 33);
static constexpr double P2_34 = 1.0 / (1LL << 34);
static constexpr double P2_43 = 1.0 / (8796093022208LL);   // 2^43
static constexpr double P2_46 = 1.0 / (70368744177664LL);  // 2^46
static constexpr double P2_50 = 1.0 / (1125899906842624LL); // 2^50
static constexpr double P2_55 = 3.552713678800501e-17;     // 2^-55
static constexpr double P2_59 = 1.734723475976807e-18;     // 2^-59
static constexpr double P2_66 = 1.355252715606881e-20;     // 2^-66

// ============================================================================
//  GPS C/A  (SBF block 4017 — GPSRawCA)
// ============================================================================

// Convert 10 × u32 SBF NAVBits (little-endian) to a 30-byte packed buffer.
// SBF GPS word: bits 6-29 = 24 data bits, bit 29 = D1 (first received = MSB).
// Output: buf[30], bit 0 = D1 of word 1, bit 239 = D24 of word 10.
inline void gps_navbits_to_buf(const uint8_t* navbits_le, uint8_t buf[30])
{
    std::memset(buf, 0, 30);
    for (int i = 0; i < 10; i++)
    {
        // Read 32-bit word little-endian
        uint32_t w = static_cast<uint32_t>(navbits_le[i * 4]) |
                     (static_cast<uint32_t>(navbits_le[i * 4 + 1]) << 8) |
                     (static_cast<uint32_t>(navbits_le[i * 4 + 2]) << 16) |
                     (static_cast<uint32_t>(navbits_le[i * 4 + 3]) << 24);
        // bits 6-29: data, bit 29 = MSB of 24-bit data chunk
        uint32_t data24 = (w >> 6) & 0x00FFFFFFu;
        setbitu(buf, 24 * i, 24, data24);
    }
}

// Accumulate one GPS C/A subframe into the 90-byte buffer.
// sf_buf[90]: SF1 at [0:30], SF2 at [30:60], SF3 at [60:90].
// sf_rx: bit mask, bit k set means SF(k+1) received.
// Returns true when SF1+SF2+SF3 are all available.
inline bool gps_sf_accumulate(uint8_t sf_buf[90], uint8_t& sf_rx,
                               const uint8_t* navbits_le)
{
    uint8_t tmp[30];
    gps_navbits_to_buf(navbits_le, tmp);
    int id = static_cast<int>(getbitu(tmp, 43, 3)); // HOW word 2 bits 20-22
    if (id < 1 || id > 3) return false;
    std::memcpy(sf_buf + (id - 1) * 30, tmp, 30);
    sf_rx |= static_cast<uint8_t>(1u << (id - 1));
    return (sf_rx & 0x07u) == 0x07u;
}

// Decode GPS ephemeris from accumulated SF1+SF2+SF3 into GpsNavMsg.
// 'prn'  = GPS PRN  (1–32, from SBF block SVID).
// 'wnc'  = full GPS week number from SBF block header.
inline void gps_decode_nav_msg(const uint8_t sf_buf[90],
                               uint8_t prn, uint16_t wnc,
                               GpsNavMsg& msg)
{
    // Subframe bit-offsets in the 90-byte buffer (24 bits/word × 10 words each)
    static constexpr int SF1 = 0;
    static constexpr int SF2 = 240; // 30 bytes × 8 bits
    static constexpr int SF3 = 480;
    const uint8_t* b = sf_buf;

    msg.prn = prn;

    // --- Subframe 1 ---
    msg.wn            = static_cast<uint16_t>(getbitu(b, SF1 + 48, 10));
    msg.ca_or_p_on_l2 = static_cast<uint8_t> (getbitu(b, SF1 + 58,  2));
    msg.ura           = static_cast<uint8_t> (getbitu(b, SF1 + 60,  4));
    msg.health        = static_cast<uint8_t> (getbitu(b, SF1 + 64,  6));
    {
        uint32_t iodc_msb = getbitu(b, SF1 + 70, 2);
        uint32_t iodc_lsb = getbitu(b, SF1 + 168, 8);
        msg.iodc        = static_cast<uint16_t>((iodc_msb << 8) | iodc_lsb);
    }
    msg.l2_data_flag  = static_cast<uint8_t>(getbitu(b, SF1 + 72, 1));
    msg.t_gd          = static_cast<float>  (getbits(b, SF1 + 160, 8) * P2_31);
    msg.t_oc          = static_cast<uint32_t>(getbitu(b, SF1 + 176, 16) * 16u);
    msg.a_f2          = static_cast<float>  (getbits(b, SF1 + 192,  8) * P2_55);
    msg.a_f1          = static_cast<float>  (getbits(b, SF1 + 200, 16) * P2_43);
    msg.a_f0          = static_cast<float>  (getbits(b, SF1 + 216, 22) * P2_31);

    // --- Subframe 2 ---
    msg.iode2  = static_cast<uint8_t>(getbitu(b, SF2 + 48, 8));
    msg.c_rs   = static_cast<float> (getbits(b, SF2 + 56, 16) * P2_5);
    msg.del_n  = static_cast<float> (getbits(b, SF2 + 72, 16) * P2_43);
    {
        // M0: 32-bit signed = 8 MSBs (word 4) | 24 LSBs (word 5)
        uint32_t hi = getbitu(b, SF2 + 88,  8);
        uint32_t lo = getbitu(b, SF2 + 96,  24);
        msg.m_0 = static_cast<double>(static_cast<int32_t>((hi << 24) | lo)) * P2_31;
    }
    msg.c_uc   = static_cast<float>(getbits(b, SF2 + 120, 16) * P2_29);
    {
        // e: 32-bit unsigned = 8 MSBs (word 6) | 24 LSBs (word 7)
        uint32_t hi = getbitu(b, SF2 + 136,  8);
        uint32_t lo = getbitu(b, SF2 + 144, 24);
        msg.e = static_cast<double>((hi << 24) | lo) * P2_33;
    }
    msg.c_us   = static_cast<float>(getbits(b, SF2 + 168, 16) * P2_29);
    {
        // sqrt_A: 32-bit unsigned = 8 MSBs (word 8) | 24 LSBs (word 9)
        uint32_t hi = getbitu(b, SF2 + 184,  8);
        uint32_t lo = getbitu(b, SF2 + 192, 24);
        msg.sqrt_a = static_cast<double>((hi << 24) | lo) * P2_19;
    }
    msg.t_oe       = static_cast<uint32_t>(getbitu(b, SF2 + 216, 16) * 16u);
    msg.fit_int_flg = static_cast<uint8_t>(getbitu(b, SF2 + 232,  1));

    // --- Subframe 3 ---
    msg.c_ic   = static_cast<float>(getbits(b, SF3 + 48, 16) * P2_29);
    {
        // omega_0: 32-bit signed = 8 MSBs (word 3) | 24 LSBs (word 4)
        uint32_t hi = getbitu(b, SF3 + 64,  8);
        uint32_t lo = getbitu(b, SF3 + 72, 24);
        msg.omega_0 = static_cast<double>(static_cast<int32_t>((hi << 24) | lo)) * P2_31;
    }
    msg.c_is   = static_cast<float>(getbits(b, SF3 + 96, 16) * P2_29);
    {
        // i_0: 32-bit signed = 8 MSBs (word 5) | 24 LSBs (word 6)
        uint32_t hi = getbitu(b, SF3 + 112,  8);
        uint32_t lo = getbitu(b, SF3 + 120, 24);
        msg.i_0 = static_cast<double>(static_cast<int32_t>((hi << 24) | lo)) * P2_31;
    }
    msg.c_rc   = static_cast<float>(getbits(b, SF3 + 144, 16) * P2_5);
    {
        // omega: 32-bit signed = 8 MSBs (word 7) | 24 LSBs (word 8)
        uint32_t hi = getbitu(b, SF3 + 160,  8);
        uint32_t lo = getbitu(b, SF3 + 168, 24);
        msg.omega = static_cast<double>(static_cast<int32_t>((hi << 24) | lo)) * P2_31;
    }
    msg.omegadot = static_cast<float>(getbits(b, SF3 + 192, 24) * P2_43);
    msg.iode3    = static_cast<uint8_t>(getbitu(b, SF3 + 216, 8));
    msg.idot     = static_cast<float>(getbits(b, SF3 + 224, 14) * P2_43);

    // Use the full GPS week from the SBF block header
    msg.wnt_oc = wnc;
    msg.wnt_oe = wnc;
}

// ============================================================================
//  Galileo I/NAV  (SBF block 4023 — GALRawINAV)
// ============================================================================

// Convert 8 × u32 SBF NAVBits (LE) to a 32-byte buffer, MSB-first.
// bit 0 = first received = MSB of nav_bits[0].
inline void gal_navbits_to_buf(const uint8_t* navbits_le, int nwords,
                               uint8_t* buf)
{
    std::memset(buf, 0, static_cast<size_t>(nwords * 4));
    for (int i = 0; i < nwords; i++)
    {
        uint32_t w = static_cast<uint32_t>(navbits_le[i * 4]) |
                     (static_cast<uint32_t>(navbits_le[i * 4 + 1]) << 8) |
                     (static_cast<uint32_t>(navbits_le[i * 4 + 2]) << 16) |
                     (static_cast<uint32_t>(navbits_le[i * 4 + 3]) << 24);
        // Store big-endian so MSB of w → bit 0 of this chunk in the byte array
        buf[i * 4]     = static_cast<uint8_t>((w >> 24) & 0xFF);
        buf[i * 4 + 1] = static_cast<uint8_t>((w >> 16) & 0xFF);
        buf[i * 4 + 2] = static_cast<uint8_t>((w >>  8) & 0xFF);
        buf[i * 4 + 3] = static_cast<uint8_t>( w        & 0xFF);
    }
}

// Accumulate one Galileo I/NAV page into the 112-byte buffer.
// inav_buf[112]: word-type N stored at [N*16 : N*16+16).
// inav_rx: bit N set when word type N has been received.
// Returns true when word types 1–5 all available.
inline bool gal_inav_accumulate(uint8_t inav_buf[112], uint8_t& inav_rx,
                                const uint8_t* navbits_le)
{
    uint8_t raw[32];
    gal_navbits_to_buf(navbits_le, 8, raw);

    // Even/odd part indicators and page type
    int part1 = static_cast<int>(getbitu(raw,   0, 1)); // 0 = even
    int page1 = static_cast<int>(getbitu(raw,   1, 1)); // 0 = nominal
    int part2 = static_cast<int>(getbitu(raw, 114, 1)); // 1 = odd
    int page2 = static_cast<int>(getbitu(raw, 115, 1));
    if (part1 != 0 || part2 != 1) return false; // must be even+odd pair
    if (page1 || page2)           return false; // alert pages discarded

    int wt = static_cast<int>(getbitu(raw, 2, 6)); // word type 0–6
    if (wt > 6) return false;

    // Store 14 bytes from even page (bits 2–113) and 2 bytes from odd (116–131)
    for (int i = 0, j = 2; i < 14; i++, j += 8)
        inav_buf[wt * 16 + i] = static_cast<uint8_t>(getbitu(raw, j, 8));
    for (int i = 14, j = 116; i < 16; i++, j += 8)
        inav_buf[wt * 16 + i] = static_cast<uint8_t>(getbitu(raw, j, 8));

    inav_rx |= static_cast<uint8_t>(1u << wt);
    // Need word types 1, 2, 3, 4, 5 (bits 1–5)
    return (inav_rx & 0x3Eu) == 0x3Eu;
}

// Decode Galileo I/NAV ephemeris from 112-byte accumulated buffer into GalNavMsg.
// 'svid' = SBF Galileo SVID (71–106) from block header.
// 'wnc'  = GPS week from SBF block header.
inline bool gal_inav_decode_nav_msg(const uint8_t inav_buf[112],
                                    uint8_t svid, uint16_t wnc,
                                    GalNavMsg& msg)
{
    // Word type offset: type N starts at bit 128*N in inav_buf
    // (16 bytes × 8 bits = 128 bits per word type)

    // --- Word type 1 ---
    int i = 128;
    int type1  = static_cast<int>(getbitu(inav_buf, i, 6)); i += 6;
    int iodnav = static_cast<int>(getbitu(inav_buf, i, 10)); i += 10;
    double toes = getbitu(inav_buf, i, 14) * 60.0;          i += 14;
    double m0   = static_cast<double>(getbits(inav_buf, i, 32)) * P2_31; i += 32;
    double e    = static_cast<double>(getbitu(inav_buf, i, 32)) * P2_33; i += 32;
    double sqrtA= static_cast<double>(getbitu(inav_buf, i, 32)) * P2_19;

    // --- Word type 2 ---
    i = 128 * 2;
    int type2    = static_cast<int>(getbitu(inav_buf, i, 6)); i += 6;
    int iodnav2  = static_cast<int>(getbitu(inav_buf, i, 10)); i += 10;
    double omg0  = static_cast<double>(getbits(inav_buf, i, 32)) * P2_31; i += 32;
    double i0    = static_cast<double>(getbits(inav_buf, i, 32)) * P2_31; i += 32;
    double omg   = static_cast<double>(getbits(inav_buf, i, 32)) * P2_31; i += 32;
    double idot  = static_cast<double>(getbits(inav_buf, i, 14)) * P2_43;

    // --- Word type 3 ---
    i = 128 * 3;
    int type3    = static_cast<int>(getbitu(inav_buf, i, 6)); i += 6;
    int iodnav3  = static_cast<int>(getbitu(inav_buf, i, 10)); i += 10;
    double omgd  = static_cast<double>(getbits(inav_buf, i, 24)) * P2_43; i += 24;
    double dn    = static_cast<double>(getbits(inav_buf, i, 16)) * P2_43; i += 16;
    double cuc   = static_cast<double>(getbits(inav_buf, i, 16)) * P2_29; i += 16;
    double cus   = static_cast<double>(getbits(inav_buf, i, 16)) * P2_29; i += 16;
    double crc   = static_cast<double>(getbits(inav_buf, i, 16)) * P2_5;  i += 16;
    double crs   = static_cast<double>(getbits(inav_buf, i, 16)) * P2_5;  i += 16;
    uint32_t sisa = getbitu(inav_buf, i, 8);  // SISA_E1E5b

    // --- Word type 4 ---
    i = 128 * 4;
    int type4    = static_cast<int>(getbitu(inav_buf, i, 6)); i += 6;
    int iodnav4  = static_cast<int>(getbitu(inav_buf, i, 10)); i += 10;
    /*int svid_nav =*/ (void)getbitu(inav_buf, i, 6); i += 6; // ICD SVID (not used; we use SBF SVID)
    double cic   = static_cast<double>(getbits(inav_buf, i, 16)) * P2_29; i += 16;
    double cis   = static_cast<double>(getbits(inav_buf, i, 16)) * P2_29; i += 16;
    double toc   = getbitu(inav_buf, i, 14) * 60.0;             i += 14;
    double af0   = static_cast<double>(getbits(inav_buf, i, 31)) * P2_34; i += 31;
    double af1   = static_cast<double>(getbits(inav_buf, i, 21)) * P2_46; i += 21;
    double af2   = static_cast<double>(getbits(inav_buf, i,  6)) * P2_59;

    // --- Word type 5 ---
    i = 128 * 5;
    int type5    = static_cast<int>(getbitu(inav_buf, i, 6)); i += 6;
    i += 11 + 11 + 14 + 5; // skip ai0, ai1, ai2, storm flags
    double bgd_e5a = static_cast<double>(getbits(inav_buf, i, 10)) * P2_32; i += 10;
    double bgd_e5b = static_cast<double>(getbits(inav_buf, i, 10)) * P2_32; i += 10;
    uint32_t e5b_hs  = getbitu(inav_buf, i, 2); i += 2;
    uint32_t e1b_hs  = getbitu(inav_buf, i, 2); i += 2;
    uint32_t e5b_dvs = getbitu(inav_buf, i, 1); i += 1;
    uint32_t e1b_dvs = getbitu(inav_buf, i, 1); i += 1;
    uint32_t gst_wk  = getbitu(inav_buf, i, 12); i += 12; // GST week (12 bits)
    double   gst_tow = static_cast<double>(getbitu(inav_buf, i, 20)); // GST TOW

    // Validate word types
    if (type1 != 1 || type2 != 2 || type3 != 3 || type4 != 4 || type5 != 5)
        return false;
    // Validate IODnav consistency
    if (iodnav != iodnav2 || iodnav != iodnav3 || iodnav != iodnav4)
        return false;

    // GST-week adjustment: t_oe should be within ±302400 s of t_tr
    int wk = static_cast<int>(gst_wk);
    double dt = toes - gst_tow;
    if (dt >  302400.0) wk--;
    else if (dt < -302400.0) wk++;

    // Populate message fields
    msg.svid      = svid;
    msg.source    = 2; // I/NAV
    msg.iodnav    = static_cast<uint16_t>(iodnav);

    // Orbital parameters (semi-circles)
    msg.sqrt_a    = sqrtA;
    msg.m_0       = m0;
    msg.e         = e;
    msg.i_0       = i0;
    msg.omega     = omg;
    msg.omega_0   = omg0;
    msg.omegadot  = static_cast<float>(omgd);
    msg.idot      = static_cast<float>(idot);
    msg.del_n     = static_cast<float>(dn);

    // Harmonic corrections
    msg.c_uc  = static_cast<float>(cuc);
    msg.c_us  = static_cast<float>(cus);
    msg.c_rc  = static_cast<float>(crc);
    msg.c_rs  = static_cast<float>(crs);
    msg.c_ic  = static_cast<float>(cic);
    msg.c_is  = static_cast<float>(cis);

    // Reference times (in GPS-equivalent seconds)
    msg.t_oe    = static_cast<uint32_t>(toes);
    msg.t_oc    = static_cast<uint32_t>(toc);

    // Clock parameters
    msg.a_f2  = static_cast<float>(af2);
    msg.a_f1  = static_cast<float>(af1);
    msg.a_f0  = af0; // float64 in GalNavMsg

    // Week numbers (GPS = GST + 1024)
    uint16_t gps_wk = static_cast<uint16_t>(wk + 1024);
    msg.wnt_oe = gps_wk;
    msg.wnt_oc = gps_wk;

    // Signal health / DVS (health_ossol bitmask, per SBF GALNav convention)
    // Bit 0: L1B valid; bit 1: L1B_DVS; bits 2-3: L1B_HS
    // Bit 4: E5b valid; bit 5: E5b_DVS; bits 6-7: E5b_HS
    uint16_t health = 0;
    health |= (1u << 0);                       // L1B (E1B) info valid
    health |= (static_cast<uint16_t>(e1b_dvs) << 1);
    health |= (static_cast<uint16_t>(e1b_hs)  << 2);
    health |= (1u << 4);                       // E5b info valid
    health |= (static_cast<uint16_t>(e5b_dvs) << 5);
    health |= (static_cast<uint16_t>(e5b_hs)  << 6);
    msg.health_ossol = health;

    // SISA
    msg.sisa_l1e5b = static_cast<uint8_t>(sisa); // from word type 3
    msg.sisa_l1e5a = 0xFFu;                       // not available from I/NAV

    // BGD
    msg.bgd_l1e5a = static_cast<float>(bgd_e5a);
    msg.bgd_l1e5b = static_cast<float>(bgd_e5b);

    // wnc from block header is the GPS week; prefer it over computed gps_wk
    // if it is non-zero (it is always set for SBF file replay)
    if (wnc != 0)
    {
        msg.wnt_oe = wnc;
        msg.wnt_oc = wnc;
    }

    return true;
}

// ============================================================================
//  Galileo F/NAV  (SBF block 4022 — GALRawFNAV)
// ============================================================================

// Accumulate one Galileo F/NAV page (244 bits in 8 × u32) into 186-byte buffer.
// fnav_buf[186]: page-type N stored at [(N-1)*31 : N*31) for N = 1..6.
// fnav_rx: bits 0–5 set when page types 1–6 received.
// Returns true when page types 1–4 all available (enough for ephemeris).
inline bool gal_fnav_accumulate(uint8_t fnav_buf[186], uint8_t& fnav_rx,
                                const uint8_t* navbits_le)
{
    uint8_t raw[32];
    gal_navbits_to_buf(navbits_le, 8, raw);

    int ptype = static_cast<int>(getbitu(raw, 0, 6));
    if (ptype == 63) return false; // dummy page
    if (ptype < 1 || ptype > 6) return false;

    std::memcpy(fnav_buf + (ptype - 1) * 31, raw, 31);
    fnav_rx |= static_cast<uint8_t>(1u << (ptype - 1));
    // Need page types 1–4 (bits 0–3)
    return (fnav_rx & 0x0Fu) == 0x0Fu;
}

// Decode Galileo F/NAV ephemeris from 186-byte accumulated buffer into GalNavMsg.
// 'svid' = SBF Galileo SVID (71–106); 'wnc' = GPS week from SBF header.
inline bool gal_fnav_decode_nav_msg(const uint8_t fnav_buf[186],
                                    uint8_t svid, uint16_t wnc,
                                    GalNavMsg& msg)
{
    // --- Page type 1 ---
    int i = 0;
    int type1    = static_cast<int>(getbitu(fnav_buf, i, 6)); i += 6;
    /*svid_nav*/  i += 6;  // ICD SVID (not used)
    int iodnav   = static_cast<int>(getbitu(fnav_buf, i, 10)); i += 10;
    double toc   = getbitu(fnav_buf, i, 14) * 60.0;           i += 14;
    double af0   = static_cast<double>(getbits(fnav_buf, i, 31)) * P2_34; i += 31;
    double af1   = static_cast<double>(getbits(fnav_buf, i, 21)) * P2_46; i += 21;
    double af2   = static_cast<double>(getbits(fnav_buf, i,  6)) * P2_59; i += 6;
    uint32_t sisa = getbitu(fnav_buf, i, 8); i += 8;
    i += 11 + 11 + 14 + 5; // skip ai0, ai1, ai2, storm flags
    double bgd_e5a = static_cast<double>(getbits(fnav_buf, i, 10)) * P2_32; i += 10;
    uint32_t e5a_hs  = getbitu(fnav_buf, i, 2); i += 2;
    uint32_t gst_wk1 = getbitu(fnav_buf, i, 12); i += 12;
    double   gst_tow = static_cast<double>(getbitu(fnav_buf, i, 20)); i += 20;
    uint32_t e5a_dvs = getbitu(fnav_buf, i, 1);

    // --- Page type 2 ---
    i = 31 * 8;
    int type2    = static_cast<int>(getbitu(fnav_buf, i, 6)); i += 6;
    int iodnav2  = static_cast<int>(getbitu(fnav_buf, i, 10)); i += 10;
    double m0    = static_cast<double>(getbits(fnav_buf, i, 32)) * P2_31; i += 32;
    double omgd  = static_cast<double>(getbits(fnav_buf, i, 24)) * P2_43; i += 24;
    double e     = static_cast<double>(getbitu(fnav_buf, i, 32)) * P2_33; i += 32;
    double sqrtA = static_cast<double>(getbitu(fnav_buf, i, 32)) * P2_19; i += 32;
    double omg0  = static_cast<double>(getbits(fnav_buf, i, 32)) * P2_31; i += 32;
    double idot  = static_cast<double>(getbits(fnav_buf, i, 14)) * P2_43;

    // --- Page type 3 ---
    i = 62 * 8;
    int type3    = static_cast<int>(getbitu(fnav_buf, i, 6)); i += 6;
    int iodnav3  = static_cast<int>(getbitu(fnav_buf, i, 10)); i += 10;
    double i0    = static_cast<double>(getbits(fnav_buf, i, 32)) * P2_31; i += 32;
    double omg   = static_cast<double>(getbits(fnav_buf, i, 32)) * P2_31; i += 32;
    double dn    = static_cast<double>(getbits(fnav_buf, i, 16)) * P2_43; i += 16;
    double cuc   = static_cast<double>(getbits(fnav_buf, i, 16)) * P2_29; i += 16;
    double cus   = static_cast<double>(getbits(fnav_buf, i, 16)) * P2_29; i += 16;
    double crc   = static_cast<double>(getbits(fnav_buf, i, 16)) * P2_5;  i += 16;
    double crs   = static_cast<double>(getbits(fnav_buf, i, 16)) * P2_5;  i += 16;
    double toes  = getbitu(fnav_buf, i, 14) * 60.0;

    // --- Page type 4 ---
    i = 93 * 8;
    int type4    = static_cast<int>(getbitu(fnav_buf, i, 6)); i += 6;
    int iodnav4  = static_cast<int>(getbitu(fnav_buf, i, 10)); i += 10;
    double cic   = static_cast<double>(getbits(fnav_buf, i, 16)) * P2_29; i += 16;
    double cis   = static_cast<double>(getbits(fnav_buf, i, 16)) * P2_29;

    if (type1 != 1 || type2 != 2 || type3 != 3 || type4 != 4) return false;
    if (iodnav != iodnav2 || iodnav != iodnav3 || iodnav != iodnav4) return false;

    // GST-week adjustment
    int wk = static_cast<int>(gst_wk1);
    double dt = toes - gst_tow;
    if (dt >  302400.0) wk--;
    else if (dt < -302400.0) wk++;

    // Populate message fields
    msg.svid      = svid;
    msg.source    = 16; // F/NAV
    msg.iodnav    = static_cast<uint16_t>(iodnav);

    msg.sqrt_a    = sqrtA;
    msg.m_0       = m0;
    msg.e         = e;
    msg.i_0       = i0;
    msg.omega     = omg;
    msg.omega_0   = omg0;
    msg.omegadot  = static_cast<float>(omgd);
    msg.idot      = static_cast<float>(idot);
    msg.del_n     = static_cast<float>(dn);
    msg.c_uc      = static_cast<float>(cuc);
    msg.c_us      = static_cast<float>(cus);
    msg.c_rc      = static_cast<float>(crc);
    msg.c_rs      = static_cast<float>(crs);
    msg.c_ic      = static_cast<float>(cic);
    msg.c_is      = static_cast<float>(cis);

    msg.t_oe      = static_cast<uint32_t>(toes);
    msg.t_oc      = static_cast<uint32_t>(toc);
    msg.a_f2      = static_cast<float>(af2);
    msg.a_f1      = static_cast<float>(af1);
    msg.a_f0      = af0;

    uint16_t gps_wk = static_cast<uint16_t>(wk + 1024);
    msg.wnt_oe    = wnc ? wnc : gps_wk;
    msg.wnt_oc    = wnc ? wnc : gps_wk;

    // Health (E5a only available from F/NAV, bits 8–11)
    uint16_t health = 0;
    health |= (1u << 8);                        // E5a info valid
    health |= (static_cast<uint16_t>(e5a_dvs) << 9);
    health |= (static_cast<uint16_t>(e5a_hs)  << 10);
    msg.health_ossol  = health;
    msg.sisa_l1e5a    = static_cast<uint8_t>(sisa);
    msg.sisa_l1e5b    = 0xFFu;  // not available from F/NAV
    msg.bgd_l1e5a     = static_cast<float>(bgd_e5a);
    msg.bgd_l1e5b     = 0.0f;   // not available from F/NAV

    return true;
}

// ============================================================================
//  BeiDou D1  (SBF block 4047 — BDSRaw)
// ============================================================================

// Convert 10 × u32 SBF NAVBits (LE) to 40-byte buffer, MSB-first.
// bit 0 = MSB of nav_bits[0] = first received bit.
inline void bds_navbits_to_buf(const uint8_t* navbits_le, uint8_t buf[40])
{
    // re-use the generic helper
    gal_navbits_to_buf(navbits_le, 10, buf);
}

// Accumulate one BDS D1 subframe into the 114-byte buffer.
// d1_buf[114]: SF1 @ [0:38), SF2 @ [38:76), SF3 @ [76:114).
// d1_rx: bits 0–2 set when SF1–SF3 received.
// Returns true when SF1+SF2+SF3 available.
inline bool bds_d1_accumulate(uint8_t d1_buf[114], uint8_t& d1_rx,
                               const uint8_t* navbits_le)
{
    uint8_t tmp[40];
    bds_navbits_to_buf(navbits_le, tmp);
    int id = static_cast<int>(getbitu(tmp, 15, 3)); // subframe ID
    if (id < 1 || id > 3) return false;
    std::memcpy(d1_buf + (id - 1) * 38, tmp, 38);
    d1_rx |= static_cast<uint8_t>(1u << (id - 1));
    return (d1_rx & 0x07u) == 0x07u;
}

// Decode BDS D1 ephemeris from 114-byte buffer into BdsNavMsg.
// 'prn'  = BDS PRN (1–63) = SBF raw-block SVID − 160.
// 'wnc'  = GPS week from SBF block header (not used for BDS time).
//
// Bit-offset layout (from eph_decoder.h / RTKLIB conventions):
//   SF1 base = 8*38*0 =   0 bits
//   SF2 base = 8*38*1 = 304 bits
//   SF3 base = 8*38*2 = 608 bits
inline bool bds_d1_decode_nav_msg(const uint8_t d1_buf[114],
                                  uint8_t prn, uint16_t /*wnc*/,
                                  BdsNavMsg& msg)
{
    static constexpr int SF1 = 0;
    static constexpr int SF2 = 304; // 38*8
    static constexpr int SF3 = 608; // 76*8
    const uint8_t* b = d1_buf;

    // --- Subframe 1 ---
    int frn1 = static_cast<int>(getbitu(b, SF1 + 15, 3));
    uint32_t sow1 = getbitu2(b, SF1 + 18, 8, SF1 + 30, 12);
    uint32_t sat_h1  = getbitu(b, SF1 + 42, 1); // SatH1 health flag
    uint32_t iodc    = getbitu(b, SF1 + 43, 5); // AODC
    uint32_t sva     = getbitu(b, SF1 + 48, 4);
    uint32_t bds_wk  = getbitu(b, SF1 + 60, 13);
    double toc_bds = getbitu2(b, SF1 + 73, 9, SF1 + 90, 8) * 8.0;
    double tgd1    = static_cast<double>(getbits(b, SF1 + 98, 10)) * 0.1e-9;
    double tgd2    = static_cast<double>(getbits2(b, SF1 + 108, 4, SF1 + 120, 6)) * 0.1e-9;
    double af2     = static_cast<double>(getbits(b, SF1 + 214, 11)) * P2_66;
    double af0     = static_cast<double>(getbits2(b, SF1 + 225, 7, SF1 + 240, 17)) * P2_33;
    double af1     = static_cast<double>(getbits2(b, SF1 + 257, 5, SF1 + 270, 17)) * P2_50;
    uint32_t iode  = getbitu(b, SF1 + 287, 5); // AODE

    // --- Subframe 2 ---
    int frn2 = static_cast<int>(getbitu(b, SF2 + 15, 3));
    uint32_t sow2 = getbitu2(b, SF2 + 18, 8, SF2 + 30, 12);
    double dn    = static_cast<double>(getbits2(b, SF2 + 42, 10, SF2 + 60, 6)) * P2_43;
    double cuc   = static_cast<double>(getbits2(b, SF2 + 66, 16, SF2 + 90, 2)) * P2_31;
    double m0    = static_cast<double>(getbits2(b, SF2 + 92, 20, SF2 + 120, 12)) * P2_31;
    double e     = static_cast<double>(getbitu2(b, SF2 + 132, 10, SF2 + 150, 22)) * P2_33;
    double cus   = static_cast<double>(getbits(b, SF2 + 180, 18)) * P2_31;
    double crc   = static_cast<double>(getbits2(b, SF2 + 198, 4, SF2 + 210, 14)) * P2_6;
    double crs   = static_cast<double>(getbits2(b, SF2 + 224, 8, SF2 + 240, 10)) * P2_6;
    double sqrtA = static_cast<double>(getbitu2(b, SF2 + 250, 12, SF2 + 270, 20)) * P2_19;
    uint32_t toe1 = getbitu(b, SF2 + 290, 2); // TOE MSBs (2 bits)

    // --- Subframe 3 ---
    int frn3 = static_cast<int>(getbitu(b, SF3 + 15, 3));
    uint32_t sow3 = getbitu2(b, SF3 + 18, 8, SF3 + 30, 12);
    uint32_t toe2 = getbitu2(b, SF3 + 42, 10, SF3 + 60, 5); // TOE LSBs (15 bits)
    double i0    = static_cast<double>(getbits2(b, SF3 + 65, 17, SF3 + 90, 15)) * P2_31;
    double cic   = static_cast<double>(getbits2(b, SF3 + 105, 7, SF3 + 120, 11)) * P2_31;
    double omgd  = static_cast<double>(getbits2(b, SF3 + 131, 11, SF3 + 150, 13)) * P2_43;
    double cis   = static_cast<double>(getbits2(b, SF3 + 163, 9, SF3 + 180, 9)) * P2_31;
    double idot  = static_cast<double>(getbits2(b, SF3 + 189, 13, SF3 + 210, 1)) * P2_43;
    double omg0  = static_cast<double>(getbits2(b, SF3 + 211, 21, SF3 + 240, 11)) * P2_31;
    double omg   = static_cast<double>(getbits2(b, SF3 + 251, 11, SF3 + 270, 21)) * P2_31;
    double toes  = merge_two_u(toe1, toe2, 15) * 8.0; // BDT seconds

    // Consistency checks
    if (frn1 != 1 || frn2 != 2 || frn3 != 3) return false;
    if (sow2 != sow1 + 6 || sow3 != sow2 + 6) return false;
    if (toc_bds != toes) return false; // toe/toc mismatch

    // BDS week adjustment
    uint32_t wk = bds_wk;
    if (toes > static_cast<double>(sow1) + 302400.0) wk++;
    else if (toes < static_cast<double>(sow1) - 302400.0) wk--;

    // Populate message (angular values in semi-circles, times in BDT seconds)
    msg.prn       = prn;
    msg.wn        = static_cast<uint16_t>(wk);
    msg.ura       = static_cast<uint8_t>(sva);
    msg.sat_h1    = static_cast<uint8_t>(sat_h1);
    msg.iodc      = static_cast<uint8_t>(iodc);
    msg.iode      = static_cast<uint8_t>(iode);

    msg.t_gd1     = static_cast<float>(tgd1);
    msg.t_gd2     = static_cast<float>(tgd2);
    msg.t_oc      = static_cast<uint32_t>(toc_bds);
    msg.a_f2      = static_cast<float>(af2);
    msg.a_f1      = static_cast<float>(af1);
    msg.a_f0      = static_cast<float>(af0);

    msg.c_rs      = static_cast<float>(crs);
    msg.del_n     = static_cast<float>(dn);
    msg.m_0       = m0;   // semi-circles
    msg.c_uc      = static_cast<float>(cuc);
    msg.e         = e;
    msg.c_us      = static_cast<float>(cus);
    msg.sqrt_a    = sqrtA;
    msg.t_oe      = static_cast<uint32_t>(toes);
    msg.c_ic      = static_cast<float>(cic);
    msg.omega_0   = omg0;
    msg.c_is      = static_cast<float>(cis);
    msg.i_0       = i0;
    msg.c_rc      = static_cast<float>(crc);
    msg.omega     = omg;
    msg.omegadot  = static_cast<float>(omgd);
    msg.idot      = static_cast<float>(idot);

    msg.wnt_oc    = static_cast<uint16_t>(wk);
    msg.wnt_oe    = static_cast<uint16_t>(wk);

    return true;
}

} // namespace sbf_raw
