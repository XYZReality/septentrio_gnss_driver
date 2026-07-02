// *****************************************************************************
//
// Test: GPSNavParser, GALNavParser, BDSNavParser, plus the raw-subframe decoder
// Verifies that the ephemeris SBF block parsers correctly extract all fields
// from a known synthetic byte buffer, and that the raw GPS subframe decoder
// maps bit fields back to the expected values (round trip).
//
// *****************************************************************************

#include <gtest/gtest.h>
#include <cstring>
#include <vector>

#include <septentrio_gnss_driver/parsers/sbf_blocks.hpp>
#include <septentrio_gnss_driver/parsers/sbf_raw_decode.hpp>
#include <septentrio_gnss_driver/abstraction/typedefs.hpp>

namespace {

// Helper: append a value to a byte buffer (little-endian).
template <typename T>
void appendLE(std::vector<uint8_t>& buf, T val)
{
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&val);
    for (size_t i = 0; i < sizeof(T); ++i)
        buf.push_back(p[i]);
}

// Build a minimal valid SBF BlockHeader byte sequence.
// block_id must fit in lower 13 bits of the ID word.
void appendBlockHeader(std::vector<uint8_t>& buf, uint16_t block_id,
                       uint16_t total_length)
{
    appendLE<uint8_t>(buf, 0x24);          // sync1
    appendLE<uint8_t>(buf, 0x40);          // sync2
    appendLE<uint16_t>(buf, 0x0000);       // crc  (not checked by parser)
    appendLE<uint16_t>(buf, block_id);     // id word (revision=0, id=block_id)
    appendLE<uint16_t>(buf, total_length); // length
    appendLE<uint32_t>(buf, 399000);       // tow (ms)
    appendLE<uint16_t>(buf, 2300);         // wnc (GPS week)
}

} // namespace

// ---------------------------------------------------------------------------
// GPSNavParser test
// ---------------------------------------------------------------------------
TEST(SbfEphemerisParser, GPSNavAllFields)
{
    std::vector<uint8_t> buf;

    // Block total length = 14 (header) + 126 (payload) = 140 bytes
    appendBlockHeader(buf, 5891, 140);

    // Known test values
    const uint8_t  prn          = 7;
    const uint16_t wn           = 2300;
    const uint8_t  ca_or_p      = 1;
    const uint8_t  ura          = 2;
    const uint8_t  health       = 0;
    const uint8_t  l2_flag      = 0;
    const uint16_t iodc         = 123;
    const uint8_t  iode2        = 45;
    const uint8_t  iode3        = 45;
    const uint8_t  fit_int      = 0;
    const float    t_gd         = -1.862645e-9f;
    const uint32_t t_oc         = 388800u;
    const float    a_f2         = 0.0f;
    const float    a_f1         = -3.637979e-12f;
    const float    a_f0         = -0.000286102295f;
    const float    c_rs         = -31.5625f;
    const float    del_n        = 4.3540e-9f;
    const double   m_0          = 0.31415926535897932;
    const float    c_uc         = -1.676381e-6f;
    const double   e            = 0.00549316406;
    const float    c_us         = 9.313226e-7f;
    const double   sqrt_a       = 5153.7957764;
    const uint32_t t_oe         = 388800u;
    const float    c_ic         = 5.587935e-8f;
    const double   omega_0      = -0.94294572;
    const float    c_is         = -1.117587e-8f;
    const double   i_0          = 0.95690918;
    const float    c_rc         = 280.34375f;
    const double   omega        = 0.29905295;
    const float    omegadot     = -8.081749e-9f;
    const float    idot         = -1.490116e-10f;
    const uint16_t wnt_oc       = 2300;
    const uint16_t wnt_oe       = 2300;

    appendLE<uint8_t>(buf,  prn);
    appendLE<uint8_t>(buf,  0);          // reserved
    appendLE<uint16_t>(buf, wn);
    appendLE<uint8_t>(buf,  ca_or_p);
    appendLE<uint8_t>(buf,  ura);
    appendLE<uint8_t>(buf,  health);
    appendLE<uint8_t>(buf,  l2_flag);
    appendLE<uint16_t>(buf, iodc);
    appendLE<uint8_t>(buf,  iode2);
    appendLE<uint8_t>(buf,  iode3);
    appendLE<uint8_t>(buf,  fit_int);
    appendLE<uint8_t>(buf,  0);          // reserved2
    appendLE<float>(buf,    t_gd);
    appendLE<uint32_t>(buf, t_oc);
    appendLE<float>(buf,    a_f2);
    appendLE<float>(buf,    a_f1);
    appendLE<float>(buf,    a_f0);
    appendLE<float>(buf,    c_rs);
    appendLE<float>(buf,    del_n);
    appendLE<double>(buf,   m_0);
    appendLE<float>(buf,    c_uc);
    appendLE<double>(buf,   e);
    appendLE<float>(buf,    c_us);
    appendLE<double>(buf,   sqrt_a);
    appendLE<uint32_t>(buf, t_oe);
    appendLE<float>(buf,    c_ic);
    appendLE<double>(buf,   omega_0);
    appendLE<float>(buf,    c_is);
    appendLE<double>(buf,   i_0);
    appendLE<float>(buf,    c_rc);
    appendLE<double>(buf,   omega);
    appendLE<float>(buf,    omegadot);
    appendLE<float>(buf,    idot);
    appendLE<uint16_t>(buf, wnt_oc);
    appendLE<uint16_t>(buf, wnt_oe);

    GpsNavMsg msg;
    // nullptr is safe here: all data is valid, no error-path logging occurs.
    bool ok = GPSNavParser(nullptr, buf.begin(), buf.end(), msg);

    ASSERT_TRUE(ok);
    EXPECT_EQ(msg.block_header.id, 5891);
    EXPECT_EQ(msg.block_header.tow, 399000u);
    EXPECT_EQ(msg.block_header.wnc, 2300);
    EXPECT_EQ(msg.prn,           prn);
    EXPECT_EQ(msg.wn,            wn);
    EXPECT_EQ(msg.ura,           ura);
    EXPECT_EQ(msg.health,        health);
    EXPECT_EQ(msg.iodc,          iodc);
    EXPECT_EQ(msg.iode2,         iode2);
    EXPECT_EQ(msg.iode3,         iode3);
    EXPECT_FLOAT_EQ(msg.t_gd,    t_gd);
    EXPECT_EQ(msg.t_oc,          t_oc);
    EXPECT_FLOAT_EQ(msg.a_f1,    a_f1);
    EXPECT_FLOAT_EQ(msg.a_f0,    a_f0);
    EXPECT_FLOAT_EQ(msg.c_rs,    c_rs);
    EXPECT_DOUBLE_EQ(msg.m_0,    m_0);
    EXPECT_DOUBLE_EQ(msg.e,      e);
    EXPECT_DOUBLE_EQ(msg.sqrt_a, sqrt_a);
    EXPECT_EQ(msg.t_oe,          t_oe);
    EXPECT_DOUBLE_EQ(msg.omega_0, omega_0);
    EXPECT_DOUBLE_EQ(msg.i_0,    i_0);
    EXPECT_DOUBLE_EQ(msg.omega,  omega);
    EXPECT_FLOAT_EQ(msg.omegadot, omegadot);
    EXPECT_FLOAT_EQ(msg.idot,    idot);
    EXPECT_EQ(msg.wnt_oe,        wnt_oe);
}

// ---------------------------------------------------------------------------
// GALNavParser test
// ---------------------------------------------------------------------------
TEST(SbfEphemerisParser, GALNavBlockId)
{
    std::vector<uint8_t> buf;
    // Block header: 14 bytes.
    // GALNav payload consumed by GALNavParser (matches SBF ref guide field
    // order, including the reserved bytes the parser skips):
    //   svid(1)+source(1)+sqrt_a(8)+m_0(8)+e(8)+i_0(8)+omega(8)+omega_0(8)+
    //   omegadot(4)+idot(4)+del_n(4)+c_uc(4)+c_us(4)+c_rc(4)+c_rs(4)+c_ic(4)+
    //   c_is(4)+t_oe(4)+t_oc(4)+a_f2(4)+a_f1(4)+a_f0(8)+wnt_oe(2)+wnt_oc(2)+
    //   iodnav(2)+health_ossol(2)+Health_PRS(1)+sisa_l1e5a(1)+sisa_l1e5b(1)+
    //   SISA_L1AE6A(1)+bgd_l1e5a(4)+bgd_l1e5b(4)+BGD_L1AE6A(4)+CNAVenc(1) = 135
    // Padded 3 bytes to a 4-byte boundary -> 138 payload.
    // Total = 14 + 138 = 152.
    const uint16_t total_length = 152;
    appendBlockHeader(buf, 4002, total_length);

    appendLE<uint8_t>(buf, 11);    // svid
    appendLE<uint8_t>(buf, 2);     // source (I/NAV)
    appendLE<double>(buf,  5153.6); // sqrt_a
    appendLE<double>(buf,  0.314);  // m_0
    appendLE<double>(buf,  0.001);  // e
    appendLE<double>(buf,  0.956);  // i_0
    appendLE<double>(buf,  0.299);  // omega
    appendLE<double>(buf, -0.942);  // omega_0
    appendLE<float>(buf,  -8.0e-9f); // omegadot
    appendLE<float>(buf,   0.0f);    // idot
    appendLE<float>(buf,   4.4e-9f); // del_n
    appendLE<float>(buf,   0.0f);    // c_uc
    appendLE<float>(buf,   0.0f);    // c_us
    appendLE<float>(buf,   0.0f);    // c_rc
    appendLE<float>(buf,   0.0f);    // c_rs
    appendLE<float>(buf,   0.0f);    // c_ic
    appendLE<float>(buf,   0.0f);    // c_is
    appendLE<uint32_t>(buf, 388800u); // t_oe
    appendLE<uint32_t>(buf, 388800u); // t_oc
    appendLE<float>(buf,   0.0f);    // a_f2
    appendLE<float>(buf,  -3.6e-12f); // a_f1
    appendLE<double>(buf, -2.8e-4);  // a_f0
    appendLE<uint16_t>(buf, 2300);   // wnt_oe
    appendLE<uint16_t>(buf, 2300);   // wnt_oc
    appendLE<uint16_t>(buf, 55);     // iodnav
    appendLE<uint16_t>(buf, 0);      // health_ossol
    appendLE<uint8_t>(buf, 0);       // Health_PRS (skipped by parser)
    appendLE<uint8_t>(buf, 107);     // sisa_l1e5a
    appendLE<uint8_t>(buf, 107);     // sisa_l1e5b
    appendLE<uint8_t>(buf, 255);     // SISA_L1AE6A (skipped by parser)
    appendLE<float>(buf,  2.3e-9f);  // bgd_l1e5a
    appendLE<float>(buf,  2.1e-9f);  // bgd_l1e5b
    appendLE<float>(buf,  0.0f);     // BGD_L1AE6A (f4, skipped by parser)
    appendLE<uint8_t>(buf, 0);       // CNAVenc (u1, skipped by parser)
    appendLE<uint8_t>(buf, 0);       // padding to 4-byte block boundary
    appendLE<uint8_t>(buf, 0);       // padding
    appendLE<uint8_t>(buf, 0);       // padding

    GalNavMsg msg;
    bool ok = GALNavParser(nullptr, buf.begin(), buf.end(), msg);

    ASSERT_TRUE(ok);
    EXPECT_EQ(msg.block_header.id, 4002);
    EXPECT_EQ(msg.svid,   11);
    EXPECT_EQ(msg.source, 2);
    EXPECT_DOUBLE_EQ(msg.sqrt_a, 5153.6);
    EXPECT_EQ(msg.iodnav, 55);
    EXPECT_EQ(msg.wnt_oe, 2300);
}

// ---------------------------------------------------------------------------
// BDSNavParser test
// ---------------------------------------------------------------------------
TEST(SbfEphemerisParser, BDSNavBlockId)
{
    std::vector<uint8_t> buf;
    // BDSNav payload: prn(1)+reserved(1)+wn(2)+ura(1)+sat_h1(1)+iodc(1)+iode(1)+
    //   reserved2(2)+t_gd1(4)+t_gd2(4)+t_oc(4)+a_f2(4)+a_f1(4)+a_f0(4)+
    //   c_rs(4)+del_n(4)+m_0(8)+c_uc(4)+e(8)+c_us(4)+sqrt_a(8)+t_oe(4)+
    //   c_ic(4)+omega_0(8)+c_is(4)+i_0(8)+c_rc(4)+omega(8)+omegadot(4)+
    //   idot(4)+wnt_oc(2)+wnt_oe(2) = 126
    // Total = 14 + 126 = 140
    const uint16_t total_length = 140;
    appendBlockHeader(buf, 4081, total_length);

    appendLE<uint8_t>(buf, 21);     // prn
    appendLE<uint8_t>(buf, 0);      // reserved
    appendLE<uint16_t>(buf, 900);   // wn (BeiDou week)
    appendLE<uint8_t>(buf, 3);      // ura
    appendLE<uint8_t>(buf, 0);      // sat_h1
    appendLE<uint8_t>(buf, 5);      // iodc
    appendLE<uint8_t>(buf, 5);      // iode
    appendLE<uint16_t>(buf, 0);     // reserved2
    appendLE<float>(buf, -6.0e-9f); // t_gd1 (B1I)
    appendLE<float>(buf, -5.8e-9f); // t_gd2 (B2I)
    appendLE<uint32_t>(buf, 390000u); // t_oc (BDT)
    appendLE<float>(buf, 0.0f);     // a_f2
    appendLE<float>(buf, 0.0f);     // a_f1
    appendLE<float>(buf, 1.4e-4f);  // a_f0
    appendLE<float>(buf, 50.25f);   // c_rs
    appendLE<float>(buf, 4.5e-9f);  // del_n
    appendLE<double>(buf, 0.628);   // m_0
    appendLE<float>(buf, 2.0e-6f);  // c_uc
    appendLE<double>(buf, 0.003);   // e
    appendLE<float>(buf, 3.0e-6f);  // c_us
    appendLE<double>(buf, 5282.6);  // sqrt_a
    appendLE<uint32_t>(buf, 390000u); // t_oe (BDT)
    appendLE<float>(buf, 0.0f);     // c_ic
    appendLE<double>(buf, 0.5);     // omega_0
    appendLE<float>(buf, 0.0f);     // c_is
    appendLE<double>(buf, 0.9);     // i_0
    appendLE<float>(buf, 200.0f);   // c_rc
    appendLE<double>(buf, 0.8);     // omega
    appendLE<float>(buf, -7.0e-9f); // omegadot
    appendLE<float>(buf, 0.0f);     // idot
    appendLE<uint16_t>(buf, 900);   // wnt_oc
    appendLE<uint16_t>(buf, 900);   // wnt_oe

    BdsNavMsg msg;
    bool ok = BDSNavParser(nullptr, buf.begin(), buf.end(), msg);

    ASSERT_TRUE(ok);
    EXPECT_EQ(msg.block_header.id, 4081);
    EXPECT_EQ(msg.prn,  21);
    EXPECT_EQ(msg.wn,   900);
    EXPECT_EQ(msg.iodc, 5);
    EXPECT_EQ(msg.iode, 5);
    EXPECT_FLOAT_EQ(msg.t_gd1, -6.0e-9f);
    EXPECT_FLOAT_EQ(msg.t_gd2, -5.8e-9f);
    EXPECT_DOUBLE_EQ(msg.sqrt_a, 5282.6);
    EXPECT_EQ(msg.wnt_oe, 900);
}

// ---------------------------------------------------------------------------
// Raw GPS subframe decoder round trip
// Encodes known bit-field values into a 90-byte SF1/SF2/SF3 buffer with the
// same MSB-first convention the decoder uses, then verifies gps_decode_nav_msg
// maps each field back (bit offset, sign handling and scale factor).
// ---------------------------------------------------------------------------
TEST(SbfRawDecode, GpsSubframeDecodeRoundTrip)
{
    using namespace sbf_raw;

    uint8_t sf[90];
    std::memset(sf, 0, sizeof(sf));
    constexpr int SF1 = 0;
    constexpr int SF2 = 240; // 30 bytes
    constexpr int SF3 = 480;

    // Known raw bit-field values (as they appear on the wire, pre-scaling)
    const uint32_t raw_wn       = 678;      // 10-bit field (max 1023)
    const uint32_t raw_ura      = 5;
    const uint32_t raw_health   = 0;
    const uint32_t raw_iode2    = 77;
    const uint32_t raw_iode3    = 77;
    const uint32_t raw_e_hi     = 0x00;     // 8 bits
    const uint32_t raw_e_lo     = 0x123456; // 24 bits
    const uint32_t raw_sqrta_hi = 0x2A;     // 8 bits
    const uint32_t raw_sqrta_lo = 0x654321; // 24 bits
    const int32_t  raw_m0       = -100000;  // 32-bit signed

    // Subframe 1
    setbitu(sf, SF1 + 48, 10, raw_wn);
    setbitu(sf, SF1 + 60,  4, raw_ura);
    setbitu(sf, SF1 + 64,  6, raw_health);
    // Subframe 2
    setbitu(sf, SF2 + 48,  8, raw_iode2);
    setbitu(sf, SF2 + 136, 8, raw_e_hi);       // e (u32) high 8 bits
    setbitu(sf, SF2 + 144, 24, raw_e_lo);      // e (u32) low 24 bits
    setbitu(sf, SF2 + 184, 8, raw_sqrta_hi);   // sqrt_a (u32) high 8 bits
    setbitu(sf, SF2 + 192, 24, raw_sqrta_lo);  // sqrt_a (u32) low 24 bits
    setbitu(sf, SF2 + 88,  8,
            (static_cast<uint32_t>(raw_m0) >> 24) & 0xFF);   // m_0 (s32) high 8
    setbitu(sf, SF2 + 96, 24,
            static_cast<uint32_t>(raw_m0) & 0xFFFFFF);       // m_0 (s32) low 24
    // Subframe 3
    setbitu(sf, SF3 + 216, 8, raw_iode3);

    GpsNavMsg msg;
    gps_decode_nav_msg(sf, /*prn=*/7, /*wnc=*/2300, msg);

    EXPECT_EQ(msg.prn,    7u);
    EXPECT_EQ(msg.wn,     raw_wn);
    EXPECT_EQ(msg.ura,    raw_ura);
    EXPECT_EQ(msg.health, raw_health);
    EXPECT_EQ(msg.iode2,  raw_iode2);
    EXPECT_EQ(msg.iode3,  raw_iode3);

    const double exp_e = static_cast<double>((raw_e_hi << 24) | raw_e_lo) * P2_33;
    EXPECT_DOUBLE_EQ(msg.e, exp_e);

    const double exp_sqrta =
        static_cast<double>((raw_sqrta_hi << 24) | raw_sqrta_lo) * P2_19;
    EXPECT_DOUBLE_EQ(msg.sqrt_a, exp_sqrta);

    EXPECT_DOUBLE_EQ(msg.m_0, static_cast<double>(raw_m0) * P2_31);

    // wnc from the SBF header is copied into both week-number fields
    EXPECT_EQ(msg.wnt_oe, 2300u);
    EXPECT_EQ(msg.wnt_oc, 2300u);
}

