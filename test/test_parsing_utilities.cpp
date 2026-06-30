// *****************************************************************************
//
// © Copyright 2020, Septentrio NV/SA.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//    1. Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//    2. Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//    3. Neither the name of the copyright holder nor the names of its
//       contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#include <gtest/gtest.h>
#include <septentrio_gnss_driver/parsers/parsing_utilities.hpp>

TEST(WrapTest, angle180)
{
    {
        double val = 270.0;
        auto wrapped_val = parsing_utilities::wrapAngle180to180(val);

        EXPECT_EQ(wrapped_val, -90.0);
    }
    {
        double val = -270.0;
        auto wrapped_val = parsing_utilities::wrapAngle180to180(val);

        EXPECT_EQ(wrapped_val, 90.0);
    }
    {
        double val = -90.0;
        auto wrapped_val = parsing_utilities::wrapAngle180to180(val);

        EXPECT_EQ(wrapped_val, -90.0);
    }
    {
        double val = 90.0;
        auto wrapped_val = parsing_utilities::wrapAngle180to180(val);

        EXPECT_EQ(wrapped_val, 90.0);
    }
    {
        double val = 630.0;
        auto wrapped_val = parsing_utilities::wrapAngle180to180(val);

        EXPECT_EQ(wrapped_val, -90.0);
    }
    {
        double val = -630.0;
        auto wrapped_val = parsing_utilities::wrapAngle180to180(val);

        EXPECT_EQ(wrapped_val, 90.0);
    }
}

TEST(StaleGnssTimestamp, livePassesThrough)
{
    // Block GNSS time within ~120 ms of the host clock: not stale.
    const uint64_t host = 1700000000000000000ULL;
    const uint64_t gnss = host - 120000000ULL; // 120 ms earlier
    EXPECT_FALSE(parsing_utilities::isStaleGnssTimestamp(
        gnss, host, 5ULL * 1000000000ULL));
}

TEST(StaleGnssTimestamp, replayBacklogIsStale)
{
    // Receiver replay: block time 45 s behind the host clock with a 5 s gate.
    const uint64_t host = 1700000000000000000ULL;
    const uint64_t gnss = host - 45ULL * 1000000000ULL;
    EXPECT_TRUE(parsing_utilities::isStaleGnssTimestamp(
        gnss, host, 5ULL * 1000000000ULL));
}

TEST(StaleGnssTimestamp, futureBlockIsStale)
{
    // Symmetric: a block far ahead of the host clock is also rejected.
    const uint64_t host = 1700000000000000000ULL;
    const uint64_t gnss = host + 30ULL * 1000000000ULL;
    EXPECT_TRUE(parsing_utilities::isStaleGnssTimestamp(
        gnss, host, 5ULL * 1000000000ULL));
}

TEST(StaleGnssTimestamp, zeroThresholdDisables)
{
    // Threshold 0 disables the check even for a huge gap.
    const uint64_t host = 1700000000000000000ULL;
    const uint64_t gnss = host - 28975ULL * 1000000000ULL; // ~8 h behind
    EXPECT_FALSE(parsing_utilities::isStaleGnssTimestamp(gnss, host, 0ULL));
}

TEST(StaleGnssTimestamp, exactlyAtThresholdNotStale)
{
    // Difference equal to the threshold is allowed (strictly-greater rejects).
    const uint64_t host = 1700000000000000000ULL;
    const uint64_t threshold = 5ULL * 1000000000ULL;
    const uint64_t gnss = host - threshold;
    EXPECT_FALSE(parsing_utilities::isStaleGnssTimestamp(gnss, host, threshold));
}