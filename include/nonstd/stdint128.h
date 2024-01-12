/*
 * copyright svante karlsson at csi dot se 2021
 */
#pragma once

__extension__ using int128_t = __int128_t;
__extension__ using uint128_t = __uint128_t;

static const uint128_t UINT128_MAX =uint128_t(int128_t(-1L));
static const int128_t  INT128_MAX = UINT128_MAX >> 1;
static const int128_t  INT128_MIN = -INT128_MAX - 1;

__extension__ constexpr __int128 pow10_128(unsigned int n) {
    return (n <= 0) ? 1 : (10LL * pow10_128(n - 1));
  }

