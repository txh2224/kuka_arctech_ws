#include "sha256.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <vector>

namespace tool {
namespace {

inline uint32_t rotr32(uint32_t x, uint32_t n) {
    return (x >> n) | (x << (32 - n));
}

}  // namespace

std::string sha256_hex(const std::string &data) {
    // SHA-256 定义的 64 个压缩轮常量。
    static constexpr std::array<uint32_t, 64> k = {
        0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u,
        0x3956c25bu, 0x59f111f1u, 0x923f82a4u, 0xab1c5ed5u,
        0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u,
        0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u, 0xc19bf174u,
        0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu,
        0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau,
        0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u,
        0xc6e00bf3u, 0xd5a79147u, 0x06ca6351u, 0x14292967u,
        0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu, 0x53380d13u,
        0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u,
        0xa2bfe8a1u, 0xa81a664bu, 0xc24b8b70u, 0xc76c51a3u,
        0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u,
        0x19a4c116u, 0x1e376c08u, 0x2748774cu, 0x34b0bcb5u,
        0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu, 0x682e6ff3u,
        0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u,
        0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u,
    };

    // 八个 32 位状态字组成 256 位摘要，每个数据块都会更新它们。
    std::array<uint32_t, 8> h = {
        0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
        0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u,
    };

    // 追加一个 1 位并补零，最后用 8 个字节按大端序写入原始消息的位长度。
    std::vector<uint8_t> msg(data.begin(), data.end());
    const uint64_t bit_len = static_cast<uint64_t>(msg.size()) * 8u;
    msg.push_back(0x80u);
    while ((msg.size() % 64u) != 56u) {
        msg.push_back(0x00u);
    }
    for (int i = 7; i >= 0; --i) {
        msg.push_back(static_cast<uint8_t>((bit_len >> (i * 8)) & 0xffu));
    }

    for (std::size_t chunk = 0; chunk < msg.size(); chunk += 64u) {
        // 每块 64 字节先读成 16 个大端序的字，再扩展成 64 个供压缩使用的消息字。
        uint32_t w[64];
        for (int i = 0; i < 16; ++i) {
            const std::size_t idx = chunk + static_cast<std::size_t>(i) * 4u;
            w[i] = (static_cast<uint32_t>(msg[idx]) << 24) |
                   (static_cast<uint32_t>(msg[idx + 1]) << 16) |
                   (static_cast<uint32_t>(msg[idx + 2]) << 8) |
                   (static_cast<uint32_t>(msg[idx + 3]));
        }
        for (int i = 16; i < 64; ++i) {
            const uint32_t s0 = rotr32(w[i - 15], 7) ^ rotr32(w[i - 15], 18) ^ (w[i - 15] >> 3);
            const uint32_t s1 = rotr32(w[i - 2], 17) ^ rotr32(w[i - 2], 19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16] + s0 + w[i - 7] + s1;
        }

        // 完成 64 轮压缩后，将工作变量累加回摘要状态。
        uint32_t a = h[0], b = h[1], c = h[2], d = h[3];
        uint32_t e = h[4], f = h[5], g = h[6], hh = h[7];

        for (int i = 0; i < 64; ++i) {
            const uint32_t S1 = rotr32(e, 6) ^ rotr32(e, 11) ^ rotr32(e, 25);
            const uint32_t ch = (e & f) ^ ((~e) & g);
            const uint32_t temp1 = hh + S1 + ch + k[i] + w[i];
            const uint32_t S0 = rotr32(a, 2) ^ rotr32(a, 13) ^ rotr32(a, 22);
            const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
            const uint32_t temp2 = S0 + maj;

            hh = g;
            g = f;
            f = e;
            e = d + temp1;
            d = c;
            c = b;
            b = a;
            a = temp1 + temp2;
        }

        h[0] += a;
        h[1] += b;
        h[2] += c;
        h[3] += d;
        h[4] += e;
        h[5] += f;
        h[6] += g;
        h[7] += hh;
    }

    // 八个状态字各输出 8 个十六进制字符，保留前导零。
    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    for (uint32_t word : h) {
        oss << std::setw(8) << word;
    }
    return oss.str();
}

}  // namespace tool
