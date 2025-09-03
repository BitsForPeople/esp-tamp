#pragma once
#include <version>
#if __cpp_lib_span >= 202002L
#include <span>
#endif
#include <cstdint>

namespace tamp {

    #if __cpp_lib_span >= 202002L
    using byte_span = std::span<const uint8_t>;
    #else
    class byte_span {
        const uint8_t* m_data {nullptr};
        std::size_t m_len {0};
        public:
        constexpr byte_span() noexcept = default;
        constexpr byte_span(const byte_span&) noexcept = default;
        constexpr byte_span(const uint8_t* const data, const std::size_t len) :
            m_data {data},
            m_len {len} {

            }

        constexpr byte_span& operator =(const byte_span&) noexcept = default;

        constexpr const uint8_t* data() const noexcept {
            return m_data;
        }

        constexpr bool empty() const noexcept {
            return m_len == 0;
        }

        constexpr std::size_t size() const noexcept {
            return m_len;
        }

        constexpr std::size_t size_bytes() const noexcept {
            return m_len;
        }
    };
    #endif


} // namespace tamp