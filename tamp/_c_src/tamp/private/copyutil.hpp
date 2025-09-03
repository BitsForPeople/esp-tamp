/*

    Copyright 2024, <https://github.com/BitsForPeople>
     
    This program is free software: you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by the
    Free Software Foundation, either version 3 of the License, or (at your
    option) any later version.

    This program is distributed in the hope that it will be useful, bu
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
    for more details.

    You should have received a copy of the GNU General Public License along
    with this program. If not, see <https://www.gnu.org/licenses/>.
*/
#pragma once
#include <cstdint>
#include <type_traits>

namespace mem {
    namespace {
        template<int32_t INC, typename T>
        static inline void __attribute__((always_inline)) incptr(T*& ptr) noexcept {
            if constexpr (std::is_const_v<T>) {
                ptr = (T*)((const uint8_t*)ptr + INC);
            } else {
                ptr = (T*)((uint8_t*)ptr + INC);
            }
        }

        template<typename T>
        static inline T* pplus(T* const ptr, const int32_t inc) {
            if constexpr (std::is_const_v<T>) {
                return (T*)((const uint8_t*)ptr + inc);
            } else {
                return (T*)((uint8_t*)ptr + inc);
            }
        }

        template<int32_t INC, typename T>
        static inline T* pplus(T* const ptr) {
            if constexpr (std::is_const_v<T>) {
                return (T*)((const uint8_t*)ptr + INC);
            } else {
                return (T*)((uint8_t*)ptr + INC);
            }
        }


        using uint = unsigned int;
        static constexpr std::size_t WORD_SIZE = sizeof(uint);

        static constexpr std::size_t BLOCK_SIZE = 4*WORD_SIZE;        

        template<uint32_t CNT>
        static inline void __attribute__((always_inline)) _cpy(void* dst, const void* src) noexcept {

            if constexpr (CNT > 0) {

                using T = 
                        std::conditional_t<(CNT >= sizeof(uint)), uint, // Prefer to use native word size
                            std::conditional_t<(CNT >= sizeof(uint32_t)), uint32_t,
                                std::conditional_t<(CNT >= sizeof(uint16_t)), uint16_t,
                                    uint8_t
                                >
                            >
                        >;

                constexpr size_t S = sizeof(T);
                *(T*)dst = *(const T*)src;
                if constexpr (CNT > S) {
                    _cpy<CNT-S>(pplus<S>(dst),pplus<S>(src));
                }
            }
        }

        template<uint32_t N>
        static constexpr uint32_t log2() {
            static_assert( N != 0 );
            return 31 - __builtin_clz(N);
        }

        template<uint32_t N>
        requires( N > 0 )
        static constexpr uint32_t MSB = (1 << log2<N>());

        template<uint32_t N>
        static constexpr bool IS_POW2 = (N!=0) && ((N & (N-1)) == 0);

        /**
         * @brief Copies a number of bytes from \p src to \p dst.
         * The number of bytes copied is \c min( \p len \c , \p CEIL-1 \c ) , i.e.
         * 0 to \p CEIL-1 bytes can be copied.
         * 
         * @tparam CEIL maximum number of bytes that can by copied + 1
         * @param dst 
         * @param src 
         * @param len 
         */
        template<uint32_t CEIL = 16>
        requires (CEIL != 0 /* && (CEIL & (CEIL-1)) == 0 */)
        static inline void __attribute__((always_inline)) unrolled_cpy(void* dst, const void* src, std::size_t len) noexcept {
            static_assert( IS_POW2<BLOCK_SIZE> );

            if constexpr (CEIL > (2*BLOCK_SIZE)) {
                // You should probably be using std::memcpy...
                {
                    void* const dend = pplus(dst,(len/BLOCK_SIZE)*BLOCK_SIZE);
                    while(dst < dend) {
                        _cpy<BLOCK_SIZE>(dst,src);
                        incptr<BLOCK_SIZE>(dst);
                        incptr<BLOCK_SIZE>(src);
                    }
                    len = len % BLOCK_SIZE;
                }
                if(len) {
                    unrolled_cpy<BLOCK_SIZE>(dst,src,len);
                }
            } else
            if constexpr (!IS_POW2<CEIL>) {
                static_assert( MSB<CEIL> <= BLOCK_SIZE );                
                static_assert( (CEIL - MSB<CEIL>) < BLOCK_SIZE );
                if(len >= MSB<CEIL>) {
                    _cpy<CEIL-MSB<CEIL>>(dst,src);
                    incptr<CEIL-MSB<CEIL>>(dst);
                    incptr<CEIL-MSB<CEIL>>(src);
                    len -= CEIL-MSB<CEIL>;
                }
                unrolled_cpy<MSB<CEIL>>(dst,src,len);
            } else
            if constexpr (CEIL >= 2) {
                if(len & (CEIL/2)) {
                    _cpy<CEIL/2>(dst,src);
                    incptr<CEIL/2>(dst);
                    incptr<CEIL/2>(src);
                }
                unrolled_cpy<CEIL/2>(dst,src,len);
            }
        }
    } // anon namespace

    /**
     * @brief Copies a number of bytes from \p src to \p dst.
     * The number of bytes copied is \c min( \p len \c , \p CEIL-1 \c ) , i.e.
     * 0 to \p CEIL-1 bytes can be copied.
     * 
     * @tparam CEIL maximum number of bytes that can by copied + 1
     * @param dst 
     * @param src 
     * @param len 
     */
    template<uint32_t CEIL = 16>
    /* static */ inline void /* __attribute__((always_inline)) */ /* __attribute__((noinline)) */ cpy_short(void* dst, const void* src, const std::size_t len) noexcept {
        unrolled_cpy<CEIL>(dst,src,len);
    }

    /**
     * @brief Copies \p CNT bytes from \p src to \p dst
     * 
     * @tparam CNT number of bytes to copy
     * @param dst 
     * @param src 
     */
    template<uint32_t CNT>
    static inline void cpy_short(void* const dst, const void* const src) noexcept {
        _cpy<CNT>(dst,src);
    }

    template<uint32_t MAX>
    static inline void cpy_short_2(void* const dst, const void* const src, std::size_t len) noexcept {
        if constexpr (IS_POW2<MAX>) {
            if(len == MAX) {
                unrolled_cpy<2*MAX>(dst,src,MAX);
            } else {
                unrolled_cpy<MAX>(dst,src,len);
            }
        } else {
            unrolled_cpy<MAX+1>(dst,src,len);
        }
    }    

}