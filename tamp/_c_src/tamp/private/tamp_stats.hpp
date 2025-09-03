#pragma once
#include <cstdint>

#ifdef ESP_PLATFORM
#include <esp_log.h>
#endif

namespace tamp {

    template<bool EN>
    class Stats {

        public:

        static constexpr bool ENABLED = false;

        void log() const noexcept {
        } 

        constexpr void reset() noexcept {
        }

        constexpr void enterFind(const uint32_t dataLen, const uint32_t patLen) noexcept {
        }

        constexpr void potMatchesFound(const uint32_t cnt) noexcept {
        }

        constexpr void matchFound(const uint32_t patLen) noexcept {
        }

        constexpr void matchFound(const uint32_t patLen, const uint32_t searchLen) noexcept {
        }

        constexpr void noMatchFound(const uint32_t dataLen) noexcept {
        }               
    };

    template<>
    class Stats<true> {
        private:
            static constexpr const char* TAG = "Tamp Cstats";
        public:

        static constexpr bool ENABLED = true;

        uint32_t searchCnt {0}; // Number of pattern searches performed
        uint32_t cumPotSearchLen {0}; // Total number of data bytes given to pattern search
        uint32_t cumPatLen {0}; // Total number of pattern bytes given to pattern search
        uint32_t potMatchCnt {0}; // Number of potential/partial matches found
        uint32_t matchCnt {0}; // Number of full pattern matches found
        uint32_t cumSearchLen {0}; // Number of data bytes actually searched
        uint32_t cumMatchPatLen {0}; // Number of bytes found in matches

        // Needed as work-around for gcc bug
        // "error: default member initializer for ... required before the end of its enclosing class"
        constexpr Stats() noexcept = default;
        // :
        //     searchCnt {0},
        //     cumPotSearchLen {0},
        //     cumPatLen {0},
        //     potMatchCnt {0},
        //     matchCnt {0},
        //     cumSearchLen {0},
        //     cumMatchPatLen {0}
        //     {

        //     }



        void log() const noexcept {
#ifdef ESP_LOGI
            ESP_LOGI(TAG,
                "\n\tSearches: %" PRIu32
                "\n\tdata len: %" PRIu32
                "\n\tcum patlen: %" PRIu32
                "\n\tpot matches: %" PRIu32
                "\n\tmatches: %" PRIu32
                "\n\tsearched: %" PRIu32
                "\n\tmatched: %" PRIu32,
                this->searchCnt,
                this->cumPotSearchLen,
                this->cumPatLen,
                this->potMatchCnt,
                this->matchCnt,
                this->cumSearchLen,
                this->cumMatchPatLen                
            );
#endif
        }

        constexpr void reset() noexcept {
            searchCnt = 0;
            cumPotSearchLen = 0;
            cumPatLen = 0;
            potMatchCnt = 0;
            matchCnt = 0;
            cumSearchLen = 0;
            cumMatchPatLen = 0;
        }

        constexpr void enterFind(const uint32_t dataLen, const uint32_t patLen) noexcept {
            this->searchCnt += 1;
            this->cumPatLen += patLen;
            this->cumPotSearchLen += dataLen;
        }

        constexpr void potMatchesFound(const uint32_t cnt) noexcept {
            this->potMatchCnt += cnt;
        }

        constexpr void matchFound(const uint32_t patLen) noexcept {
            this->matchCnt += 1;
            this->cumMatchPatLen += patLen;
        }

        constexpr void matchFound(const uint32_t patLen, const uint32_t searchLen) noexcept {
            this->matchFound(patLen);
            this->cumSearchLen += searchLen;
        }

        constexpr void noMatchFound(const uint32_t dataLen) noexcept {
            this->cumSearchLen += dataLen;
        }

    };


} // namespace tamp