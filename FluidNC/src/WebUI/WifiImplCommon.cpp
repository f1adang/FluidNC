#include "WifiImpl.h"

#include <Arduino.h>  // delay()
#include "Driver/watchdog.h"  // feed_watchdog()
#include "../Logging.h"

namespace WebUI {
    int32_t WifiImpl::beginApListScan() {
        // Block until a scan completes, driving the non-blocking primitives.
        // On platforms whose startApListScan() is itself synchronous this
        // returns after one pass.
        //
        // The WebUI reaches ESP410 through the async path, but a console
        // "$Wifi/ListAPs" still lands here, on the task that has to feed the
        // task watchdog.  A scan dwells about a second on each of the ~13
        // channels, so an unfed wait outlasts the watchdog even when the scan
        // succeeds, and a scan that never reports Done would wait forever:
        //
        //   task_wdt: Task watchdog got triggered ... loopTask (CPU 1)
        //   Tasks currently running:  CPU 0: IDLE0   CPU 1: IDLE1
        //
        // Both cores idle because the caller was parked in the delay below.
        // Poll faster, feed the watchdog, and give up rather than hang.
        const uint32_t poll_ms    = 100;
        const uint32_t timeout_ms = 30000;

        for (uint32_t waited = 0; waited < timeout_ms; waited += poll_ms) {
            startApListScan();
            if (apListScanState() == ApScanState::Done) {
                return apListCount();
            }
            delay(poll_ms);
            feed_watchdog();
        }
        log_warn("WiFi scan did not finish within " << (timeout_ms / 1000) << "s; reporting an empty AP list");
        return 0;
    }

    enum WiFiCountry {
        WiFiCountry01 = 0,
        WiFiCountryAT,
        WiFiCountryAU,
        WiFiCountryBE,
        WiFiCountryBG,
        WiFiCountryBR,
        WiFiCountryCA,
        WiFiCountryCH,
        WiFiCountryCN,
        WiFiCountryCY,
        WiFiCountryCZ,
        WiFiCountryDE,
        WiFiCountryDK,
        WiFiCountryEE,
        WiFiCountryES,
        WiFiCountryFI,
        WiFiCountryFR,
        WiFiCountryGB,
        WiFiCountryGR,
        WiFiCountryHK,
        WiFiCountryHR,
        WiFiCountryHU,
        WiFiCountryIE,
        WiFiCountryIN,
        WiFiCountryIS,
        WiFiCountryIT,
        WiFiCountryJP,
        WiFiCountryKR,
        WiFiCountryLI,
        WiFiCountryLT,
        WiFiCountryLU,
        WiFiCountryLV,
        WiFiCountryMT,
        WiFiCountryMX,
        WiFiCountryNL,
        WiFiCountryNO,
        WiFiCountryNZ,
        WiFiCountryPL,
        WiFiCountryPT,
        WiFiCountryRO,
        WiFiCountrySE,
        WiFiCountrySI,
        WiFiCountrySK,
        WiFiCountryTW,
        WiFiCountryUS,
    };

    static const enum_opt_t kWifiCountryOptionsMap = {
        { "01", WiFiCountry01 }, { "AT", WiFiCountryAT }, { "AU", WiFiCountryAU }, { "BE", WiFiCountryBE }, { "BG", WiFiCountryBG },
        { "BR", WiFiCountryBR }, { "CA", WiFiCountryCA }, { "CH", WiFiCountryCH }, { "CN", WiFiCountryCN }, { "CY", WiFiCountryCY },
        { "CZ", WiFiCountryCZ }, { "DE", WiFiCountryDE }, { "DK", WiFiCountryDK }, { "EE", WiFiCountryEE }, { "ES", WiFiCountryES },
        { "FI", WiFiCountryFI }, { "FR", WiFiCountryFR }, { "GB", WiFiCountryGB }, { "GR", WiFiCountryGR }, { "HK", WiFiCountryHK },
        { "HR", WiFiCountryHR }, { "HU", WiFiCountryHU }, { "IE", WiFiCountryIE }, { "IN", WiFiCountryIN }, { "IS", WiFiCountryIS },
        { "IT", WiFiCountryIT }, { "JP", WiFiCountryJP }, { "KR", WiFiCountryKR }, { "LI", WiFiCountryLI }, { "LT", WiFiCountryLT },
        { "LU", WiFiCountryLU }, { "LV", WiFiCountryLV }, { "MT", WiFiCountryMT }, { "MX", WiFiCountryMX }, { "NL", WiFiCountryNL },
        { "NO", WiFiCountryNO }, { "NZ", WiFiCountryNZ }, { "PL", WiFiCountryPL }, { "PT", WiFiCountryPT }, { "RO", WiFiCountryRO },
        { "SE", WiFiCountrySE }, { "SI", WiFiCountrySI }, { "SK", WiFiCountrySK }, { "TW", WiFiCountryTW }, { "US", WiFiCountryUS },
    };

    const enum_opt_t* getWifiCountryOptions() { return &kWifiCountryOptionsMap; }

    int getWifiCountryDefault() { return WiFiCountry01; }
}
