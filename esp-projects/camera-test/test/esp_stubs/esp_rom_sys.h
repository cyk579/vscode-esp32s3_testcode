#pragma once
/* host stub: busy-wait delay is a no-op off-target */
static inline void esp_rom_delay_us(unsigned us) { (void)us; }
