# ESP-Tamp Demo

Demonstrates data compression and decompression using the Tamp library on ESP32-series chips.

Two files are embedded into the application and used as input data for compression,
a 24-bit bitmap file ("[espressif-logo.bmp](main/espressif-logo.bmp)") and the first
10^5 (=100000) bytes of the enwik8 (text) dataset ("[enwik5.txt](main/enwik5.txt)").

The two embedded files are compressed, then decompressed again and the result verified to be identical
to the input.

On an ESP32-S3 @ 240MHz the log output looks like this:
```
I (272) main_task: Calling app_main()
I (272) tamp-demo: ======= Tamp demo =======
I (272) tamp-demo: [ESP32-optimized: YES]
I (272) tamp-demo: CPU @ 240MHz
I (282) tamp-demo: ======= Bitmap =======
I (282) tamp-demo: Compressing 25862 bytes of input using a window of 1024 bytes.
I (292) tamp-demo: === Incremental compression ===
I (312) tamp-demo: Compressed 25862 -> 6539 bytes (25.3%) in 14.5ms (1743.8kb/s).
I (312) tamp-demo: Decompressed to 25862 bytes (5.4ms).
I (322) tamp-demo: Decompressed data matches input data.
I (322) tamp-demo: === One-step compression ===
I (332) tamp-demo: Compressed 25862 -> 6539 bytes (25.3%) in 13.1ms (1932.4kb/s).
I (342) tamp-demo: Decompressed to 25862 bytes (5.4ms).
I (342) tamp-demo: Decompressed data matches input data.
I (342) tamp-demo: === Done ===
I (342) tamp-demo: ======= Text =======
I (352) tamp-demo: Compressing 100000 bytes of input using a window of 1024 bytes.
I (352) tamp-demo: === Incremental compression ===
I (502) tamp-demo: Compressed 100000 -> 51407 bytes (51.4%) in 143.1ms (682.5kb/s).
I (542) tamp-demo: Decompressed to 100000 bytes (36.4ms).
I (552) tamp-demo: Decompressed data matches input data.
I (552) tamp-demo: === One-step compression ===
I (682) tamp-demo: Compressed 100000 -> 51407 bytes (51.4%) in 139.4ms (700.4kb/s).
I (722) tamp-demo: Decompressed to 100000 bytes (36.4ms).
I (732) tamp-demo: Decompressed data matches input data.
I (732) tamp-demo: === Done ===
I (732) main_task: Returned from app_main()
```

