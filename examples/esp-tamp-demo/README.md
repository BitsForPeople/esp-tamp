# ESP-Tamp Demo

Demonstrates data compression and decompression using the Tamp library on ESP32-series chips.

Two files are embedded into the application and used as input data for compression,
a 24-bit bitmap file ("[espressif-logo.bmp](main/espressif-logo.bmp)") and the first
10^5 (=100000) bytes of the enwik8 (text) dataset ("[enwik5.txt](main/enwik5.txt)").

The two embedded files are compressed, then decompressed again, and the result verified to be identical
to the input.

On an ESP32-S3 @ 240MHz the log output looks like this:
```
I (267) main_task: Calling app_main()
I (267) tamp-demo: ======= Tamp demo =======
I (267) tamp-demo: [ESP32-optimized: YES]
I (267) tamp-demo: CPU @ 240MHz
I (277) tamp-demo: ======= Bitmap =======
I (277) tamp-demo: Compressing 25862 bytes of input using a window of 1024 bytes.
I (287) tamp-demo: === Incremental compression ===
I (307) tamp-demo: Compressed 25862 -> 6538 bytes (25.3%) in 14.3ms (1768.4KB/s).
I (307) tamp-demo: Decompressed to 25862 bytes (3.3ms).
I (307) tamp-demo: Decompressed data matches input data.
I (307) tamp-demo: === One-step compression ===
I (327) tamp-demo: Compressed 25862 -> 6538 bytes (25.3%) in 13.1ms (1932.6KB/s).
I (337) tamp-demo: Decompressed to 25862 bytes (3.2ms).
I (337) tamp-demo: Decompressed data matches input data.
I (337) tamp-demo: === Done ===
I (337) tamp-demo: ======= Text =======
I (347) tamp-demo: Compressing 100000 bytes of input using a window of 1024 bytes.
I (347) tamp-demo: === Incremental compression ===
I (497) tamp-demo: Compressed 100000 -> 51405 bytes (51.4%) in 142.9ms (683.6KB/s).
I (527) tamp-demo: Decompressed to 100000 bytes (27.2ms).
I (527) tamp-demo: Decompressed data matches input data.
I (527) tamp-demo: === One-step compression ===
I (667) tamp-demo: Compressed 100000 -> 51405 bytes (51.4%) in 139.2ms (701.7KB/s).
I (697) tamp-demo: Decompressed to 100000 bytes (27.2ms).
I (707) tamp-demo: Decompressed data matches input data.
I (707) tamp-demo: === Done ===
I (707) main_task: Returned from app_main()
```

