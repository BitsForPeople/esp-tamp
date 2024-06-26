Tamp for ESP32
==============

This is a modification of the Tamp compression library created by `Brian Pugh <https://github.com/BrianPugh>`,
aggressively speed-optimized for use as a C/C++ library on ESP32 MCUs.

The C implementation of the original Tamp library is made into a component for use in ESP-IDF builds and
speed-optimized for 32-bit architectures like the ESP32s', including faster pattern-match search functions.

While the original Tamp C code is compatible with most (/all?) architectures, this optimized variant leverages
the ESP32s' ability to do unaligned 16- and 32-bit memory accesses. It should be able to run on architectures
other than ESP32s (e.g. other RISC-V CPUs), but it will *not* work on chips which don't support unaligned memory
accesses (e.g. the Raspberry Pi Pico RP2040).

The performance optimizations work out to about 2.5-4x faster compression on ESP32s (both Xtensa and RISC-V,
i.e. ESP32, ESP32-C2, ESP32-S3, &c.).
When built for an **ESP32-S3**, the MCU's SIMD instructions ("PIE") are used which increases compression speed/
throughput by a factor of 8-20x.

When building the C library as a component in an ESP-IDF build, you can choose to build the original or the 
ESP32-optimized variant in menuconfig.

Benchmarks
==========

Tamp can deliver decent compression with minimal memory and CPU requirements.

Compression ratio
-----------------

Input: 5614 bytes of text

Compressed output in bytes vs. window size ("window bits (bytes)"), i.e. RAM requirement

+-------------+--------------+----------+----------+----------+----------+----------+
|             | Uncompressed | 8 (256b) | 9 (512b) | 10 (1kb) | 11 (2kb) | 12 (4kb) |
+=============+==============+==========+==========+==========+==========+==========+
| Bytes       | 5614         | 3195     | 2800     | 2649     | 2581     | 2571     |
+-------------+--------------+----------+----------+----------+----------+----------+
| %           | 100,0 %      | 56,9 %   | 49,9 %   | 47,2 %   | 46,0 %   | 45,8 %   |
+-------------+--------------+----------+----------+----------+----------+----------+
| Compression | 1,0 x        | 1,8 x    | 2,0 x    | 2,1 x    | 2,2 x    | 2,2 x    |
+-------------+--------------+----------+----------+----------+----------+----------+

Input: Espressif logo in 24-bit BMP format, 25862 bytes

Compressed output in bytes vs. window size ("window bits (bytes)")

+-------------+--------------+----------+----------+----------+----------+----------+----------+
|             | Uncompressed | 8 (256b) | 9 (512b) | 10 (1kb) | 11 (2kb) | 12 (4kb) | 13 (8kb) |
+=============+==============+==========+==========+==========+==========+==========+==========+
| Bytes       | 25862        | 7720     | 7772     | 6538     | 6517     | 6601     | 6590     |
+-------------+--------------+----------+----------+----------+----------+----------+----------+
| %           | 100,0 %      | 29,9 %   | 30,1 %   | 25,3 %   | 25,2 %   | 25,5 %   | 25,5 %   |
+-------------+--------------+----------+----------+----------+----------+----------+----------+
| Compression | 1,0 x        | 3,4 x    | 3,3 x    | 4,0 x    | 4,0 x    | 3,9 x    | 3,9 x    |
+-------------+--------------+----------+----------+----------+----------+----------+----------+

As to the overall compression efficiency, 7zip in "Ultra" mode compresses the same file down to 3022 bytes (11,7%),
which should be a rough approximation of the entropy of that file, i.e. the theoretical limit of what is possible.

Speed
-----

Performance on an **ESP32-S3** (Xtensa) @ **240 MHz**:

Input: Espressif logo in 24-bit BMP format, 25862 bytes

**Original C** version:

+------------+------------+------------+-----------+-----------+-----------+-----------+
|            | 8          | 9          | 10        | 11        | 12        | 13        |
+============+============+============+===========+===========+===========+===========+
| CPU time   | 103,5 ms   | 182,1 ms   | 252,7 ms  | 420,7 ms  | 782,1 ms  | 1419,1 ms |
+------------+------------+------------+-----------+-----------+-----------+-----------+
| Throughput | 244,0 kb/s | 138,7 kb/s | 99,9 kb/s | 60,0 kb/s | 32,3 kb/s | 17,8 kb/s |
+------------+------------+------------+-----------+-----------+-----------+-----------+

**Optimized C/C++** variant:

+-------------+------------+------------+------------+------------+------------+-----------+
|             | 8          | 9          | 10         | 11         | 12         | 13        |
+=============+============+============+============+============+============+===========+
| CPU time    | 35,4 ms    | 56,6 ms    | 73,3 ms    | 122,3 ms   | 221,9 ms   | 395,9 ms  |
+-------------+------------+------------+------------+------------+------------+-----------+
| Throughput  | 713,1 kb/s | 446,6 kb/s | 344,6 kb/s | 206,6 kb/s | 113,8 kb/s | 63,8 kb/s |
+-------------+------------+------------+------------+------------+------------+-----------+
| Speed (rel) | 2,9 x      | 3,2 x      | 3,4 x      | 3,4 x      | 3,5 x      | 3,6 x     |
+-------------+------------+------------+------------+------------+------------+-----------+

**ESP32-S3 SIMD/PIE** variant:

+-------------+-------------+-------------+-------------+-------------+------------+------------+
|             | 8           | 9           | 10          | 11          | 12         | 13         |
+=============+=============+=============+=============+=============+============+============+
| CPU time    | 13,0 ms     | 15,4 ms     | 16,9 ms     | 23,6 ms     | 35,9 ms    | 59,1 ms    |
+-------------+-------------+-------------+-------------+-------------+------------+------------+
| Throughput  | 1948,2 kb/s | 1636,2 kb/s | 1493,0 kb/s | 1070,4 kb/s | 703,9 kb/s | 427,1 kb/s |
+-------------+-------------+-------------+-------------+-------------+------------+------------+
| Speed (rel) | 8,0 x       | 11,8 x      | 14,9 x      | 17,8 x      | 21,8 x     | 24,0 x     |
+-------------+-------------+-------------+-------------+-------------+------------+------------+

An **ESP32-C3** (RISC-V, max. 160 MHz) consistently runs the original variant about 5% faster and the
optimized C/C++ variant about 10% faster *per CPU clock cycle* than the ESP32-S3; so the C3 is, for
this workload, a little more efficient than the S3.

Tamp's original Readme below.

****

.. image:: https://raw.githubusercontent.com/BrianPugh/tamp/main/assets/logo_300w.png
   :alt: tamp logo
   :width: 300
   :align: center

-----------------------------------------------------------------

|Python compat| |PyPi| |GHA tests| |Codecov report| |readthedocs|

.. inclusion-marker-do-not-remove

Tamp
====

Tamp is a low-memory, DEFLATE-inspired lossless compression library.

Features
========

* Various implementations available:

  * Pure Python:

    * ``tamp/compressor.py``, ``tamp/decompressor.py``

    * When available, Tamp will use a python-bound C implementation for speed.

  * C library:

    * ``tamp/_c_src/``

* High compression ratios and low memory use.

* Compact compression and decompression implementations.

  * Compiled C library is <4KB (compressor + decompressor).

* Mid-stream flushing.

  * Allows for submission of messages while continuing to compress subsequent data.

* Customizable dictionary for greater compression of small messages.

* Convenient CLI interface.

Installation
============
Tamp contains 3 implementations:

1. A desktop Cpython implementation that is optimized for readability

2. A micropython viper implementation that is optimized for runtime performance.

3. A C implementation (with python bindings) for accelerated desktop use and to be used in C projects.

Desktop Python
^^^^^^^^^^^^^^
The Tamp library and CLI requires Python ``>=3.8`` and can be installed via:

.. code-block:: bash

   pip install tamp

MicroPython
^^^^^^^^^^^
For micropython use, there are 3 main files:

1. ``tamp/__init__.py`` - Always required.

2. ``tamp/decompressor_viper.py`` - Required for on-device decompression.

3. ``tamp/compressor_viper.py`` - Required for on-device compression.

For example, if on-device decompression isn't used, then do not include ``decompressor_viper.py``.
If manually installing, just copy these files to your microcontroller's ``/lib/tamp`` folder.

If using `mip`_, tamp can be installed by specifying the appropriate ``package-*.json`` file.

.. code-block:: bash

   mip install github:brianpugh/tamp  # Defaults to package.json: Compressor & Decompressor
   mip install github:brianpugh/tamp/package-compressor.json  # Compressor only
   mip install github:brianpugh/tamp/package-decompressor.json  # Decompressor only

If using `Belay`_, tamp can be installed by adding the following to ``pyproject.toml``.

.. code-block:: toml

   [tool.belay.dependencies]
   tamp = [
      "https://github.com/BrianPugh/tamp/blob/main/tamp/__init__.py",
      "https://github.com/BrianPugh/tamp/blob/main/tamp/compressor_viper.py",
      "https://github.com/BrianPugh/tamp/blob/main/tamp/decompressor_viper.py",
   ]

C
^

Copy the ``tamp/_c_src/tamp`` folder into your project.
For more information, see `the documentation`_.

Usage
=====
Tamp works on desktop python and micropython. On desktop, Tamp is bundled with the
``tamp`` command line tool for compressing and decompressing tamp files.

CLI
^^^

Compression
-----------
Use ``tamp compress`` to compress a file or stream.
If no input file is specified, data from stdin will be read.
If no output is specified, the compressed output stream will be written to stdout.

.. code-block:: bash

   $ tamp compress --help

    Usage: tamp compress [OPTIONS] [INPUT_PATH]

    Compress an input file or stream.

   ╭─ Arguments ────────────────────────────────────────────────────────────────────────╮
   │   input_path      [INPUT_PATH]  Input file to compress or decompress. Defaults to  │
   │                                 stdin.                                             │
   ╰────────────────────────────────────────────────────────────────────────────────────╯
   ╭─ Options ──────────────────────────────────────────────────────────────────────────╮
   │ --output   -o      PATH                      Output file. Defaults to stdout.      │
   │ --window   -w      INTEGER RANGE [8<=x<=15]  Number of bits used to represent the  │
   │                                              dictionary window.                    │
   │                                              [default: 10]                         │
   │ --literal  -l      INTEGER RANGE [5<=x<=8]   Number of bits used to represent a    │
   │                                              literal.                              │
   │                                              [default: 8]                          │
   │ --help                                       Show this message and exit.           │
   ╰────────────────────────────────────────────────────────────────────────────────────╯


Example usage:

.. code-block:: bash

   tamp compress enwik8 -o enwik8.tamp  # Compress a file
   echo "hello world" | tamp compress | wc -c  # Compress a stream and print the compressed size.

The following options can impact compression ratios and memory usage:

* ``window`` -  ``2^window`` plaintext bytes to look back to try and find a pattern.
  A larger window size will increase the chance of finding a longer pattern match,
  but will use more memory, increase compression time, and cause each pattern-token
  to take up more space. Try smaller window values if compressing highly repetitive
  data, or short messages.

* ``literal`` - Number of bits used in each plaintext byte. For example, if all input
  data is 7-bit ASCII, then setting this to 7 will improve literal compression
  ratios by 11.1%. The default, 8-bits, can encode any binary data.

Decompression
-------------
Use ``tamp decompress`` to decompress a file or stream.
If no input file is specified, data from stdin will be read.
If no output is specified, the compressed output stream will be written to stdout.

.. code-block:: bash

  $ tamp decompress --help

  Usage: tamp decompress [OPTIONS] [INPUT_PATH]

  Decompress an input file or stream.

 ╭─ Arguments ────────────────────────────────────────────────────────────────────────╮
 │   input_path      [INPUT_PATH]  Input file. If not provided, reads from stdin.     │
 ╰────────────────────────────────────────────────────────────────────────────────────╯
 ╭─ Options ──────────────────────────────────────────────────────────────────────────╮
 │ --output  -o      PATH  Output file. Defaults to stdout.                           │
 │ --help                  Show this message and exit.                                │
 ╰────────────────────────────────────────────────────────────────────────────────────╯

Example usage:

.. code-block:: bash

   tamp decompress enwik8.tamp -o enwik8
   echo "hello world" | tamp compress | tamp decompress

Python
^^^^^^
The python library can perform one-shot compression, as well as operate on files/streams.

.. code-block:: python

   import tamp

   # One-shot compression
   string = b"I scream, you scream, we all scream for ice cream."
   compressed_data = tamp.compress(string)
   reconstructed = tamp.decompress(compressed_data)
   assert reconstructed == string

   # Streaming compression
   with tamp.open("output.tamp", "wb") as f:
       for _ in range(10):
           f.write(string)

   # Streaming decompression
   with tamp.open("output.tamp", "rb") as f:
       reconstructed = f.read()


Benchmark
=========
In the following section, we compare Tamp against:

* zlib_, a python builtin gzip-compatible DEFLATE compression library.

* heatshrink_, a data compression library for embedded/real-time systems.
  Heatshrink has similar goals as Tamp.

All of these are LZ-based compression algorithms, and tests were performed using a 1KB (10 bit) window.
Since zlib already uses significantly more memory by default, the lowest memory level (``memLevel=1``) was used in
these benchmarks. It should be noted that higher zlib memory levels will having greater compression ratios than Tamp.
Currently, there is no micropython-compatible zlib or heatshrink compression implementation, so these numbers are
provided simply as a reference.

Compression Ratio
^^^^^^^^^^^^^^^^^
The following table shows compression algorithm performance over a variety of input data sourced from the `Silesia Corpus`_
and Enwik8_. This should give a general idea of how these algorithms perform over a variety of input data types.

+-----------------------+-------------+----------------+----------------+------------+
| dataset               | raw         | tamp           | zlib           | heatshrink |
+=======================+=============+================+================+============+
| enwik8                | 100,000,000 | **51,635,633** | 56,205,166     | 56,110,394 |
+-----------------------+-------------+----------------+----------------+------------+
| build/silesia/dickens | 10,192,446  | **5,546,761**  | 6,049,169      | 6,155,768  |
+-----------------------+-------------+----------------+----------------+------------+
| build/silesia/mozilla | 51,220,480  | 25,121,385     | **25,104,966** | 25,435,908 |
+-----------------------+-------------+----------------+----------------+------------+
| build/silesia/mr      | 9,970,564   | 5,027,032      | **4,864,734**  | 5,442,180  |
+-----------------------+-------------+----------------+----------------+------------+
| build/silesia/nci     | 33,553,445  | 8,643,610      | **5,765,521**  | 8,247,487  |
+-----------------------+-------------+----------------+----------------+------------+
| build/silesia/ooffice | 6,152,192   | **3,814,938**  | 4,077,277      | 3,994,589  |
+-----------------------+-------------+----------------+----------------+------------+
| build/silesia/osdb    | 10,085,684  | **8,520,835**  | 8,625,159      | 8,747,527  |
+-----------------------+-------------+----------------+----------------+------------+
| build/silesia/reymont | 6,627,202   | **2,847,981**  | 2,897,661      | 2,910,251  |
+-----------------------+-------------+----------------+----------------+------------+
| build/silesia/samba   | 21,606,400  | 9,102,594      | **8,862,423**  | 9,223,827  |
+-----------------------+-------------+----------------+----------------+------------+
| build/silesia/sao     | 7,251,944   | **6,137,755**  | 6,506,417      | 6,400,926  |
+-----------------------+-------------+----------------+----------------+------------+
| build/silesia/webster | 41,458,703  | **18,694,172** | 20,212,235     | 19,942,817 |
+-----------------------+-------------+----------------+----------------+------------+
| build/silesia/x-ray   | 8,474,240   | 7,510,606      | **7,351,750**  | 8,059,723  |
+-----------------------+-------------+----------------+----------------+------------+
| build/silesia/xml     | 5,345,280   | 1,681,687      | **1,586,985**  | 1,665,179  |
+-----------------------+-------------+----------------+----------------+------------+

Tamp usually out-performs heatshrink, and is generally very competitive with zlib.
While trying to be an apples-to-apples comparison, zlib still uses significantly more
memory during both compression and decompression (see next section). Tamp accomplishes
competitive performance while using around 10x less memory.

Memory Usage
^^^^^^^^^^^^
The following table shows approximately how much memory each algorithm uses during compression and decompression.

+---------------+-------------------+------------------------------+-------------------+
| Action        | tamp              | zlib                         | heatshrink        |
+===============+===================+==============================+===================+
| Compression   | (1 << windowBits) | (1 << (windowBits+2)) + 7 KB | (1 << windowBits) |
+---------------+-------------------+------------------------------+-------------------+
| Decompression | (1 << windowBits) | (1 << windowBits) + 7 KB     | (1 << windowBits) |
+---------------+-------------------+------------------------------+-------------------+

Both tamp and heatshrink have a few dozen bytes of overhead in addition to the primary window buffer, but are implementation-specific and ignored for clarity here.

Runtime
^^^^^^^
As a rough benchmark, here is the performance (in seconds) of these different compression algorithms on the 100MB enwik8 dataset.
These tests were performed on an M1 Macbook Air.

+---------------+--------------------+-------+------+--------------+-----------------+
| Action        | tamp               | tamp  | zlib | heatshrink   | heatshrink      |
|               | (Python Reference) | (C)   |      | (with index) | (without index) |
+===============+====================+=======+======+==============+=================+
| Compression   | 109.5              | 16.45 | 4.84 | 6.22         | 41.729          |
+---------------+--------------------+-------+------+--------------+-----------------+
| Decompression | 54.0               | 0.70  | 0.98 | 0.82         | 0.82            |
+---------------+--------------------+-------+------+--------------+-----------------+

Heatshrink v0.4.1 was used in these benchmarks.
When heathshrink uses an index, an additional ``(1 << (windowBits + 1))`` bytes of memory are used, tripling the memory requirement.
Tamp could use a similar indexing to increase compression speed, but has chosen not to to focus on the primary goal of a low-memory compressor.

To give an idea of Tamp's speed on an embedded device, the following table shows compression/decompression in **bytes/second of the first 100KB of enwik8 on a pi pico (rp2040)** at the default 125MHz clock rate.
This isn't exactly an apples-to-apples comparison because the C benchmark does not use a filesystem (and thusly, reduced overhead) nor dynamic memory allocation, but is good enough to get the idea across.

+---------------+---------------------+------------+
| Action        | tamp                | tamp       |
|               | (Micropython Viper) | (C)        |
+===============+=====================+============+
| Compression   | ~4,300              | ~28,500    |
+---------------+---------------------+------------+
| Decompression | ~42,000             | ~1,042,524 |
+---------------+---------------------+------------+

Binary Size
^^^^^^^^^^^
To give an idea on the resulting binary sizes, Tamp and other libraries were compiled for the Pi Pico (``armv6m``).
All libraries were compiled with ``-O3``.
Numbers reported in bytes.

+--------------------+------------+--------------+---------------------------+
| Library            | Compressor | Decompressor | Compressor + Decompressor |
+====================+============+==============+===========================+
| Tamp (micropython) | 4429       | 4205         | 7554                      |
+--------------------+------------+--------------+---------------------------+
| Tamp (C)           | 2008       | 1972         | 3864                      |
+--------------------+------------+--------------+---------------------------+
| Heatshrink         | 2956       | 3876         | 6832                      |
+--------------------+------------+--------------+---------------------------+
| uzlib              | 2355       | 3963         | 6318                      |
+--------------------+------------+--------------+---------------------------+

Heatshrink doesn't include a high level API; in an apples-to-apples comparison the Tamp library would be even smaller.


.. |GHA tests| image:: https://github.com/BrianPugh/tamp/workflows/tests/badge.svg
   :target: https://github.com/BrianPugh/tamp/actions?query=workflow%3Atests
   :alt: GHA Status
.. |Codecov report| image:: https://codecov.io/github/BrianPugh/tamp/coverage.svg?branch=main
   :target: https://codecov.io/github/BrianPugh/tamp?branch=main
   :alt: Coverage
.. |readthedocs| image:: https://readthedocs.org/projects/tamp/badge/?version=latest
        :target: https://tamp.readthedocs.io/en/latest/?badge=latest
        :alt: Documentation Status
.. |Python compat| image:: https://img.shields.io/badge/>=python-3.8-blue.svg
.. |PyPi| image:: https://img.shields.io/pypi/v/tamp.svg
        :target: https://pypi.python.org/pypi/tamp
.. _Belay: https://github.com/BrianPugh/belay
.. _zlib:  https://docs.python.org/3/library/zlib.html
.. _heatshrink: https://github.com/atomicobject/heatshrink
.. _Silesia Corpus: https://sun.aei.polsl.pl//~sdeor/index.php?page=silesia
.. _Enwik8: https://mattmahoney.net/dc/textdata.html
.. _mip: https://docs.micropython.org/en/latest/reference/packages.html#installing-packages-with-mip
.. _the documentation: https://tamp.readthedocs.io/en/latest/c_library.html
