cimport ctamp
from cpython.mem cimport PyMem_Malloc, PyMem_Free
from libc.stddef cimport size_t
from io import BytesIO
from . import ExcessBitsError
try:
    from typing import Union
except ImportError:
    pass


CHUNK_SIZE = (1 << 20)


cdef class Compressor:
    cdef ctamp.TampCompressor* _c_compressor
    cdef ctamp.TampConf* _c_conf
    cdef bytearray _window_buffer
    cdef object f

    def __cinit__(self):
        self._c_conf = <ctamp.TampConf*>PyMem_Malloc(sizeof(ctamp.TampConf))
        if self._c_conf is NULL:
            raise MemoryError

        self._c_compressor = <ctamp.TampCompressor*>PyMem_Malloc(sizeof(ctamp.TampCompressor))
        if self._c_compressor is NULL:
            PyMem_Free(self._c_conf)
            raise MemoryError


    def __dealloc__(self):
        PyMem_Free(self._c_conf)
        PyMem_Free(self._c_compressor)

    def __init__(
        self,
        f,
        *,
        window=10,
        literal=8,
        dictionary=None,
    ):
        if not hasattr(f, "write"):  # It's probably a path-like object.
            f = open(str(f), "wb")

        self.f = f

        self._c_conf.window = window
        self._c_conf.literal = literal
        self._c_conf.use_custom_dictionary = bool(dictionary)

        self._window_buffer = bytearray(1 << window)

        res = ctamp.tamp_compressor_init(self._c_compressor, self._c_conf, <char *>self._window_buffer)

    def write(self, data: bytes) -> int:
        cdef bytearray output_buffer = bytearray(CHUNK_SIZE)
        cdef size_t input_consumed_size = 0
        cdef size_t output_buffer_written_size = 1
        cdef ctamp.tamp_res res
        cdef int written_to_disk_size = 0

        while output_buffer_written_size:
            data = data[input_consumed_size:]
            res = ctamp.tamp_compressor_compress(
                self._c_compressor,
                <char *>output_buffer,
                CHUNK_SIZE,
                &output_buffer_written_size,
                data,
                len(data),
                &input_consumed_size
            )
            if(res < 0):
                if res == ctamp.tamp_res.TAMP_EXCESS_BITS:
                    raise ExcessBitsError
                else:
                    raise NotImplementedError
            self.f.write(output_buffer[:output_buffer_written_size])
            written_to_disk_size += output_buffer_written_size

        return written_to_disk_size

    cpdef int flush(self, write_token: bool = True):
        cdef ctamp.tamp_res res
        cdef bytearray buffer = bytearray(20)
        cdef size_t output_written_size = 0

        res = ctamp.tamp_compressor_flush(
            self._c_compressor,
            <char *> buffer,
            len(buffer),
            &output_written_size,
            write_token,
        )

        if(res < 0):
            raise NotImplementedError

        if output_written_size:
            self.f.write(buffer[:output_written_size])

        return output_written_size

    def close(self) -> int:
        bytes_written = self.flush(write_token=False)
        self.f.close()
        return bytes_written

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


class TextCompressor(Compressor):
    """Compresses text to a file or stream."""

    def write(self, data: str) -> int:
        return super().write(data.encode())


def compress(data: Union[bytes, str], *args, **kwargs) -> bytes:
    """Single-call to compress data.

    Parameters
    ----------
    data: Union[str, bytes]
        Data to compress.
    *args: tuple
        Passed along to :class:`Compressor`.
    **kwargs : dict
        Passed along to :class:`Compressor`.

    Returns
    -------
    bytes
        Compressed data
    """
    with BytesIO() as f:
        if isinstance(data, str):
            c = TextCompressor(f, *args, **kwargs)
            c.write(data)
        else:
            c = Compressor(f, *args, **kwargs)
            c.write(data)
        c.flush(write_token=False)
        f.seek(0)
        return f.read()
