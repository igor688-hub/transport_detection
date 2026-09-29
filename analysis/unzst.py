import pathlib
import sys
import tarfile

import zstandard


def unpack(src, out):
    out = pathlib.Path(out)
    out.mkdir(parents=True, exist_ok=True)
    with open(src, 'rb') as f:
        head = zstandard.ZstdDecompressor().stream_reader(f).read(512)
    with open(src, 'rb') as f:
        reader = zstandard.ZstdDecompressor(max_window_size=2 ** 31).stream_reader(f)
        if head[257:262] == b'ustar':
            with tarfile.open(fileobj=reader, mode='r|') as t:
                for m in t:
                    print(m.name, flush=True)
                    t.extract(m, out, filter='data')
        else:
            with open(out / pathlib.Path(src).stem, 'wb') as o:
                while chunk := reader.read(1 << 24):
                    o.write(chunk)


if __name__ == '__main__':
    unpack(sys.argv[1], sys.argv[2])
