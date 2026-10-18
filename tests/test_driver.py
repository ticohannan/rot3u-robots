"""DD-DRV requirements, exercised through a small test sketch on the emulated PCA9685."""
import tempfile
from pathlib import Path
import host_build as hb

EXE = None


def exe():
    global EXE
    if EXE is None:
        EXE = hb.build("tests/host/driver_sketch")
    return EXE


def test_cold_boot_outputs_off():                      # DRV-1
    out, log = hb.run(exe(), [])
    assert "BEGIN 1 ADOPTED 0" in out
    w = log.writes()
    assert len(w) == 32 and all(off == 4096 for _, off in w), w


def test_write_clamps():                               # DRV-3
    out, _ = hb.run(exe(), ["W 3 3000", "W 4 100", "W 5 1500"])
    reads = [int(l.split()[1]) for l in out.splitlines() if l.startswith("READ")]
    assert abs(reads[0] - 2500) <= 5 and abs(reads[1] - 500) <= 5 and abs(reads[2] - 1500) <= 5, reads


def test_release_full_off():                           # DRV-4
    out, log = hb.run(exe(), ["W 2 1600", "R 2"])
    assert "LIVE? 0" in out
    assert log.writes()[-1] == (2, 4096)


def test_adopt_live_channels():                        # DRV-2: second run = reset with the board still powered
    st = Path(tempfile.mkdtemp()) / "state.bin"
    hb.run(exe(), ["W 1 1400", "W 7 2100"], state=st)
    out, log = hb.run(exe(), [], state=st)
    assert "BEGIN 1 ADOPTED 1" in out
    live = {int(l.split()[1]): int(l.split()[2]) for l in out.splitlines() if l.startswith("LIVE ")}
    assert set(live) == {1, 7} and abs(live[1] - 1400) <= 5 and abs(live[7] - 2100) <= 5, live
    assert log.writes() == []                          # adoption writes nothing


def test_reconfigure_all_off_first():                  # DD-DRV section 3: ALL_LED_OFF before wake
    st = Path(tempfile.mkdtemp()) / "state.bin"
    hb.run(exe(), ["W 3 1600"], state=st)
    # make the chip look cold (asleep) but with an old pulse still in channel 3's registers
    b = bytearray(st.read_bytes()); b[0] = 0x11; st.write_bytes(bytes(b))
    out, log = hb.run(exe(), [], state=st)
    assert "ADOPTED 0" in out
    alls = [i for i, (t, r) in enumerate(log.events) if t == "W" and r.endswith("ALL")]
    singles = [i for i, (t, r) in enumerate(log.events) if t == "W" and not r.endswith("ALL")]
    assert len(alls) == 16 and max(alls) < min(singles), (alls[:3], singles[:3])
