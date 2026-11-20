# AI-generated (Claude, Anthropic).
# Prompt:
#   pytest for tap_speed.py, no ROS, no arm. A tap with one send_angles at byte 30, the Atom's
#   sync-write recorded on 13 Nov 2026 and a get_angles reply: the send is decoded (byte,
#   targets), the sync-write is attributed to it with the goal speed of each servo, the pose
#   before it is found, a corrupted checksum is skipped, a cut last row is ignored, and --csv
#   writes one row per servo.
"""Tests of tap_speed.py."""

import csv

import tap_speed

SEND = 'fefe0f22' + '07d0' + 'fff6' + '0000' * 4 + '1efa'  # J1 20.00 deg, J2 -0.10, byte 30
SYNC = ('fffffe3483290701' '32e8070000f101' '0232010800003201' '03324a0800002c01'
        '04323f0800002c01' '0532130800009e01' '0632de080000fe01' '2a')
REPLY = 'fefe0e20' + '07d0' + 'fff6' + '0000' * 4 + 'fa'


def write_tap(path, rows):
    """Write a proxy_tap.py csv."""
    with open(path, 'w', encoding='utf-8') as f:
        for seconds, side, hexbytes in rows:
            f.write(f'{seconds:.6f},{side},{len(hexbytes) // 2},{hexbytes}\n')


def test_frames_are_what_the_tap_holds():
    assert len(bytes.fromhex(SEND)) == 18
    assert len(bytes.fromhex(SYNC)) == 56
    assert len(bytes.fromhex(REPLY)) == 17


def test_a_send_its_sync_write_and_the_pose_before(tmp_path):
    tap = tmp_path / 'tap.csv'
    write_tap(tap, [(0.000, 'DRV', 'fefe0220fa'), (0.008, 'ATOM', REPLY),
                    (0.010, 'DRV', SEND), (0.030, 'ATOM', SYNC)])
    rows, seen = tap_speed.per_send(str(tap))
    assert len(rows) == 1 and len(seen) == 1
    row = rows[0]
    assert row['byte'] == 30
    assert row['targets'][:2] == [20.0, -0.1]
    assert row['pose'][:2] == [20.0, -0.1]
    assert len(row['writes']) == 1
    servos = row['writes'][0][1]
    assert [s[3] for s in servos] == [497, 306, 300, 300, 414, 510]
    assert {s[0] for s in servos} == {50}
    assert abs(servos[0][1] - (2024 - 2048) * 360 / 4096) < 1e-9


def test_a_bad_checksum_and_a_cut_row_are_skipped(tmp_path):
    tap = tmp_path / 'tap.csv'
    bad = SYNC[:-2] + '2b'
    write_tap(tap, [(0.010, 'DRV', SEND), (0.030, 'ATOM', bad)])
    with open(tap, 'a', encoding='utf-8') as f:
        f.write('0.040,ATOM,5')
    rows, _ = tap_speed.per_send(str(tap))
    assert rows[0]['writes'] == []


def test_csv_has_a_row_per_servo(tmp_path, capsys):
    tap = tmp_path / 'tap.csv'
    out = tmp_path / 'rows.csv'
    write_tap(tap, [(0.010, 'DRV', SEND), (0.030, 'ATOM', SYNC)])
    assert tap_speed.main([str(tap), '--csv', str(out)]) == 0
    with open(out, newline='', encoding='utf-8') as f:
        rows = list(csv.reader(f))
    assert len(rows) == 7
    assert rows[1][2] == '30' and rows[1][-1] == '497'
    assert 'mode 1' in capsys.readouterr().out
