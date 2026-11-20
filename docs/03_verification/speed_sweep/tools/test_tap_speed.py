# AI-generated (Claude, Anthropic).
# Prompt:
#   pytest for tap_speed.py, no ROS, no arm. A tap with one send_angles at byte 30, the Atom's
#   sync-write recorded on 13 Nov 2026 and a get_angles reply: the send is decoded (byte,
#   targets), the sync-write is attributed to it with the goal speed of each servo, the pose
#   before it is found, a corrupted checksum is skipped, a cut last row is ignored, and --csv
#   writes one row per servo.
# Follow-up prompt:
#   A stop at 15 deg/s on J1 followed by poses that move 0.5 deg more and then hold: the speed
#   before, the travel and the time to rest are reported; a stop with no rest in the tap is
#   left out.
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


def pose_reply(j1_centideg):
    """Return a get_angles reply with J1 at `j1_centideg` and the others at 0."""
    return 'fefe0e20' + f'{j1_centideg & 0xFFFF:04x}' + '0000' * 5 + 'fa'


def test_a_stop_reports_speed_travel_and_rest(tmp_path):
    tap = tmp_path / 'tap.csv'
    rows = []
    t = 0.0
    j1 = 0
    for _ in range(10):  # 15 deg/s: 0.5 deg every 33 ms
        rows.append((t, 'ATOM', pose_reply(j1)))
        t += 1 / 30
        j1 += 50
    rows.append((t, 'DRV', 'fefe0229fa'))
    for step in (25, 15, 10):  # 0.5 deg more, then still
        t += 0.01
        j1 += step
        rows.append((t, 'ATOM', pose_reply(j1)))
    for _ in range(8):
        t += 0.01
        rows.append((t, 'ATOM', pose_reply(j1)))
    write_tap(tap, rows)
    found = tap_speed.stops(str(tap))
    assert len(found) == 1
    assert abs(found[0]['speed'] - 15.0) < 0.5
    # From the last pose before the stop (4.50 deg, 33 ms before it) to the rest (5.50): the
    # travel counts from the newest pose the driver had, so it includes up to one poll of
    # motion before the stop. 0.5 deg of that here.
    assert abs(found[0]['travel'][0] - 1.0) < 1e-6
    assert 25 <= found[0]['rest_ms'] <= 35


def test_a_stop_without_rest_is_left_out(tmp_path):
    tap = tmp_path / 'tap.csv'
    rows = [(i / 30, 'ATOM', pose_reply(50 * i)) for i in range(10)]
    rows.append((0.34, 'DRV', 'fefe0229fa'))
    rows.append((0.35, 'ATOM', pose_reply(600)))
    write_tap(tap, rows)
    assert tap_speed.stops(str(tap)) == []
