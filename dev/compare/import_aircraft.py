"""Bench aircraft for every player plane, from the game's own flight parameters.

    python dev/compare/import_aircraft.py <spec-probe-*.json> [--check]

The spec probe (F7 in game, spec_probe.lua) reads each aircraft's DT_PlayerParameter table:
level 1 (what the campaign flies; levels are the online mode's) of AceFlightEnginePlayerParameter.
This turns it into the bench's by-speed tables and writes dev/compare/aircraft/aircraft.txt,
which harness.cpp loads as one condition per aircraft, sorted by the hangar's Speed + Mobility.
The file is derived from game data and stays out of the repository (.gitignore).

Mapping, checked against the aircraft flown and fitted (fit_speed_tables.py, 2026-10-05):
  - SpeedRot<i> at SpeedGraph<i> (km/h): Y pitch, X roll, deg/s. Full stick measured 0.41 x
    Y and 0.41 x X across speeds on the FA-36 and Su-57 (--check prints the others).
  - High-G: written (2.2) but unused since 2026-10-06; the bench's high-G rate is one curve for
    every aircraft (harness.cpp highg_rate), as measured.
  - Thrust: full throttle dV/dt = 0.053 x AcceleR x (SpeedMax - V), at most 75 m/s^2. The
    FA-36, Su-35, Su-57 and ADF-X02 logged 0.11-0.16 /s x (SpeedMax - V) at 200-700 m/s with
    AcceleR 2.45-2.5; low down the logged dV/dt tops out near 70. (Scaling the FA-36's curve
    instead left a positive tail past SpeedMax: an A-6E, 1770 km/h, reached 3000 on the bench.)
  - Brake: the FA-36's logged curve scaled by DeceleR (2.4).
  - Stall: SpeedStall (km/h).
"""
import argparse, json, os

RATE = 0.41          # measured full-stick rate / SpeedRot
HIGH_G = 2.2         # high-G full pull / full pull
THRUST_RATE, THRUST_CAP = 0.053, 75.0   # full throttle: THRUST_RATE x AcceleR x (SpeedMax - V), m/s^2
PULL_BASE, ROLL_BASE = 50.0, 105.0   # the bench plant's pitch_pull and roll_max the factors scale
# FA-36, logged 2026-10-05 (m/s^2 at band centres 50..850 m/s)
FA36_THRUST = [68, 68, 73, 59, 50, 38, 18, 4.5, 5.6]
FA36_BRAKE = [-25, -25, -59, -85, -122, -132, -126, -112, -83]
FA36_MAX, FA36_ACCEL, FA36_DECEL = 2850.0, 2.5, 2.4
NAMES = {'a10c': 'A-10C', 'f02a': 'F-2A', 'f04e': 'F-4E', 'f15c': 'F-15C', 'f22a': 'F-22A', 'f35c': 'F-35C',
         'f18c': 'F/A-18C', 'j39e': 'Gripen E', 'su34': 'Su-34', 'e18g': 'EA-18G', 'typn': 'Typhoon', 'f15j': 'F-15J',
         'a06e': 'A-6E', 'rflm': 'Rafale M', 'su25': 'Su-25', 'x40a': 'X-40A', 'mr2k': 'Mirage 2000-5', 'fa36': 'FA-36',
         'ea36': 'EA-36', 'm29a': 'MiG-29A', 'su57': 'Su-57', 'f16c': 'F-16C', 'f14d': 'F-14D', 'f18f': 'F/A-18F',
         'f15e': 'F-15E', 'su47': 'Su-47', 'm21b': 'MiG-21bis', 'su35': 'Su-35', 'm31b': 'MiG-31B', 'su33': 'Su-33',
         'e06b': 'EA-6B', 'f18e': 'F/A-18E', 'adfx02': 'ADF-X02', 'f14a': 'F-14A', 'av8b': 'AV-8B'}
# Fitted from flights (harness.cpp), for --check: pull and roll p90 deg/s by band
MEASURED = {
    'fa36': ([51.1, 51.1, 51.2, 51.2, 49.2, 42.7, 35.9, 28.3, 22.8], [105.8, 105.8, 113.2, 115.8, 115.9, 105.5, 98.5, 76.8, 79.3]),
    'su57': ([46, 50, 52, 53, 52, 48, 44.5, 33, 11], [57, 110, 109, 116, 110, 98, 92, 79, 48]),
    'su35': ([47.5, 47.5, 50, 40, 41, 32.5, 33, 34, 21], [76, 76, 97, 97, 105, 99, 89, 71, 77]),
    'adfx02': ([55, 56, 56.5, 39, 38.5, 35, 23.5, 23.5, 17], [100, 100, 98, 103, 87, 86, 71, 68, 60]),
}


def interp(xs, ys, x):
    pts = sorted(zip(xs, ys))
    if x <= pts[0][0]: return pts[0][1]
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        if x0 <= x <= x1: return y0 + (y1 - y0) * (x - x0) / (x1 - x0) if x1 > x0 else y1
    return pts[-1][1]


def aircraft(rows):
    p = 'LevelParameter.1.FlightEnginePlayerParameter.'
    get = lambda k, d=0.0: rows.get(p + k) if rows.get(p + k) is not None else d
    xs = [get(f'SpeedGraph{i}') / 3.6 for i in range(10)]
    pitch = [get(f'SpeedRot{i}.Y') for i in range(10)]
    roll = [get(f'SpeedRot{i}.X') for i in range(10)]
    centres = [k * 100 + 50 for k in range(9)]
    speed_max, accel, decel = get('SpeedMax', FA36_MAX), get('AcceleR', FA36_ACCEL), get('DeceleR', FA36_DECEL)
    thrust = [min(THRUST_CAP, THRUST_RATE * accel * (speed_max / 3.6 - v)) for v in centres]
    return {
        'pull': [RATE * interp(xs, pitch, v) for v in centres],
        'roll': [RATE * interp(xs, roll, v) for v in centres],
        'thrust': thrust,
        'brake': [b * decel / FA36_DECEL for b in FA36_BRAKE],
        'stall': get('SpeedStall', 225) / 3.6,
        'speed_max': speed_max,
        'bars': [rows.get('LevelParameter.1.' + k) or 0 for k in ('GraphSpeed', 'GraphMobility', 'GraphStability')],
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('probe')
    ap.add_argument('--check', action='store_true')
    args = ap.parse_args()
    data = json.load(open(args.probe, encoding='utf-8'))
    planes = []
    for table in data['tables']:
        if 'DT_PlayerParameter_' not in table['name']: continue
        code = table['name'].split('DT_PlayerParameter_')[-1].split('.')[0]          # PP0018_fa36
        key = code.split('_', 1)[1] if '_' in code else code                          # fa36, a06e_ms15, adfx02_L001
        if key.endswith('_ms15'): continue                                            # a mission's variant
        key = key.split('_L')[0]
        rows = next(iter(table['rows'].values()), {})
        if rows.get('LevelParameter.1.GraphSpeed') is None: continue
        a = aircraft(rows)
        a['id'], a['title'] = key, NAMES.get(key, key.upper())
        planes.append(a)
    planes.sort(key=lambda a: -(a['bars'][0] + a['bars'][1]))
    if args.check:
        for a in planes:
            if a['id'] not in MEASURED: continue
            mp, mr = MEASURED[a['id']]
            ep = [a['pull'][k] / mp[k] for k in range(1, 7)]
            er = [a['roll'][k] / mr[k] for k in range(1, 7)]
            print(f"{a['title']:8s} table/measured 150-650 m/s  pull {' '.join(f'{x:.2f}' for x in ep)}  roll {' '.join(f'{x:.2f}' for x in er)}")
    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'aircraft')
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'aircraft.txt')
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        f.write('# made by import_aircraft.py from ' + os.path.basename(args.probe) + ' (game data: not in the repository)\n')
        f.write('# id | title | speed mobility stability | stall m/s | high-G | pull factors x9 | roll factors x9 | thrust x9 | brake x9 | top speed m/s\n')
        for a in planes:
            nums = lambda xs, fmt: ' '.join(fmt % x for x in xs)
            f.write(f"{a['id']}|{a['title']}|{nums(a['bars'], '%d')}|{a['stall']:.1f}|{HIGH_G}|"
                    f"{nums([x / PULL_BASE for x in a['pull']], '%.3f')}|{nums([x / ROLL_BASE for x in a['roll']], '%.3f')}|"
                    f"{nums(a['thrust'], '%.1f')}|{nums(a['brake'], '%.1f')}|{a['speed_max'] / 3.6:.1f}\n")
    print(f"{len(planes)} aircraft -> {path}")


if __name__ == '__main__':
    main()
