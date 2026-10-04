"""Acceptance of recorded PX4 output, never of sender stdout alone."""


def autopilot_rows(rows, system_id, kind, phase=None):
    return [r for r in rows if r['source_system'] == system_id and
            r['source_component'] == 1 and r['fields'].get('mavpackettype') == kind and
            (phase is None or r['phase'] == phase)]


def validate_fixed(rows, system_id):
    selected = autopilot_rows(rows, system_id, 'BATTERY_STATUS', 'fixed')
    if len(selected) < 3:
        raise ValueError('Need at least three PX4-generated battery packets')
    expected = {'id': 0, 'current_battery': 725, 'battery_remaining': 73,
                'temperature': 2850, 'current_consumed': 123}
    for row in selected:
        fields = row['fields']
        for key, value in expected.items():
            if fields.get(key) != value:
                raise ValueError(f'Fixed battery mismatch: {key}: {fields.get(key)} != {value}')
        if fields.get('voltages') != [4010, 4020, 4030, 4040] + [65535] * 6:
            raise ValueError('Fixed cell voltages or unknown slots differ')
    return {'battery_packets': len(selected), 'expected_fields': expected,
            'first_actual_packet': selected[0]}


def validate_flight(rows, system_id):
    import math
    import statistics
    from collections import Counter

    def selected(kind, phase=None):
        return [r['fields'] for r in autopilot_rows(rows, system_id, kind, phase)]

    def require(condition, message):
        if not condition:
            raise ValueError(message)

    before = selected('LOCAL_POSITION_NED', 'preflight')
    hover = selected('LOCAL_POSITION_NED', 'hover')
    after = selected('LOCAL_POSITION_NED', 'landed')
    require(before and hover and after, 'Missing preflight/hover/landed position evidence')
    z0 = statistics.median(f['z'] for f in before)
    heights = [z0 - f['z'] for f in selected('LOCAL_POSITION_NED')]
    require(all(math.isfinite(x) for x in heights), 'Nonfinite position')
    require(max(heights) >= 2.0, 'Takeoff altitude was not observed')
    hover_height = statistics.median(z0 - f['z'] for f in hover)
    require(2.0 <= hover_height <= 4.5, 'Hover height outside the 3m reference window')
    require(max(abs(f['vz']) for f in hover) < 0.8, 'No settled hover')
    require(abs(statistics.median(f['z'] for f in after) - z0) < 0.6, 'Did not return to ground altitude')
    require(any(f['base_mode'] & 128 for f in selected('HEARTBEAT', 'hover')), 'No armed heartbeat in hover')
    final_hb = selected('HEARTBEAT', 'landed')
    require(final_hb and not final_hb[-1]['base_mode'] & 128, 'Final heartbeat is not disarmed')
    require(any(f['landed_state'] == 2 for f in selected('EXTENDED_SYS_STATE')), 'No in-air state')
    final_land = selected('EXTENDED_SYS_STATE', 'landed')
    require(final_land and final_land[-1]['landed_state'] == 1, 'No final on-ground state')

    esc = selected('ESC_STATUS', 'hover')
    require(esc, 'Missing hover ESC data')
    for f in esc:
        require(f['index'] == 0 and all(len(f[k]) == 4 for k in ['rpm', 'voltage', 'current']), 'ESC identity/length mismatch')
        require(all(isinstance(x, (int, float)) and math.isfinite(x) and x >= 0
                    for k in ['rpm', 'voltage', 'current'] for x in f[k]), 'Invalid ESC field')
    require(all(max(f['rpm'][i] for f in esc) > 1000 for i in range(4)), 'Four rotating motors not observed')
    pre_bat = selected('BATTERY_STATUS', 'preflight')
    hov_bat = selected('BATTERY_STATUS', 'hover')
    post_bat = selected('BATTERY_STATUS', 'landed')
    require(pre_bat and hov_bat and post_bat, 'Missing battery phase data')
    all_bat = selected('BATTERY_STATUS')
    require(all(f['id'] == 0 and f['current_battery'] >= 0 and len(f['voltages']) == 10
                and all(0 < v < 65535 for v in f['voltages'][:4]) for f in all_bat), 'Invalid battery identity or fields')
    pre_current = statistics.median(f['current_battery'] / 100 for f in pre_bat)
    hover_current = statistics.median(f['current_battery'] / 100 for f in hov_bat)
    hover_motor_sum = statistics.median(sum(f['current']) for f in esc)
    require(hover_current > pre_current + 2.0, 'Battery does not respond to motor load')
    require(abs(hover_current - hover_motor_sum - 0.5) < 0.5, 'Battery current does not match four motors plus 0.5A auxiliary load')
    pack_v = lambda f: sum(f['voltages'][:4]) / 1000
    pre_voltage = statistics.median(pack_v(f) for f in pre_bat)
    hover_voltage = statistics.median(pack_v(f) for f in hov_bat)
    require(hover_voltage < pre_voltage - 0.05, 'No load-dependent voltage sag')
    used = post_bat[-1]['current_consumed'] - pre_bat[0]['current_consumed']
    require(used > 0, 'No consumed charge accumulated')
    phase_counts = Counter((r['phase'], r['fields'].get('mavpackettype')) for r in rows
                           if r['source_system'] == system_id and r['source_component'] == 1)
    return {'max_relative_altitude_m': max(heights), 'hover_relative_altitude_m': hover_height,
            'preflight_battery_current_a': pre_current, 'hover_battery_current_a': hover_current,
            'hover_motor_current_sum_a': hover_motor_sum, 'auxiliary_current_a': 0.5,
            'preflight_battery_voltage_v': pre_voltage, 'hover_battery_voltage_v': hover_voltage,
            'consumed_during_run_mah': used,
            'hover_rpm_medians': [statistics.median(f['rpm'][i] for f in esc) for i in range(4)],
            'final_disarmed': True, 'final_on_ground': True,
            'phase_message_counts': {f'{p}/{k}': n for (p, k), n in sorted(phase_counts.items())}}
