"""검증된 CMU 카탈로그를 Gazebo Plugin SDF로 전달한다; 데이터 재생은 하지 않는다."""
import copy
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

MODEL_NAME = 'custom_x500_cmu'


def validate_catalog(value):
    """네 프로파일의 팩 Ah/Ω·셀 V 곡선을 검사하고 독립 복사본을 반환한다."""
    if not isinstance(value, dict) or value.get('schema') != 'arion.cmu_battery_profiles.v1':
        raise ValueError('CMU runtime catalog schema mismatch')
    if value.get('series_cells') != 4 or value.get('parallel_cells') != 2:
        raise ValueError('This runtime supports the declared 4S2P reference pack only')
    rows = value.get('profiles')
    if not isinstance(rows, list) or len(rows) != 4:
        raise ValueError('Exactly four CMU profiles are required')
    ids = set()
    for p in rows:
        if not isinstance(p, dict) or not isinstance(p.get('id'), str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,48}', p['id']):
            raise ValueError('Invalid profile identifier')
        if p['id'] in ids:
            raise ValueError('Duplicate profile identifier')
        ids.add(p['id'])
        for key in ('capacity_ah', 'pack_resistance_ohm', 'initial_soc', 'soh_percent'):
            v = p.get(key)
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0:
                raise ValueError('Invalid finite positive profile value: ' + key)
        if p['initial_soc'] > 1 or p['capacity_ah'] > 100 or p['pack_resistance_ohm'] > 2 or p['soh_percent'] > 110:
            raise ValueError('Profile outside supported reference ranges')
        xs, ys = p.get('ocv_soc'), p.get('ocv_cell_v')
        if not isinstance(xs, list) or not isinstance(ys, list) or not 2 <= len(xs) <= 1000 or len(xs) != len(ys):
            raise ValueError('OCV curve length mismatch')
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in xs + ys):
            raise ValueError('Nonfinite OCV curve')
        if xs[0] != 0 or xs[-1] != 1 or any(a >= b for a, b in zip(xs, xs[1:])):
            raise ValueError('SOC curve must be strictly increasing from zero to one')
        if any(v < 2 or v > 4.5 for v in ys) or any(a > b for a, b in zip(ys, ys[1:])):
            raise ValueError('Cell OCV equivalent curve must be monotonic within 2..4.5 V')
        if not isinstance(p.get('source'), dict) or not p['source'] or not isinstance(p.get('label_ko'), str):
            raise ValueError('Missing profile provenance or label')
    if value.get('default_profile_id') not in ids:
        raise ValueError('Unknown default profile')
    json.dumps(value, allow_nan=False)
    return copy.deepcopy(value)


def write_model(directory, catalog, profile_id):
    """선택 프로파일과 모든 곡선을 새 모델 폴더에 기록하고 model.sdf 경로를 돌려준다."""
    data = validate_catalog(catalog)
    if profile_id not in {p['id'] for p in data['profiles']}:
        raise ValueError('Unknown selected profile: ' + str(profile_id))
    folder = Path(directory) / MODEL_NAME
    folder.mkdir(parents=True, exist_ok=False)
    root = ET.Element('sdf', version='1.9')
    model = ET.SubElement(root, 'model', name=MODEL_NAME)
    include = ET.SubElement(model, 'include', merge='true')
    ET.SubElement(include, 'uri').text = 'model://x500'
    plugin = ET.SubElement(model, 'plugin', filename='ArionCoupledPowerPlugin',
                           name='arion::CoupledPowerState')
    ET.SubElement(plugin, 'allow_command_fallback').text = 'false'
    ET.SubElement(plugin, 'initial_profile').text = profile_id
    profiles = ET.SubElement(plugin, 'battery_profiles')
    for entry in data['profiles']:
        node = ET.SubElement(profiles, 'profile', id=entry['id'])
        for key in ('capacity_ah', 'pack_resistance_ohm', 'initial_soc', 'ocv_soc', 'ocv_cell_v'):
            v = entry[key]
            ET.SubElement(node, key).text = ' '.join(map(str, v)) if isinstance(v, list) else str(v)
    ET.indent(root)
    target = folder / 'model.sdf'
    ET.ElementTree(root).write(target, encoding='utf-8', xml_declaration=True)
    config = ET.Element('model')
    ET.SubElement(config, 'name').text = MODEL_NAME
    ET.SubElement(config, 'version').text = '1.0'
    ET.SubElement(config, 'sdf', version='1.9').text = 'model.sdf'
    ET.SubElement(config, 'description').text = 'CMU-reference telemetry model; stock x500 propulsion.'
    ET.indent(config)
    ET.ElementTree(config).write(folder / 'model.config', encoding='utf-8', xml_declaration=True)
    return target


def write_world(source, target, model_instance):
    """원본 world를 보존한 사본에 기체를 초기 include한다; 전체 reset 후 동적 spawn 삭제를 피한다."""
    if not re.fullmatch(r'custom_x500_cmu_[0-9]{1,3}', model_instance):
        raise ValueError('Invalid model instance')
    root = ET.parse(source)
    world = root.getroot().find('world')
    if world is None or world.get('name') != 'default':
        raise ValueError('The reference default world is required')
    include = ET.SubElement(world, 'include')
    ET.SubElement(include, 'uri').text = 'model://' + MODEL_NAME
    ET.SubElement(include, 'name').text = model_instance
    ET.SubElement(include, 'pose').text = '0 0 0.2 0 0 0'
    ET.indent(root)
    with Path(target).open('xb') as handle:
        root.write(handle, encoding='utf-8', xml_declaration=True)
    return Path(target)
