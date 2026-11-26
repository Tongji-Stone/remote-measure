"""Publish the E101 deliverables to a directory containing only point-cloud PLY."""
import json
import os
from pathlib import Path

HERE = Path(__file__).resolve().parent
TARGET = HERE.parents[1] / 'point cloud' / 'E101'
KNOWN_BRANCHES = ('ChArUco','Chessboard')


def attach(source, target):
    target.parent.mkdir(parents=True,exist_ok=True)
    temporary = target.with_suffix('.part')
    os.link(source, temporary)
    temporary.replace(target)


def pair(source, target, count):
    target.parent.mkdir(parents=True,exist_ok=True)
    temporary = target.with_suffix('.part')
    with source.open('rb') as origin, temporary.open('wb') as output:
        for line in origin:
            if line.startswith(b'element vertex '):
                line = f'element vertex {count}\n'.encode()
            output.write(line)
            if line == b'end_header\n':
                break
        data = origin.read(count * 16)
        if len(data) != count * 16:
            raise ValueError(source)
        output.write(data)
    temporary.replace(target)


def main():
    for branch in KNOWN_BRANCHES:
        for acquisition in sorted((HERE/branch).glob('*/采集.json')):
            case = acquisition.parent
            out = TARGET / branch / case.name
            stats = json.loads(acquisition.read_text())
            pair_count = sum(v['all_points'] for v in stats['views'][:2])
            for slide in sorted((case/'单视角').glob('*.ply')):
                attach(slide, out/'单视角'/slide.name)
            for scene in sorted((case/'整圈').glob('*.ply')):
                attach(scene, out/'整圈'/scene.name)
                pair(scene,out/'双视角'/scene.name,pair_count)
    print(TARGET)


if __name__ == '__main__':
    main()
