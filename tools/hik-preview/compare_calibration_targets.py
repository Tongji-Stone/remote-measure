"""比较两套相机—投影仪标定。在同一批留出姿态比较投影转移误差。"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from automatic_calibration import fit, pose_residuals, rms


def load(xml):
    metadata = json.loads(xml.with_suffix('.json').read_text(encoding='utf-8'))
    observations = xml.with_name(xml.stem + '_observations')
    views = {}
    for pose in metadata['poses']:
        name = pose['folder']
        if pose['status'] in ('used', 'camera_only'):
            with np.load(observations / (name + '.npz')) as arrays:
                views[name] = {key: arrays[key] for key in
                               ('world', 'camera', 'shared_world', 'shared_camera', 'projector')}
    model = {key: np.array(value) for key, value in metadata['parameters'].items()}
    return dict(views=views, model=model, stats=metadata)


def evaluate(model, views):
    records, cam, proj = [], [], []
    for name, view in views.items():
        _, _, ec, ep = pose_residuals(view, model)
        cam.extend(ec)
        proj.extend(ep)
        records.append(dict(pose=name, camera_rms_px=rms(ec), projector_transfer_rms_px=rms(ep),
                            camera_points=len(ec), projector_points=len(ep)))
    camera = np.array(cam)
    projector = np.array(proj)
    return dict(views=len(records), camera_rms_px=rms(camera), projector_transfer_rms_px=rms(projector),
                projector_transfer_median_px=float(np.median(projector)),
                projector_transfer_p95_px=float(np.percentile(projector, 95)),
                per_view=records)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('calibrations', nargs=2, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    cv2.setNumThreads(3)
    data = {p.stem: load(p) for p in args.calibrations}
    if len(data) != 2:
        raise ValueError('两套标定文件名必须不同')
    full = {a: {b: evaluate(train['model'], test['views']) for b, test in data.items()}
            for a, train in data.items()}
    # 每折按采集顺序交错留出。两种训练模型在每折使用同一份测试数据，
    # 留出姿态的相机内参、投影仪内参、相对外参均来自各自训练折。
    folds = []
    for fold in range(4):
        train_models, test_views = {}, {}
        for key, bundle in data.items():
            items = sorted(bundle['views'].items())
            test_views[key] = {n: v for i, (n, v) in enumerate(items) if i % 4 == fold}
            train = [v for i, (_, v) in enumerate(items) if i % 4 != fold]
            meta = bundle['stats']
            model, stats = fit(train, tuple(meta['camera_size_px']), tuple(meta['projector_size_px']))
            train_models[key] = model
            print('fold', fold + 1, key, len(train), stats, flush=True)
        folds.append({a: {b: evaluate(model, test) for b, test in test_views.items()}
                      for a, model in train_models.items()})
    aggregate = {}
    for a in data:
        aggregate[a] = {}
        for b in data:
            rows = [row for fold in folds for row in fold[a][b]['per_view']]
            aggregate[a][b] = {}
            for metric, count in [('camera_rms_px', 'camera_points'),
                                  ('projector_transfer_rms_px', 'projector_points')]:
                total = sum(row[count] for row in rows)
                aggregate[a][b][metric] = float(np.sqrt(sum(
                    (row[metric] or 0) ** 2 * row[count] for row in rows) / total))
    result = dict(full_models_evaluated_on_all_poses=full, held_out_folds=folds, held_out_pooled=aggregate)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(aggregate, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
