"""S09 — S08 with measured exposed-board evidence on the contact layer."""
from pathlib import Path
import sys

import numpy as np

ENTRY = Path(__file__).resolve()
sys.path.insert(0, str(ENTRY.parents[1] / '源码'))

from 复跑 import PYRAMIDS, poisson, precise_closure, run

STEP_MM = .55


def reconstruct(case, archive):
    evidence = archive / '同期露板证据.npz'
    with np.load(evidence) as saved:
        views = [saved[key] for key in sorted(saved.files)]
    mesh = precise_closure(poisson(case), case, STEP_MM, exposed=views)
    return mesh, case['p'], case['rgb'], {'step_mm': STEP_MM, 'exposed_board_stations': len(views)}


if __name__ == '__main__':
    run(ENTRY, reconstruct, PYRAMIDS)
