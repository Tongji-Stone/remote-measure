"""S08 — Screened Poisson, calibrated free-space carving, and plate section closure."""
from pathlib import Path
import sys

ENTRY = Path(__file__).resolve()
sys.path.insert(0, str(ENTRY.parents[1] / '源码'))

from 复跑 import poisson, precise_closure, run

STEP_MM = .55


def reconstruct(case, _archive):
    mesh = precise_closure(poisson(case), case, STEP_MM)
    return mesh, case['p'], case['rgb'], {'step_mm': STEP_MM}


if __name__ == '__main__':
    run(ENTRY, reconstruct)
