# FURP 2026 — Mobile Manipulator (YUNKAI CHOU)
| Field | Your entry |
| --- | --- |
| Student name(s) | YUNKAI CHOU |
| Project title | Whole-Body Control for a Mobile Manipulator with QP + Reinforcement Learning |
| Project tag | Mobile_Manipulator |
| Track | Research |
| Supervising faculty | Chiew-Foong Kwong |
| Project lead | Fuhua JIA |
| Team or individual | Individual |
| Cited paper being replicated | (XXXXXXX) |

## Highlights

**Final result — *When Is 3D Worth It? Task-Conditioned Adequacy of Planar
Localisation for Vision-Guided Manipulation*.** A closed-form model of
off-plane planar localisation error, E_geom = |h|ρ / |Cz − h|, was validated
blindly on a dual-RGB-D setup (5 rebuilds × 2 cameras, 0–50 mm; height-curve
r = 0.9942–0.9951) and combined with a separately declared real-system error
bound to decide when planar localisation is sufficient for a task tolerance τ.

- 📄 [Showcase poster — `FURP_Showcase.pdf`](FURP_Showcase.pdf)
- 📊 [Research summary, results and limitations](docs/research_paper/README.md)
- 📝 [Working manuscript v0.1](docs/research_paper/manuscript_v0.1_WORKING_DRAFT.pdf)

## Project timeline

| Period | Phase | Main output |
| --- | --- | --- |
| Jun 2026 | ROS 2 onboarding, MoveIt 2 + Panda simulation | [Phase 1 notes](docs/phase1_moveit_panda/README.md) |
| Jul–Aug 2026 | Dual Berxel iHawk100 + RoArm integration (Tasks 1–10) | [System docs](docs/dual_berxel_roarm/README.md), [`src/`](src/README.md) |
| Sep–Oct 2026 | Planar vs 3D localisation adequacy study, poster | [Research paper](docs/research_paper/README.md), [poster](FURP_Showcase.pdf) |

Week-by-week progress: [`docs/00_weekly.md`](docs/00_weekly.md).

## Repository structure

```text
.
├── FURP_Showcase.pdf          Final showcase poster
├── src/                       ROS 2 packages (camera driver, calibration, RoArm driver, grasp demo)
├── tools/dual_berxel_roarm/   Standalone checking, validation and approach scripts
├── experiments/dual_berxel_roarm/results/   Raw and summary data for Tasks 1–10
└── docs/
    ├── 00_weekly.md           Weekly progress log
    ├── meeting_notes/         Meeting notes (YYYY-MM-DD.md)
    ├── notebooks/             Daily lab notebooks
    ├── phase1_moveit_panda/   Phase 1: MoveIt 2 tutorial, pick-and-place and troubleshooting logs
    ├── dual_berxel_roarm/     Phase 2: system overview, tasks, maths, troubleshooting, data map
    ├── research_paper/        Phase 3: research summary, manuscript and positioning drafts
    ├── poster/                Poster sources, earlier versions and template
    └── reference/             Programme handbooks (FURP SEP, ROS 2, GitHub tutorial)
```

## Dual-camera system documentation

- [System overview and quick start](docs/dual_berxel_roarm/README.md)
- [Task 1–10 execution record](docs/dual_berxel_roarm/TASKS_01_10.md)
- [Mathematics and calibration derivation](docs/dual_berxel_roarm/MATHEMATICS.md)
- [Troubleshooting and known limitations](docs/dual_berxel_roarm/TROUBLESHOOTING.md)
- [Code and data inventory](docs/dual_berxel_roarm/DATA_AND_CODE.md)

> **Safety:** the current robot-referenced absolute accuracy is approximately
> 10–12 mm and has not been validated across the full workspace. It is suitable
> for supervised coordinate experiments, not autonomous contact, low-clearance
> motion, or precision grasping.
