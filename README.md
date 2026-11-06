# Summer_research_project_2026
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

## Research outcome — Showcase poster & manuscript (Sep–Oct 2026)

**When Is 3D Worth It? Task-Conditioned Adequacy of Planar Localisation for
Vision-Guided Manipulation.** A closed-form model of off-plane planar
localisation error (E_geom = |h|ρ / |Cz − h|) was validated blindly on a
dual-RGB-D setup (5 rebuilds × 2 cameras, 0–50 mm; height-curve r =
0.9942–0.9951) and combined with a separately declared real-system error bound
to decide when planar localisation is sufficient for a task tolerance τ.

- [Final showcase poster (`FURP_Showcase.pdf`)](FURP_Showcase.pdf)
- [Research summary, results and limitations](docs/research_paper/README.md)
- [Working manuscript v0.1](docs/research_paper/manuscript_v0.1_WORKING_DRAFT.pdf)

## 2026 dual-camera mobile-manipulator integration

This repository now includes the reproducible Berxel iHawk100 dual-camera and
RoArm M3 integration completed in August 2026.

- [System overview and quick start](docs/dual_berxel_roarm/README.md)
- [Task 1–10 execution record](docs/dual_berxel_roarm/TASKS_01_10.md)
- [Mathematics and calibration derivation](docs/dual_berxel_roarm/MATHEMATICS.md)
- [Troubleshooting and known limitations](docs/dual_berxel_roarm/TROUBLESHOOTING.md)
- [Code and data inventory](docs/dual_berxel_roarm/DATA_AND_CODE.md)

> Safety: the current robot-referenced absolute accuracy is approximately
> 10–12 mm and has not been validated across the full workspace. It is suitable
> for supervised coordinate experiments, not autonomous contact, low-clearance
> motion, or precision grasping.
