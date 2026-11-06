# When Is 3D Worth It? Task-Conditioned Adequacy of Planar Localisation for Vision-Guided Manipulation

Research outcome of the 2026 summer project (Sep–Oct 2026), built on the
dual-camera / RoArm platform documented in [`../dual_berxel_roarm`](../dual_berxel_roarm/README.md).

| Item | File |
| --- | --- |
| Showcase poster (final) | [`/FURP_Showcase.pdf`](../../FURP_Showcase.pdf) · [editable .pptx](../poster/Research_Poster_Final.pptx) |
| Working manuscript v0.1 (not final) | [`manuscript_v0.1_WORKING_DRAFT.pdf`](manuscript_v0.1_WORKING_DRAFT.pdf) |
| Positioning / closest-work audit v1.1 | [`positioning_v1.1_closest_work_DRAFT.pdf`](positioning_v1.1_closest_work_DRAFT.pdf) |
| Editable Word sources | [manuscript .docx](manuscript_v0.1_WORKING_DRAFT.docx) · [positioning .docx](positioning_v1.1_closest_work_DRAFT.docx) |
| Earlier poster version / official template | [V3 .pptx](../poster/Research_Poster_V3_Completed.pptx) · [template .pptx](../poster/Poster_Template_2026_FoSE_Showcase.pptx) / [.pdf](../poster/Poster_Template_2026_FoSE_Showcase.pdf) |
| Programme reference material | [FURP SEP handbook](../reference/FURP_SEP_Handbook.pdf) · [ROS 2 handbook](../reference/ROS2_handbook.pdf) · [GitHub repo tutorial](../reference/GitHub-repo-tutorial.pdf) |
| Poster narrative discussion v2 | [`../poster/Research_Poster_Manuscript_Discussion_v2.pdf`](../poster/Research_Poster_Manuscript_Discussion_v2.pdf) |

## Research question

Under what geometric and bounded-error conditions does a planar
image-to-workspace localisation model remain adequate for a specified
manipulation tolerance despite off-plane model mismatch — and when is a
3D-aware method the only one that can provide the task guarantee?

## Method

1. **Analytical geometric model.** Camera centre C = (Cx, Cy, Cz), target
   P = (X, Y, h), calibration plane Z = 0:
   - ΔX = h(X − Cx)/(Cz − h), ΔY = h(Y − Cy)/(Cz − h)
   - ρ = √[(X − Cx)² + (Y − Cy)²]
   - **E_geom = |h|ρ / |Cz − h|** — error depends jointly on height *and*
     lateral camera–target distance, so no universal height threshold exists.
2. **Blind physical validation (E2).** Two Berxel iHawk RGB-D cameras +
   calibrated marker board; 5 physical rebuilds; 5 locations;
   h = 0:5:50 mm. Predictions use only Z = 0 geometry — no off-plane data is
   fitted.
3. **Task-conditioned adequacy.** e_P = e_geom + r_P with ‖r_P‖ ≤ S_P
   (separately declared bound):
   - Planar guaranteed if E_geom + S_P ≤ τ
   - 3D guaranteed if S_3D ≤ τ
   - Robust 3D accuracy dominance if E_geom > S_P + S_3D (a separate question
     from task necessity). "Not guaranteed" ≠ failure.

## Key results

| Evidence | Quantity | Value |
| --- | --- | --- |
| E1 (in-plane baseline) | Homography mean / RMSE / max | 0.585 / 0.660 / 1.197 mm |
| E1 | PnP mean / RMSE / max | 0.587 / 0.661 / 1.293 mm |
| E2 (5 rebuilds × 2 cameras, 0–50 mm) | Height-curve correlation | 0.9942–0.9951 |
| E2 | Point-level magnitude correlation | 0.9869–0.9942 |
| E2 | Median direction cosine | 0.9983–0.9992 |
| E2 / Stage 3 | Geometry-unexplained vector RMSE | 3.61–4.36 mm |
| Stage 3 | Measurement-only planar radial Q95 (σh = 0) | 0.024–0.266 mm |
| Stage 3 | Empirical PnP XY RMSE (cam 1 / cam 2) | 9.99 / 5.10 mm |
| STAR-loc (external) | 12,473 obs., 55 AprilTags, 8 held-out sequences | Spearman 0.981, median cosine 0.998 |

Illustrative adequacy example (iHawk 1, Cz ≈ 359 mm, centre marker ρ = 516.47 mm,
τ = 10, S_P = 2, S_3D = 5 mm — hypothetical bounds):

- h = 5 mm → E_geom = 7.29 mm, planar bound 9.29 mm < τ → **both guaranteed**
- h = 20 mm → E_geom = 30.47 mm, planar bound 32.47 mm > τ → **3D only guaranteed**

## Conclusions

- Geometry predicts the dominant off-plane degradation structure blindly.
- Geometry does not explain all error: a 3.61–4.36 mm residual remains, far
  above measurement-only variability, so a separate real-system bound is needed.
- Adequacy is decided by task tolerance: **3D is not always necessary, even
  when 2D is geometrically imperfect.**
- STAR-loc supports cross-system generality of the mechanism and exposes a
  rare near-grazing-ray failure tail (top ~0.1% of observations ≈ 99.79% of SSE).

## Limitations

- Guarantees are conditional on declared S_P, S_3D, τ; the poster scenarios are
  illustrative, not certified. Residual RMSE is not S_P.
- XY metric only; no full SE(3) or grasp-success validation.
- Board-relative h with fixed rotation; robot Z not verified.
- STAR-loc is structural support only, not absolute-accuracy replication.

## Status / next steps

- Manuscript v0.1 is a working draft: author list, exact E2 hardware/protocol
  tables, E1/E2 sample counts and final Figures 2, 3, 5 still to be filled.
- Positioning v1.1 (Sections 2–3) awaits re-freezing.
- E2 frozen package reference: `final-SEP`, commit `9a2844f`.
