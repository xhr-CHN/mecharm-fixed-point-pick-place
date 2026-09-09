# Experiment 2 Individual Report Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce a polished English LaTeX individual report that documents Haoran Xiao's verified Experiment 2 contributions and completed simulation and physical-robot work.

**Architecture:** Reuse the Experiment 1 report typography and the verified Experiment 2 evidence set. Keep the report source, copied evidence figures, reproducibility helpers, build output, and final PDF under a dedicated `report/individual_report/` directory so it does not overwrite the group report.

**Tech Stack:** XeLaTeX/Tectonic, ctexart, TikZ, Git/GitHub history, Playwright browser screenshot, OpenCV/Pillow contact sheets, Poppler PDF rendering.

**Spec:** `docs/superpowers/specs/2026-09-09-experiment-two-individual-report-design.md`

## Global Constraints

- Write the report in English and use first-person singular for personal work.
- Cover identity is `Haoran Xiao`, Student ID `24020036066`.
- Follow the four required sections: basic requirements, personal contribution, implementation principles, and problems/solutions/personal reflection.
- Describe only the author's work; do not evaluate or speculate about other group members.
- State that both stages were completed: mechArm 270 in Isaac Sim and DJI RoboMaster EP for physical verification because of equipment availability.
- Include no Experiment 3 files, results, or discussion.
- Base every contribution claim on Git history, source/configuration, logs, images, or recordings.
- Do not overwrite the existing group report.

---

### Task 1: Collect and Normalize Personal-Contribution Evidence

**Files:**
- Create: `report/individual_report/figures/github_commit_history.png`
- Create: `report/individual_report/figures/isaac_scene.png`
- Create: `report/individual_report/figures/gripper_hole_projection_scene.png`
- Create: `report/individual_report/figures/gripper_hole_projection.png`
- Create: `report/individual_report/figures/simulation_five_cycle_contact_sheet.jpg`
- Create: `report/individual_report/figures/simulation_failure_contact_sheet.jpg`
- Create: `report/individual_report/figures/hardware_five_cycle_contact_sheet.jpg`
- Create: `report/individual_report/evidence/git_history.txt`
- Create: `report/individual_report/evidence/git_summary.txt`

**Interfaces:**
- Consumes: repository `main` history, public GitHub commit page, existing group-report figures, and three Experiment 2 recordings.
- Produces: stable local evidence assets referenced by `main.tex`.

- [ ] **Step 1: Record the exact Git evidence**

Run from `E:\机器人集成小组项目\实验二`:

```powershell
git log main --date=short --pretty=format:"%h|%ad|%an|%ae|%s" > report/individual_report/evidence/git_history.txt
git shortlog -sne main > report/individual_report/evidence/git_summary.txt
```

Expected: 33 commits are listed and `git shortlog` shows the author `xhr-CHN`.

- [ ] **Step 2: Capture the public GitHub commit-history page**

Open and capture:

```text
https://github.com/xhr-CHN/mecharm-fixed-point-pick-place/commits/main/
```

Expected: the screenshot visibly contains the repository name, `main` branch history, author identity, commit messages, and dates. Save only the page content needed as contribution evidence.

- [ ] **Step 3: Copy the verified Experiment 2 figures**

Copy the seven existing scene, gripper, simulation, and hardware evidence images from `report/figures/` into `report/individual_report/figures/` without modifying the originals.

Expected: all seven copied images open successfully and retain their original pixel dimensions.

- [ ] **Step 4: Verify scope and evidence integrity**

Run:

```powershell
rg -n "experiment3|Experiment 3|实验三" report/individual_report
Get-FileHash report/figures/*.jpg,report/individual_report/figures/*.jpg -Algorithm SHA256
```

Expected: no Experiment 3 match; copied JPG pairs have matching hashes.

### Task 2: Author the English Individual Report

**Files:**
- Create: `report/individual_report/main.tex`
- Create: `report/individual_report/fonts/times.ttf`
- Create: `report/individual_report/fonts/timesbd.ttf`
- Create: `report/individual_report/fonts/timesi.ttf`
- Create: `report/individual_report/fonts/timesbi.ttf`
- Create: `report/individual_report/fonts/simsun.ttc`
- Create: `report/individual_report/fonts/cambria.ttc`

**Interfaces:**
- Consumes: Task 1 evidence assets, `机械臂定点抓取实验要求.docx`, Experiment 1 `report/main.tex`, group-report `report/main.tex`, and tracked Experiment 2 source/configuration.
- Produces: a standalone XeLaTeX document with all required sections and local assets.

- [ ] **Step 1: Create the report preamble and title matter**

Use the Experiment 1 A4 geometry, local Times/SimSun fonts, `fancyhdr`, `booktabs`, `tabularx`, `longtable`, `tikz`, `listings`, and English chapter naming. Set:

```text
Name: Haoran Xiao
Student ID: 24020036066
Experiment: Fixed-Point Robotic-Arm Pick-and-Place
Report Type: Individual Report
```

Expected: title page, abstract, keywords, and contents compile without missing glyphs.

- [ ] **Step 2: Write the basic requirements and personal contribution chapters**

Include a requirement-to-evidence matrix and a first-person contribution timeline. Cite the 33-commit history as evidence of sustained work, without claiming it proves the absence of other contributors. State explicitly that the simulation used mechArm 270 and the completed real-robot stage used DJI RoboMaster EP because of available equipment.

Expected: both completed stages are described positively and accurately; no Jetson requirement is introduced.

- [ ] **Step 3: Write the implementation-principles chapter**

Explain and illustrate:

```text
Isaac Sim 5.1 -> TCP 8765 -> Docker ROS 2 Humble -> MoveIt/numerical IK
HOME -> pre-grasp -> vertical descend -> close -> lift -> transport -> release -> HOME
quintic smoothstep: s(u) = 6u^5 - 15u^4 + 10u^3
gripper mirror command: q_left = q_master, q_right = -q_master
```

Include the fixed A/B coordinates, vertical-axis constraint, grasp-yaw offset, normal/vertical speed separation, adaptive-gripper loop-joint repair, five-cycle alternating logic, empty-grasp exception, and RoboMaster EP retry/safe-exit logic.

Expected: every numerical value agrees with the checked-in Experiment 2 source/configuration or is explicitly labeled as recorded evidence.

- [ ] **Step 4: Write the problem-solving and reflection chapter**

For each major issue, use the structure `symptom -> diagnosis -> attempted approaches -> final solution -> lesson learned`. Cover DDS/TUN discovery, Docker proxy/build failures, trajectory stutter, incorrect grasp orientation, numerical IK reachability, adaptive-gripper missing loop pins, mimic-joint asymmetry, dynamics tuning/rollback, and empty-grasp recovery.

Expected: the chapter emphasizes personal reasoning and engineering trade-offs rather than listing errors mechanically.

- [ ] **Step 5: Add evidence appendix and references**

Include the GitHub screenshot, concise commit table, source/file cross-reference, reproducible startup commands, video evidence method, and references for Elephant Robotics, Isaac Sim, ROS 2, MoveIt 2, and RoboMaster SDK.

Expected: all figures are numbered and discussed in the body; all URLs are human-readable.

- [ ] **Step 6: Run source-level consistency checks**

Run:

```powershell
rg -n "TBD|TODO|PLACEHOLDER|Experiment 3|实验三|Jetson" report/individual_report/main.tex
rg -n "Haoran Xiao|24020036066|Individual Report|RoboMaster EP|mechArm 270" report/individual_report/main.tex
```

Expected: the first command has no matches; the second confirms all required identity and platform statements.

### Task 3: Compile, Render, and Visually Verify the PDF

**Files:**
- Create: `report/individual_report/build/main.pdf`
- Create: `report/individual_report/experiment2_individual_report_haoran_xiao.pdf`
- Create: `report/individual_report/tmp/pdfs/page-*.png`

**Interfaces:**
- Consumes: Task 2 `main.tex`, fonts, and figures.
- Produces: final submission PDF and disposable rendered QA pages.

- [ ] **Step 1: Register the PDF authoring operation**

Run exactly once immediately before report authoring/compilation:

```powershell
node container_tools/mark_artifact_operation_started.mjs --operation-kind create --expected-output-count 1 --output-format pdf
```

Expected: the artifact operation is accepted successfully.

- [ ] **Step 2: Compile the report**

Run the bundled compiler from the LaTeX plugin root:

```powershell
python scripts/compile_latex.py "E:\机器人集成小组项目\实验二\report\individual_report\main.tex" --compiler tectonic --output-directory "E:\机器人集成小组项目\实验二\report\individual_report\build"
```

Expected: `build/main.pdf` is created with no fatal errors. Copy it to `experiment2_individual_report_haoran_xiao.pdf`.

- [ ] **Step 3: Render every PDF page**

Run:

```powershell
pdftoppm -png -r 110 report/individual_report/experiment2_individual_report_haoran_xiao.pdf report/individual_report/tmp/pdfs/page
pdfinfo report/individual_report/experiment2_individual_report_haoran_xiao.pdf
```

Expected: every page renders; the PDF is A4, unencrypted, and has a nonzero file size.

- [ ] **Step 4: Perform visual QA**

Inspect all rendered pages for clipping, overlap, unreadable GitHub text, missing Chinese filename glyphs, inconsistent headers/footers, orphan headings, and excessive blank pages. Fix `main.tex`, recompile, and rerender if any defect is found.

Expected: zero visible formatting defects and all evidence figures remain legible at normal zoom.

- [ ] **Step 5: Run final content checks**

Extract PDF text with `pypdf` and confirm:

```text
Haoran Xiao
24020036066
Basic Experimental Requirements
Individual Responsibilities and Contributions
Principles and Project Implementation
Problems, Solutions, and Personal Reflection
```

Also confirm the extracted text contains no `Experiment 3`, `实验三`, `TBD`, or `TODO`.

Expected: all required strings are present, forbidden strings are absent, and the final PDF opens successfully.

### Task 4: Record the Report Deliverable

**Files:**
- Modify: `docs/superpowers/plans/2026-09-09-experiment-two-individual-report.md`
- Add: `report/individual_report/main.tex`
- Add: `report/individual_report/figures/`
- Add: `report/individual_report/evidence/`
- Add: `report/individual_report/experiment2_individual_report_haoran_xiao.pdf`

**Interfaces:**
- Consumes: the visually verified Task 3 deliverables.
- Produces: a reviewable local Git commit containing only Experiment 2 individual-report material.

- [ ] **Step 1: Review the exact staged scope**

Run:

```powershell
git status --short
git diff --check -- report/individual_report docs/superpowers/specs/2026-09-09-experiment-two-individual-report-design.md docs/superpowers/plans/2026-09-09-experiment-two-individual-report.md
```

Expected: Experiment 3 and unrelated untracked files remain unstaged.

- [ ] **Step 2: Commit the report locally**

Run:

```powershell
git add report/individual_report docs/superpowers/specs/2026-09-09-experiment-two-individual-report-design.md docs/superpowers/plans/2026-09-09-experiment-two-individual-report.md
git commit -m "docs: add experiment two individual report"
```

Expected: one local `main` commit contains only the design, plan, report source, selected evidence, and final PDF. Do not push unless explicitly requested.
