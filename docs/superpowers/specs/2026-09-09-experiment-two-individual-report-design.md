# Experiment 2 Individual Report Design

## Objective

Create a detailed English LaTeX individual report for Haoran Xiao (Student ID 24020036066) based only on the laboratory requirements and verifiable Experiment 2 repository evidence. The report will describe the author's own work in the first person and will not evaluate, compare, or speculate about other group members' participation.

## Required Structure

The report will follow the four required content areas:

1. Basic Experimental Requirements
2. Individual Responsibilities and Contributions
3. Principles and Project Implementation
4. Problems, Solutions, and Personal Reflection

Supporting front and back matter will include a title page, abstract, contents, evidence note, conclusion, and references.

## Style and Format

- Reuse the visual language of the Experiment 1 LaTeX report: A4 page, Times-style English body font, clear chapter hierarchy, centered header, numbered figures and tables, and restrained academic formatting.
- Use first-person singular where personal decisions and work are described.
- Keep the tone technical, reflective, and evidence-based rather than promotional.
- Target approximately 15--20 pages after figures and appendices.
- Use the title-page identity `Haoran Xiao` and `24020036066`.

## Evidence Strategy

The report will support personal-contribution statements with repository and experiment evidence:

- Git history: all 33 commits currently visible in the Experiment 2 repository are authored by `xhr-CHN`; include a GitHub commit-history screenshot and a compact contribution table.
- Simulation: Isaac Sim scene image, successful five-cycle simulation frames, and the preserved failure-case frames.
- Mechanical debugging: adaptive-gripper linkage/pin analysis and repair figures.
- Runtime evidence: ROS 2/Docker/TCP architecture, direct-motion probe output, numerical IK checks, S-curve trajectory design, and exception handling.
- Physical evidence: RoboMaster EP five-cycle recording frames and the implemented retry/safe-exit logic. Because the available laboratory hardware differed from the simulated arm, the completed experiment used mechArm 270 in simulation and DJI RoboMaster EP for physical verification.
- Lightweight verification: include the observed build result and the latest test result honestly, including known stale contract-test assertions.

No Experiment 3 files, results, or descriptions will appear in the report.

## Contribution Narrative

The report will focus on the author's documented work:

- repository and workspace organization;
- selection and integration of the official mechArm model;
- Isaac Sim scene generation;
- ROS 2 Humble, MoveIt 2, Docker, and TCP bridge integration;
- numerical IK and vertical-grasp pose design;
- smooth joint interpolation and differentiated vertical-motion speed;
- adaptive-gripper four-bar linkage repair and dynamics tuning;
- alternating five-cycle task and empty-grasp exception program;
- simulation and physical experiment recording;
- bilingual documentation and reproducible startup instructions.

Claims will be phrased as `I implemented`, `I diagnosed`, `I verified`, or `I recorded` only where supported by source, commit, log, image, or video evidence.

## Planned Figures and Tables

1. GitHub repository commit-history screenshot.
2. Final Isaac Sim experiment scene.
3. Windows/Docker/ROS 2/Isaac TCP architecture diagram.
4. Fixed-point pick-and-place state sequence.
5. Adaptive-gripper linkage and repaired pin constraints.
6. Successful five-cycle simulation contact sheet.
7. Simulation failure-case contact sheet.
8. RoboMaster EP physical-test contact sheet.
9. Table mapping requirements to personal work and evidence.
10. Table summarizing major problems, root causes, solutions, and lessons.

## Accuracy Boundaries

- State that both the simulation and physical stages were completed: mechArm 270 was used in Isaac Sim, while DJI RoboMaster EP was used for the real-robot stage because of equipment availability. Do not introduce a Jetson requirement that does not apply to this experiment.
- Do not convert video evidence into an invented machine-readable success table.
- Do not claim full automated regression success when two stale contract assertions remain.
- Do not describe Git commits as proof that no one else participated; use them only as evidence of the author's own sustained implementation work.
- Separate demonstrated results, partial evidence, and remaining acceptance work.

## Deliverables

- `report/individual_report/main.tex`
- `report/individual_report/figures/` with selected evidence images
- `report/individual_report/experiment2_individual_report_haoran_xiao.pdf`

The PDF will be compiled with the bundled LaTeX toolchain, rendered page by page, and visually checked for clipping, overlap, missing glyphs, figure readability, and consistent pagination.
