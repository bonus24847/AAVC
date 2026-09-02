# docs/ — index

The documents themselves live next to the code that produced them. This page is
the map, so you do not have to guess which of ~40 files answers your question.

## Start here

| you want to | read |
|---|---|
| run the console on your own machine | [`../gcs/README.md`](../gcs/README.md) |
| understand the whole system | [`../README.md`](../README.md) |
| understand the mission and its architecture | [`../mission/README.md`](../mission/README.md) |
| know what actually happened and why | [`../mission/CLAUDE.md`](../mission/CLAUDE.md) §0b–§0j |
| know what is third-party in here | [`../NOTICE`](../NOTICE) |

`mission/CLAUDE.md` is the project's real logbook. It is written as context for
an AI pair-programmer, so it is blunt and dense, but it is the only place that
records *why* each number is what it is — and every post-mortem in it
(§0d the height-frame drift, §0f the tip-over mechanism, §0g the CM4 power loss,
§0h the seven pre-competition fixes) is written against evidence in this repo.

## The rules and the field

| file | what |
|---|---|
| [`../mission/docs/RULES_AAVC2026.md`](../mission/docs/RULES_AAVC2026.md) | digest of the rulebook + the 28-Aug event-briefing overrides |
| `../mission/AAVC2026_RulesAndRegulation_V1.3_140769-2.pdf` | the organisers' own document |
| [`../mission/docs/FIELD_SURVEY.md`](../mission/docs/FIELD_SURVEY.md) | how a field is surveyed into a config |
| `../mission/docs/evidence/briefing_corridor_2026-08-28.md` | the corridor as georeferenced from the briefing |

## Flying it

| file | what |
|---|---|
| [`../mission/docs/FLIGHT.md`](../mission/docs/FLIGHT.md) | real-aircraft flight procedure |
| [`../mission/docs/REAL_FLIGHT_GCS.md`](../mission/docs/REAL_FLIGHT_GCS.md) | the console against the real bird |
| [`../mission/docs/CM4_ACCESS.md`](../mission/docs/CM4_ACCESS.md) | getting onto the companion computer |
| [`../gcs/docs/FLIGHT_READINESS.md`](../gcs/docs/FLIGHT_READINESS.md) | the commissioning history that made it fly |
| `../mission/docs/AAVC_Preflight_Checklist_Full.pdf` | the printed preflight checklist |
| `../mission/docs/AAVC_Checklist_Competition_KMITL.pdf` | the competition-day checklist |
| `../mission/docs/AAVC_Checklist_Practice_KMUTNB.pdf` | the practice-day checklist |
| [`../mission/docs/SERVO_AUX_MAPPING.md`](../mission/docs/SERVO_AUX_MAPPING.md) | which egg latch is on which AUX pin |
| [`../mission/docs/BANGBO_LANDING_TEST_2026-08-29.md`](../mission/docs/BANGBO_LANDING_TEST_2026-08-29.md) | runbook for the night landing test |

## The aircraft

| file | what |
|---|---|
| [`../mission/docs/AIRFRAME_SIZING.md`](../mission/docs/AIRFRAME_SIZING.md) | why this airframe, and what it implies for the gains |
| [`../mission/docs/ECALC_PASS.md`](../mission/docs/ECALC_PASS.md) | propulsion sizing check |
| [`../mission/docs/BOM_REPORT.md`](../mission/docs/BOM_REPORT.md) | bill of materials |
| [`../mission/docs/HITL.md`](../mission/docs/HITL.md) + `HITL_CHECKLIST.md` | hardware-in-the-loop against the real FC |

There is also a `PX4MASTER` skill under `../mission/.claude/skills/` — field-proven
PX4 procedures for this aircraft, with a catalogue of the bugs that bit us
(params, battery gauge, GPS/height, MAVLink routing, SITL quirks). Written for
Claude Code, readable by anyone.

## Submissions

| file | what |
|---|---|
| `../mission/docs/AAVC2026_Technical_Report.pdf` (+ `.md`, `_TH.md`, `.tex`) | the technical report |
| `../mission/docs/presentation/` | the presentation deck and its script |
| `../mission/docs/report/` | the BOM / performance report and its data |
| `../mission/docs/report_figures/` | the figures those documents use |

## Reviews and post-mortems

| file | what |
|---|---|
| [`../mission/docs/REVIEW_2026-08-21_preflight.md`](../mission/docs/REVIEW_2026-08-21_preflight.md) | pre-flight review |
| `../mission/docs/RESUME_2026-08-19.md`, `RESUME_2026-08-21_fixes.md` | state-of-the-world checkpoints |
| `../mission/docs/superpowers/` | the overnight autonomous-work logs, plans and specs |
| `../mission/docs/evidence/` | the ULogs, images and measurements each claim rests on |

## The recordings

[`../flight-data/README.md`](../flight-data/README.md) — what every archived file
is, which runs are SITL and which are real, and where the real flight evidence
lives.
