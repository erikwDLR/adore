# Obstacle Avoidance — Clean Target Design

**Status:** design reference (2026-07-06). Not yet implemented.
**Purpose:** the single reference the OA rebuild converges to. It describes the
*target* — a completely clean, locally-traceable implementation — and the migration
from today's code toward it.

## How to use this document

1. Every change must move the code **toward** this document, never away.
2. Each stage is **subtractive/replacing**, never additive. If a change adds a new
   compensating mechanism instead of replacing one, it is wrong.
3. Half-migrated is worse than either end. Stages are PR-sized and each one leaves
   the tree building + fail-safe correct.
4. When in doubt about behaviour, this document is the intended behaviour; the code
   is what we are correcting toward it.

---

## 1. Invariants (non-negotiable, already true today — keep)

- **Route-only.** The node never emits a free-space/emergency trajectory. Ego always
  follows either the mission route or the active modified route.
- **Fail-safe.** Every failure path (planner exception, empty trajectory, invalid
  projection, impossible avoidance) ends in *braking on the route ego is driving*.
- **Monotonic progress.** Ego's arc-length on the driven route never goes backward;
  implausible forward jumps are bounded by odometry.
- **Physics-sized maneuver.** Entry-ramp length and maneuver speed are coupled by
  `v = ramp·sqrt(a/(6·|shift|))` so a late/tight shift stays drivable.
- **Opposite-lane overtaking is supported** (the real vehicle uses it): the oncoming
  subsystem stays.

## 2. The model (three steps, made precise)

LIDAR-only perception never reports true object extents: a real static object arrives
as **several fragments**, each possibly its own tracking-id, and **more fragments
become visible as ego approaches / passes**. Therefore the model is **id-independent**
and **growth-aware**:

1. **Detect.** Any detection with `speed < static_speed` whose footprint intrudes the
   ego corridor **ahead of the front bumper** is a static obstacle (or a fragment of
   one). No confirm/debounce gate — every real detection is acted on. Fragments within
   a `group_link_gap` are one obstacle.
2. **Decide.** Is a lateral avoidance allowed here (drivable area / lane topology /
   oncoming gap)? **Yes →** build the modified (laterally shifted) route. **No →** stop
   before the obstacle on the route.
3. **Commit, drive, replan from the modified route.** Once planned, the modified route
   *is the object model*. Drive it. As more of the object becomes visible (a fragment
   pokes into the **driven** corridor **beyond the already-cleared band**), **widen**
   the shift — replanned **from the modified route** (never the mission centerline, so
   no snap), **id-independent**, and **monotonic (never narrower mid-maneuver)**.
4. **Release.** When ego has passed the obstacle (`ego_s ≥ release_s`), return to the
   mission route.

The maneuver never snaps laterally: a lost/re-id'd fragment is a perception artefact
of a *real, static* object, so the shift is held to `release_s`, not abandoned.

---

## 3. Layered architecture (what stays / what changes)

| Layer | Today | Target |
|---|---|---|
| **P — Planner primitives** (geometry, projection, grouping, shift, drivable, oncoming) | clean, pure, testable | **stays**; only parameter names decoupled (§4) + one oncoming-gap function (§9) |
| **S — Shift planning** (`try_plan_obstacle_avoidance`) | candidate gen/validate/score + **redundant** final re-validation | straighten: drop redundant re-validation, single clearance concept |
| **L — Lifecycle** (`continue_active_avoidance` + `ActiveAvoidanceState`) | 1886-line flag-soup, 2–3 corridor passes, id-based bookkeeping | **explicit state machine** (§5), one corridor pass (§7), spatial cleared-band (§8) |
| **I — Input view** | raw `traffic_participants` (id-keyed) | id-independent obstacle view derived spatially (§8) |

The geometry pipeline (Layer P, ~6000 lines) is **not** the problem and is **not**
rewritten. The rework is Layers L and I plus the parameter model.

## 4. De-overload `front_clearance` / `rear_clearance`

An inventory of **every** use showed `front_clearance`/`rear_clearance` are ONE
coherent concept — the shift entry/exit ramp (and its timing-band mirror) — in ~90% of
uses (shift, grouping, candidate validation, oncoming fallback, node look-ahead). That
is **not** overloading; it stays as-is. (A cosmetic rename to
`shift_entry_ramp`/`shift_exit_ramp` with deprecated yaml aliases is optional = C1b,
deferred.) Only **two** uses genuinely mean something else and are peeled off:

| Peeled use (today) | What it really is | Target |
|---|---|---|
| `rear_clearance` at `projection.cpp:189` — keeps static obstacles up to 7 m **behind** ego | detection rear-retention | **removed.** Only obstacles ahead matter; reject anything with `object_s_max < ego_s`. No param. |
| `front_clearance` at `obstacle_avoidance.cpp:1383` — predictive corridor look-ahead cushion for **moving** objects | longitudinal safety margin for dynamic-conflict prediction | `corridor_detect_margin` — internal constant (not yaml), **2.5 m**, decoupled from the ramp. Also fixes the plan-vs-active inconsistency (front_clearance was overwritten to the sized ramp per candidate). |

**The stop stand-off is NOT a mis-use — it encodes a real intent** (two distinct stops):
- **Pre-shift stop** (neighbour-lane use not yet approved — a later AI model decides
  whether the object is permanently or only temporarily stopped): ego must stop
  **before `shift_start_s`** so a shift is still executable from standstill once
  approved. Stand-off `= max(stop_before_obstacle, front_clearance + stop_adjustment_offset)`
  in `normalized_stop_before_obstacle`. Correct, keep.
- **During-avoidance stop** (a new conflict mid-maneuver): may stop closer, keeping a
  safety margin. Uses the raw `stop_before_obstacle`.

The real bug was the **global** load-time mutation in `decision_maker.cpp:137` that
floored `stop_before_obstacle` for *both* stops, so the during-avoidance stop could
never be closer than the pre-shift one. **Fix: remove the global mutation; keep the
floor local to the pre-shift path.** `stop_adjustment_offset` is the pre-shift safety
margin (NOT dead). Target clean-up: name the two stands-off explicitly (during-avoidance
= `stop_before_obstacle`; pre-shift = derived from the shift geometry).

`side_clearance` was already single-purpose. **Net: fewer parameters, no new yaml
knobs.** Two verification tiers: **C1-neutral** (rear-retention removal + remove the
global stop mutation — behavior-neutral for the current yaml, static-verifiable) and
**C1-fix** (`corridor_detect_margin` = 2.5 m — a deliberate change to dynamic-conflict
prediction → sim-tested).

## 5. Lifecycle state machine (Layer L)

Replace the flag-soup (`committed`, `oncoming_wait_active`, `group_shrink_since_time`,
`tracked_obstacles`) with an **explicit state** and named transitions. One function per
state; transitions are guarded and logged.

```
        ┌──────┐  static obstacle ahead        ┌──────────┐
        │ Idle │ ────────────────────────────▶ │ Planning │
        └──────┘                                └────┬─────┘
           ▲                                valid shift │ no shift / oncoming-reject
           │ passed (ego_s ≥ release_s)                 ▼             ▼
        ┌──────────┐   widen (fragment beyond band)  ┌─────────────┐ ┌──────────┐
        │Releasing │◀───┐   ┌────────────────────────│ ActiveShift │ │ Stopping │
        └────┬─────┘    │   │                         └──┬───┬──────┘ └────┬─────┘
             │          └───┘ (self, monotonic widen)    │   │             │
             └──────────▶ Idle          oncoming approaching│   │ can't avoid │ obstacle
                                          (before opp lane) │   │  (alongside)│  gone
                                                            ▼   ▼             ▼
                                                    ┌──────────────┐      (Idle)
                                                    │ OncomingHold │
                                                    └──────┬───────┘
                                                  oncoming cleared → ActiveShift
```

- **Idle** — no maneuver. Plan the normal mission; run detection each cycle.
- **Planning** — transient (one cycle): obstacle detected, a maneuver is being planned.
- **ActiveShift** — committed; drive the modified route; monitor for growth/oncoming.
- **OncomingHold** — stopped, pulled up at the obstacle, waiting for an oncoming
  vehicle to clear the opposite-lane conflict interval (latched; no per-cycle re-decide).
- **Stopping** — avoidance impossible (fragment alongside / no valid shift); brake to a
  stop on the driven route.
- **Releasing** — transient: ego passed the obstacle; return to the mission route.

Transitions (each guarded, single source of truth):

| From → To | Guard |
|---|---|
| Idle → Planning | a static detection intrudes the corridor ahead of the front bumper |
| Planning → ActiveShift | a validated shift exists (commit immediately) |
| Planning → Stopping | no validated shift, or oncoming gap rejected |
| ActiveShift → ActiveShift (widen) | a detection intrudes the driven corridor **beyond the cleared band**, ahead of the front bumper → replan wider from the modified route (monotonic) |
| ActiveShift → OncomingHold | opposite-lane monitor: an oncoming vehicle will arrive before ego clears, and ego has **not yet entered** the opposite lane |
| ActiveShift → Stopping | driven-corridor intrusion that cannot be widened away (alongside / too close) |
| OncomingHold → ActiveShift | the oncoming vehicle has cleared the conflict interval / vanished / stopped with clearance |
| ActiveShift → Releasing | `ego_s ≥ release_s` on the driven route |
| Releasing → Idle | mission trajectory re-planned |
| any → Stopping | any planning failure (fail-safe) |

## 6. State data (reduced, invariants central)

```
struct ActiveAvoidance {
  Mode         mode;              // Idle/ActiveShift/OncomingHold/Stopping/Releasing
  Route        modified_route;    // the object model + geometry (drive this)
  Bounds       bounds;            // shift_start_s, release_s, obstacle_s_min,
                                  //   lateral_shift, uses_opposite_lane, commitment_s
  ClearedBand  cleared_band;      // §8 — REPLACES obstacle_ids / tracked_obstacles /
                                  //   ignore-lists as the maneuver-membership key
  MonotonicS   progress;          // last_modified_s / last_modified_time (keep)
  OncomingHold hold;              // participant ref + release edge (OncomingHold only)
};
```

**Dropped from today:** `obstacle_ids` as the membership key (kept diagnostic only),
`group_shrink_since_time` (pre-commit path is dead on the commit-immediately model),
`tracked_obstacles` max-hull (subsumed by `cleared_band` + the growth check), the
free-standing `committed` bool (becomes `mode != Idle`).

## 7. One corridor pass per cycle (Layer L)

A **single** `check_route_corridor_safety` on the driven route each cycle, id-independent,
returning **all** conflicts. The lifecycle classifies each:

- inside `cleared_band` → part of my object → ignore.
- beyond `cleared_band`, ahead of the front bumper → **widen** (replan from modified route).
- beyond `cleared_band`, alongside / too close to widen → **Stopping**.
- oncoming in the opposite lane → **OncomingHold**.

This replaces today's 2–3 passes (active_route_safety with id-ignore + guard growth_safety
without ignore + opposite monitor) and all the id-based ignore/skip lists.

## 8. Id-independence: the cleared band (Layer I + L)

`cleared_band` = the lateral corridor the modified route clears (the applied shift plus
`side_clearance`) over `[shift_start_s, release_s]`. A detection is **"mine"** iff its
route-frame footprint lies within `cleared_band + corridor_detect_margin`. **No
tracking-id appears anywhere in the maneuver-membership decision.**

Reuse existing math — no new geometry:
- `required_signed_shift_for_obstacle` (`shift.cpp`) = `object_l_max + side_clearance + ego_half_width`.
- `modified_route_clears_group_obstacles` (`candidates.cpp`) already answers "does the
  shift clear this footprint by `side_clearance`?" — generalise it to "is this footprint
  inside the cleared band?".

Growth is **monotonic + debounced on the shift magnitude** (not on detection presence):
a fragment that pokes out only widens the shift and is held; it never shrinks the shift
mid-maneuver, so fragment flicker cannot oscillate the path.

## 9. Oncoming (Layer P) — de-duplicate

`check_oncoming_gap` (plan-time) and `monitor_active_obstacle_avoidance_maneuver`
(active-time) today duplicate the slow/stopped-in-interval, slow-approaching, and
moving-arrival-vs-clear-time logic. Target: **one** gap-acceptance function called from
both, parameterised by the trajectory/hint. Behaviour unchanged; one source of truth.

## 10. Migration map (stage by stage)

| Stage | Content | Sections | Status |
|---|---|---|---|
| 0 | remove dead pre-commit block + orphaned stop builder | — | ✅ done, built green |
| C1 | split `front_clearance`/`rear_clearance` into named concepts | §4 | next (recommended first) |
| 1 | `cleared_band` replaces id-lists; monotonic widen | §6, §8 | designed |
| 2 | one corridor pass + drop redundant final re-validation | §7, §3(S) | designed |
| L | extract the explicit state machine | §5, §6 | designed |
| 3 | param diet + oncoming-gap unification | §4, §9 | designed |

Order rationale: **C1 first** — decoupling the parameters shrinks the blast radius of
every later stage (the shift ramp no longer shares a knob with the corridor margin), so
Stage 1/2/L become smaller and safer to build.

## 11. Open questions / pending decisions

- **Branch convergence.** `feature/oa-fix-static` and `feature/oa-tweaks` pin divergent
  `decision_maker` commits. The clean target should converge them; decide when.
- **Two "committed" notions** (audit C4). Unify under the geometric `commitment_s` as
  the single source; the free-standing `committed` bool disappears with the state enum.
  Cosmetic today (the opposite-lane pre-commit abort already uses `commitment_s`), but
  the rework removes the ambiguity.
- **`stop_before_obstacle` (audit C5 — REVISED):** the `front_clearance` coupling is
  intentional for the PRE-SHIFT stop (stop before `shift_start_s` so a shift stays
  possible while waiting for lane approval), see §4. The actual defect was the global
  load-time mutation coupling the during-avoidance stop too; fixed by keeping the
  floor local to the pre-shift path. `stop_adjustment_offset` is the pre-shift margin.
- **~12 of ~60 params are effectively constants** — fold into internal constants in
  Stage 3 (list in the hardening memo).
- **No detection debounce** (Erik: every detection is a real object) — robustness comes
  from the spatial cleared-band + monotonic widen, not from suppressing detections.

## 12. What explicitly stays (do not touch)

Geometry / projection / grouping / shift / drivable-area primitives; the fail-safe
braking cascade; the route-only invariant; physics-sized ramp/speed; the oncoming
subsystem (opposite-lane overtaking is used on the real vehicle).
