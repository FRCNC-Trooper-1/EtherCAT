# 05 — Motion Control Architecture

> **Goal of this stage:** a software structure where the real-time path is
> provably bounded, and everything that is not bounded lives somewhere it cannot
> hurt the machine.

---

## 1. The architecture

Three domains, separated by hard boundaries. The separation is the whole design.

```
┌──────────────────────────────────────────────────────────────┐
│  HMI / UI                          non-RT, CPU 0-1           │
│  Operator interface, DRO, program editor, diagnostics        │
│  Separate PROCESS. May crash without stopping the machine.   │
└───────────────────────────┬──────────────────────────────────┘
                            │  IPC (shared memory + socket)
┌───────────────────────────▼──────────────────────────────────┐
│  PLANNER                           non-RT thread, CPU 0-1    │
│  G-code parse → segment queue → look-ahead → feedrate plan    │
│  May allocate. May block. May take milliseconds.             │
└───────────────────────────┬──────────────────────────────────┘
                            │  lock-free SPSC ring buffer
┌───────────────────────────▼──────────────────────────────────┐
│  CYCLIC TASK              SCHED_FIFO 85, isolated CPU 2      │
│  interpolate → CiA 402 state machine → PDO exchange          │
│  NO allocation. NO locks. NO syscalls except the bus.        │
│  Bounded execution time, every cycle, no exceptions.         │
└───────────────────────────┬──────────────────────────────────┘
                            │  AF_PACKET raw socket
                     ┌──────▼──────┐
                     │ EtherCAT bus│
                     └─────────────┘
```

### Why three domains

The cyclic task must complete within its budget **every single cycle**. That is
only provable if everything it does is bounded — which rules out allocation,
locks, file I/O, and anything else with an unbounded tail.

But G-code parsing, look-ahead planning, and drawing a UI are all inherently
unbounded. So they go elsewhere, and the boundary between "bounded" and
"unbounded" is enforced structurally rather than by discipline.

The HMI is a **separate process** specifically so that a UI bug cannot stop the
machine. A crashed HMI should leave the machine running the current program with
motion unaffected.

---

## 2. Rules for the cyclic task

These are absolute. A violation is a latent missed cycle.

| Rule | Why |
|---|---|
| **No `malloc` / `new` / `free` / `delete`** | Allocator can block on a lock or a page fault |
| **No `std::vector` growth, no `std::string`** | They allocate |
| **No exceptions** | Unwinding is unbounded; the table lookup can fault |
| **No locks with the non-RT side** | Priority inversion, unbounded blocking |
| **No file I/O, no `printf`, no logging to disk** | Syscalls block |
| **No `std::chrono::system_clock`** | Can step; use `CLOCK_MONOTONIC` |
| **Absolute sleep only** | Relative sleep accumulates drift |
| **`mlockall(MCL_CURRENT \| MCL_FUTURE)`** | A page fault is a missed cycle |
| **Pre-fault stack and heap at startup** | Touch every page before going real-time |
| **Bounded loops only** | No `while (!done)` without an iteration cap |

### Startup sequence for the RT thread

```cpp
// 1. Lock all memory, current and future
mlockall(MCL_CURRENT | MCL_FUTURE);

// 2. Pre-fault the stack — touch every page you will ever use
volatile char stack[RT_STACK_SIZE];
memset((void*)stack, 0, sizeof(stack));

// 3. Disable heap trimming and mmap for malloc, then pre-fault the heap
mallopt(M_TRIM_THRESHOLD, -1);
mallopt(M_MMAP_MAX, 0);

// 4. Pin to the isolated core
cpu_set_t set; CPU_ZERO(&set); CPU_SET(2, &set);
pthread_setaffinity_np(pthread_self(), sizeof(set), &set);

// 5. SCHED_FIFO
sched_param p{}; p.sched_priority = 85;
pthread_setschedparam(pthread_self(), SCHED_FIFO, &p);

// 6. Only now enter the cyclic loop
```

Do all allocation **before** step 6. After that, the cyclic task owns only memory
it already touched.

---

## 3. Crossing the boundary

The planner produces trajectory segments; the cyclic task consumes them. That
hand-off must never block the cyclic side.

Use a **lock-free single-producer/single-consumer ring buffer** of
fixed-size, trivially-copyable segment descriptors.

```cpp
struct alignas(64) Segment {
    // Trivially copyable. No pointers to heap. No virtuals.
    double  start[3], end[3];       // mm, machine coordinates
    double  feed;                   // mm/s
    double  entry_vel, exit_vel;    // mm/s, set by look-ahead
    uint32_t type;                  // linear / arc
    uint32_t id;                    // for status reporting
    // arc parameters, etc.
};
```

Why SPSC specifically: with exactly one producer and one consumer, a ring buffer
needs only atomic head/tail indices with acquire/release ordering — no
compare-and-swap loop, no retry, no unbounded wait. The cyclic side's enqueue and
dequeue are both wait-free and take a handful of nanoseconds.

**Status flows back the same way**, in the opposite direction: a second SPSC ring
carrying position, state, and fault information to the planner and HMI. The
cyclic task never reads anything the planner might be writing.

**Never** share a `std::mutex` between the two sides. Even an uncontended mutex
can page-fault; a contended one can invert priority.

---

## 4. Trajectory generation

Build this in order. Each stage is testable on its own, and skipping ahead makes
debugging much harder.

### 4.1 Trapezoidal profile

Accelerate at constant `a` to feedrate `v`, cruise, decelerate. Simple, correct,
and adequate to prove the whole pipeline works.

Get this right before making it smooth. A working trapezoid that moves the machine
is worth more than a half-debugged S-curve that does not.

### 4.2 S-curve (jerk-limited) profile

Bound the rate of change of acceleration. Seven phases: jerk up, constant accel,
jerk down, cruise, jerk down, constant decel, jerk up.

Why it matters beyond smoothness: a step in acceleration excites every resonance
in the machine structure. Bounded jerk reduces mechanical wear, cuts audible
noise, and visibly improves surface finish. It is not a luxury feature on an
industrial machine.

### 4.3 Linear interpolation

Coordinate all three axes so they start and finish together, with the tool moving
along a straight line in Cartesian space. Each axis's position is the same
parametric fraction along its own travel.

### 4.4 Circular interpolation

G2/G3 in each plane (G17/G18/G19). Watch two things:

- **Radius error** from finite cycle time — the chord between successive setpoints
  falls inside the true arc. Bound this by limiting feedrate on tight radii.
- **Centripetal acceleration**: `a = v²/r`. On a small radius this exceeds the
  machine's acceleration limit long before the feedrate limit. The planner must
  clamp feed on curvature, or the machine will fault or lose accuracy on every
  small arc.

### 4.5 Look-ahead

A queue of 100–1000 planned segments. Without it, the machine must decelerate to
zero at every block boundary — which on a program with thousands of short moves
means it never reaches commanded feed, and the cut takes many times longer than
it should.

The planner walks the queue backward from the last known stopping point,
computing the maximum velocity each junction can sustain, then forward again
applying acceleration limits. Standard two-pass approach.

### 4.6 Corner blending

At a junction between two segments, the achievable velocity is bounded by how much
deviation from the exact corner you will tolerate. Larger tolerance → higher
corner speed → faster cutting, less accuracy. This is a user-facing parameter
(commonly exposed as a path tolerance / G64 P-value).

---

## 5. Kinematics and units

Keep a single, explicit conversion at the boundary between the motion domain and
the drive domain.

```
machine coordinates (mm)  ←→  axis position (mm)  ←→  drive units (counts)
```

**Do the unit conversion in the master.** As covered in
[`04-drive-cia402.md §6`](04-drive-cia402.md#6-configuration-objects), vendors
differ on whether `0x6091`/`0x6092`/`0x608F` actually scale `0x607A` and `0x6064`.
Setting gear 1:1 and feed constant 1:1 in the drive, and converting in software,
gives you arithmetic you fully control and can unit-test.

Keep the conversion factors in machine configuration, never in code:

```yaml
axes:
  - name: X
    slave: 1
    counts_per_mm: 10000.0
    soft_limit_min_mm: -0.5
    soft_limit_max_mm: 500.0
    max_velocity_mm_s: 500.0
    max_accel_mm_s2: 2000.0
    max_jerk_mm_s3: 20000.0
    following_error_limit_mm: 0.5
```

Changing a ballscrew pitch must not require a recompile.

---

## 6. Fault handling

The cyclic task is the only component that can stop the machine quickly, so it
owns fault detection.

| Fault | Detection | Response |
|---|---|---|
| Working counter mismatch | Every cycle, in the RT loop | Coordinated stop, disable all drives |
| Slave left `OPERATIONAL` | Every cycle | Coordinated stop |
| Drive fault (statusword bit 3) | Every cycle | **Stop all axes**, not just the faulted one |
| Following error over limit | Every cycle, per axis | Coordinated stop |
| Soft limit reached | Planner and RT | Refuse the move / stop |
| Cycle overrun | RT loop self-timing | Log; stop if persistent |
| Planner starvation | Empty segment queue mid-program | Controlled decelerate to stop |
| Process death | signal handler / `std::terminate` | **Disable drives** |

### Coordinated stop

When any axis faults, **every axis must decelerate together along the programmed
path.** An uncoordinated stop — each axis stopping at its own rate — drives the
tool off the path and can break the tool or gouge the part.

Implement it as: freeze the trajectory parameter, then ramp it to zero at the
most restrictive deceleration across all axes.

### Fail-safe on exit

```cpp
// Every abnormal exit path must reach the disable sequence.
std::signal(SIGINT,  emergency_shutdown);
std::signal(SIGTERM, emergency_shutdown);
std::set_terminate(emergency_shutdown_terminate);
```

Test this by killing the process while an axis is moving at speed. If the motor
keeps turning, the design is wrong.

> Remember from [`07-safety-and-compliance.md`](07-safety-and-compliance.md):
> none of this is a substitute for a certified safety circuit. This is the control
> channel doing its job well. The safety channel is separate hardware and is what
> actually protects people.

---

## 7. Configuration

Everything machine-specific lives in declarative configuration, loaded at startup,
validated before the RT thread starts.

```
config/
├── machine.yaml           axes, kinematics, limits
├── drives/
│   ├── delta-asda-a3.yaml PDO mapping, SDO init sequence per drive model
│   └── leadshine-el7.yaml
└── io.yaml                digital I/O assignment
```

Parse and validate in the non-RT domain, then hand the RT task a flat,
fixed-size, fully-resolved struct. **The RT task never parses anything.**

Drive-specific quirks belong in the per-model drive configuration, not in `if`
statements scattered through the code. You will support more than one drive model
eventually.

---

## 8. Testing

| Level | What | Where it runs |
|---|---|---|
| **Unit** | Trajectory math, kinematics, CiA 402 state decoding, ring buffer | CI, no hardware |
| **Simulation** | Full stack against a simulated bus | CI, no hardware |
| **Hardware-in-loop** | Real drives, motors uncoupled, on the bench | Bench |
| **Machine** | Full machine, cutting air, then cutting material | Machine |
| **Endurance** | 30 days continuous | Machine |

The CiA 402 state machine and the trajectory planner are pure functions of their
inputs — test them exhaustively in CI. The statusword decode table in particular
should have a test case for **every** mask/value pair, including the ordering trap
where `Fault Reaction Active` must be tested before `Fault`.

---

## 9. Source layout

```
src/
├── rt/            RT thread setup, timing, memory locking, cycle statistics
├── fieldbus/      SOEM wrapper, slave configuration, DC sync, PDO offset table
├── drive/         CiA 402 state machine, per-model drive configuration
├── motion/        trajectory generation, interpolation, look-ahead, kinematics
├── ipc/           lock-free SPSC rings, status reporting
├── config/        configuration parsing and validation (non-RT)
├── app/           process entry, lifecycle, signal handling
└── tools/         diagnostics: rt_probe, bus scanner, drive explorer
```

The dependency direction is one-way: `app` → everything; `motion` and `drive`
depend on `rt` and `ipc`; **`rt` depends on nothing**. Keeping `rt` dependency-free
is what makes it auditable.

---

**Next:** implement Phase 1 of [`00-roadmap.md`](00-roadmap.md), and use
`tools/rt_probe` to validate the real-time setup before writing any bus code.
