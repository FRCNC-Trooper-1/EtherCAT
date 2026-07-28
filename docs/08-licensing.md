# 08 — Licensing and Intellectual Property

> **Status: this document corrects a widely repeated error.** SOEM is frequently
> described online — and was described earlier in this project's own planning — as
> BSD or permissively licensed. **It is not.** Every claim below was verified by
> reading the license files in the SOEM repository directly, at the commits named.
>
> Read this before writing code you intend to sell.

---

## 1. Verified facts

### SOEM v2.0.0 — GPLv3 or commercial

Verified at commit `2f73eaa803f91f8332b5c8b047ba03a1210c9a80` (2026-06-11),
`project(SOEM VERSION 2.0.0)`. `LICENSE.md`, quoted verbatim:

> This software is dual-licensed.
>
> ## GPL version 3
>
> This software is distributed under GPLv3. You are allowed to use this
> software for an open-source project with a compatible license.
>
> ## Commercial license
>
> This software is also available under a commercial license with
> options for support and maintenance. Please contact sales@rt-labs.com
> for further details.
>
> **If you intend to use this stack in a commercial product, you likely need to
> buy a license.**

There is **no linking exception**. Every source and header file carries the
banner `This software is dual-licensed under GPLv3 and a commercial license.`

### SOEM v1.4.0 — GPLv2 *with* a linking exception

Verified via `git show v1.4.0:LICENSE`. The operative paragraph:

> **As a special exception**, if other files instantiate templates or use macros
> or inline functions from this file, or you compile this file and link it with
> other works to produce a work based on this file, this file does not by itself
> cause the resulting work to be covered by the GNU General Public License.
> **However the source code for this file must still be made available** in
> accordance with section (3) of the GNU General Public License.

This exception **does** permit linking proprietary application code against SOEM
v1.4.0. You must still make **SOEM's own source** available — but not your
application's.

### Both versions — Beckhoff EtherCAT Master License required

The v1.4.0 LICENSE states this explicitly:

> The EtherCAT Technology, the trade name and logo "EtherCAT" are the
> intellectual property of, and protected by Beckhoff Automation GmbH. You can
> use SOEM for the sole purpose of creating, using and/or selling or otherwise
> distributing an EtherCAT network master **provided that an EtherCAT Master
> License is obtained from Beckhoff Automation GmbH.**

This is a **patent and trademark** obligation, entirely separate from copyright.
It applies **no matter which master stack you use** — SOEM, IgH, acontis, or one
you write yourself. Writing your own master does not avoid it.

**Action:** join the [EtherCAT Technology Group](https://www.ethercat.org)
(membership is free) and obtain the EtherCAT Master License. Do this early; it is
paperwork, not money, and it is a prerequisite for shipping.

---

## 2. What this means for a proprietary product

The critical distinction in every GPL analysis:

> **GPL obligations trigger on *distribution*, not on *use*.**

Building, testing, and running SOEM v2 on your own bench — even for years, even
commercially, even on a machine you use in your own shop to make parts you sell —
triggers **nothing**. You owe no source to anyone.

The obligation attaches the moment you **ship a binary to a customer**.

This has a direct practical consequence, and it is the single most useful thing in
this document:

> **You can start developing today, on current SOEM v2, under GPLv3, and defer the
> licensing decision until shortly before you ship your first machine.**

Development is not distribution. Do not let this question block the project — but
do not forget it either, because the decision must be made before unit #1 leaves
the building. Put it on the schedule.

---

## 3. Your options, ranked

### Option A — Buy a commercial SOEM license from rt-labs ✅ *recommended*

Contact `sales@rt-labs.com`.

| | |
|---|---|
| **Cost** | Not published; request a quote. Comparable products are low-single-digit thousands EUR. |
| **You get** | Current v2 code, support, maintenance, and a counterparty with contractual responsibility |
| **Your code** | Fully proprietary. Ship binaries, publish nothing. |
| **Risk** | Lowest. A vendor relationship instead of a legal opinion. |

For a product where a licensing mistake means recalling machines or open-sourcing
your differentiator, a few thousand euros is not a meaningful cost. **This is the
right answer for most commercial machine builders**, and it also buys you someone
to call when the bus misbehaves at 2 a.m. before a customer acceptance test.

### Option B — SOEM v1.4.0 under the GPLv2 linking exception

| | |
|---|---|
| **Cost** | Free |
| **Your code** | Stays proprietary — the linking exception permits it |
| **You must** | Publish SOEM v1.4.0's source (trivial — it is unmodified upstream; a link and a tarball satisfies it) |
| **Risk** | v1.4.0 is frozen. No upstream fixes, no security patches, and the entire v2 API redesign is unavailable to you. |

Legally the cleanest free path. Technically it means adopting an abandoned
codebase for a product with a ten-year service life. The v1→v2 changes are
substantial (see §5), so this is a fork you own forever.

Viable, but understand that you are choosing a maintenance burden over a licence fee.

### Option C — IgH EtherCAT Master via its LGPL userspace library

The IgH master is a **GPLv2 kernel module**, but the userspace application library
is **LGPLv2.1**, which permits proprietary applications to link against it.

| | |
|---|---|
| **Cost** | Free |
| **Your code** | Can stay proprietary (LGPL permits linking, subject to relink/replacement conditions) |
| **Performance** | Best in class — kernel-context execution, reliable at 250 µs and below |
| **Risk** | Kernel module maintenance across kernel upgrades; you must satisfy LGPL §6 (allow users to relink against a modified library). **Get a lawyer's opinion on your specific linkage before relying on this.** |

Technically excellent, legally more nuanced than Option A. The
[`linuxcnc-ethercat/ethercat`](https://github.com/linuxcnc-ethercat/ethercat)
fork is the right source if you go this way, because upstream lags on kernel support.

### Option D — acontis EC-Master or another commercial stack

| | |
|---|---|
| **Cost** | ~€3,000–8,000, plus per-unit royalties in some models |
| **You get** | Conformance-tested, supported, indemnified, proprietary-friendly |
| **Risk** | Lowest of all, highest cost |

Worth quoting alongside Option A so you have a comparison.

### Option E — Write your own master ❌ *not recommended*

Covered previously: roughly 2–3 years with 1–2 engineers to reach production
quality, dominated by Distributed Clocks, CoE mailbox handling, and multi-vendor
interoperability. **It does not avoid the Beckhoff license requirement.** You
would be spending years to arrive where a €3,000 licence puts you in a week.

---

## 4. What is *not* a problem

Some things that look like licensing risk and are not:

| Component | License | Why it's fine |
|---|---|---|
| **Linux kernel** | GPLv2 | The syscall exception explicitly carves out userspace. An application calling `socket()`, `clock_nanosleep()`, and `sched_setscheduler()` is not a derivative work. |
| **glibc** | LGPL | Dynamic linking is expressly permitted |
| **Your own code** | BSD-3-Clause | Your choice; BSD is GPL-compatible |

The kernel point is why the **userspace + raw socket** architecture is
structurally favourable: the only copyleft boundary you must actively manage is
the EtherCAT stack itself. Get that one right and everything else follows.

> **Note on your own BSD license:** BSD is GPL-*compatible*, which means you may
> combine BSD and GPL code — but the **combined work must then be distributed
> under the GPL**. Putting a BSD header on your source files does not shield you
> from GPLv3 obligations incurred by linking GPLv3 SOEM. Your source stays BSD;
> the shipped binary would still be GPLv3. This is the trap that catches people.

---

## 5. Technical consequence: SOEM v1 and v2 are different libraries

If licence considerations push you to v1.4.0, understand what you give up. These
differences were established by diffing `v1.4.0` against `master` — there is no
upstream migration guide.

1. **The entire global API is gone in v2.** No `ec_init`, `ec_slave[]`,
   `ec_slavecount`, `ec_DCtime`, `EcatError`. Everything is `ecx_*` with an
   explicit `ecx_contextt *`.
2. **Context is by value, not by pointer.** v2 embeds all storage in the struct
   (and it is large — static or heap allocate it, never a stack local).
3. **Headers relocated**: `ethercat.h` → `soem/soem.h`; `soem/ethercat*.h` →
   `include/soem/ec_*.h`.
4. **New mailbox architecture** — a mailbox pool plus per-group queue, drained by
   `ecx_mbxhandler()` from your cyclic thread. This is what makes SDO access safe
   while the bus is in OP. It is a genuine architectural improvement and it does
   not exist in v1.
5. **ENI (EtherCAT Network Information) file support** is new in v2.
6. `ecx_config_init` lost its `usetable` parameter; overlap mapping moved to a
   context flag; tuning constants moved from headers into CMake cache variables.
7. **Platform support narrowed** to Linux, Windows, rt-kernel. macOS was dropped.

The v2 mailbox handling in particular matters for a CNC, where you want to read
drive diagnostics over SDO without disturbing a running cyclic exchange.

---

## 6. Decision record

| Question | Answer | Decide by |
|---|---|---|
| Which EtherCAT stack? | **Develop on SOEM v2 now; license decision deferred** | Before first customer shipment |
| Licensing route | Request quotes from rt-labs **and** acontis; compare against the v1.4.0 fork burden | Before first customer shipment |
| Beckhoff EtherCAT Master License | **Required regardless of stack** | Start the paperwork now |
| ETG membership | Free; prerequisite for the above | Now |
| Our own code license | BSD-3-Clause | Done |

**Two things to action this week**, both paperwork, neither blocking development:

1. Join the EtherCAT Technology Group and start the Master License process.
2. Email `sales@rt-labs.com` for a SOEM commercial quote, and acontis for an
   EC-Master quote. Knowing the real numbers turns this from an open question
   into a line item.

---

## 7. Compliance checklist before shipping unit #1

- [ ] EtherCAT Master License obtained from Beckhoff
- [ ] ETG membership active
- [ ] EtherCAT stack licensing resolved (commercial licence purchased, or v1.4.0
      source-availability obligation satisfied)
- [ ] No GPL code linked into the shipped binary — verified by automated
      dependency-license scan in CI, not by memory
- [ ] `THIRD_PARTY_LICENSES.md` generated from the real dependency tree and
      shipped with the machine
- [ ] SBOM (SPDX or CycloneDX) generated per release
- [ ] Trademark usage reviewed — "EtherCAT" is a Beckhoff trademark and its use in
      your marketing materials is governed by the ETG rules

---

## 8. Standing rule for this repository

> **Every new dependency gets a license check before it is merged.**

Add the license to `THIRD_PARTY_LICENSES.md` in the same pull request that adds
the dependency. A CI job should fail the build if a copyleft license appears in
the dependency tree without an explicit, documented exemption.

One convenience library pulled in without checking can compromise the licensing
position of the entire product — and it is far cheaper to catch at review time
than during a pre-shipment audit.

---

*Nothing in this document is legal advice. It records what the license files say
and what that appears to mean. Before shipping, have a lawyer with software
licensing experience review your actual linkage and distribution model.*
