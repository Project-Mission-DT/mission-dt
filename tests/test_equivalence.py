"""Replays the same synthetic telemetry through the original core logic
(core_orig.MissionDT._delta/_lambda/_separation) and through the formal
mission_transition, frame by frame, and compares the states and the actuation u_k^t.
Cases covered: normal telemetry, missing frames (dead reckoning), duplicate
messages, reordered old messages, aerial and surface agents, separation,
agents without telemetry during the first frames (no state, no command).

Pass criteria
  * the agents with a state and the agents with a command are the same in
    both cores in every frame;
  * a state differs only in an agent hit by D5 (a frame whose messages are
    all older than the last applied one: core_orig froze the state, the
    formal model dead-reckons);
  * a command differs only when a state it reads differs (the agent's own
    state or the state of a same-domain agent, read by the separation);
  * without reordering (mode 0) no D5 case occurs and max |dA| is exactly 0.

    python tests/test_equivalence.py [1|0]   # 1 = with reordering (default)
Exit status 0 on pass, 1 on failure. pytest also collects the test_* functions.
"""
import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from mission_dt import model as md
from mission_dt import core_orig as co

N, FRAMES = 12, 600
LATE = {"a10": 16, "a11": 24}     # agents whose telemetry starts at this frame


def _state_eq(rec, B):
    s = rec.state
    return (rec.last_seq == B.seq and (s.lat, s.lon, s.alt) == B.p
            and (s.roll, s.pitch, s.yaw) == B.att and s.t == B.t)


def replay(reorder):
    random.seed(7)
    dom = {f"a{k:02d}": ("aerial" if k % 2 else "surface") for k in range(N)}
    goals = {k: (-30.0577 + random.uniform(-4e-4, 4e-4), -51.1729 + random.uniform(-4e-4, 4e-4),
                 15.0 if dom[k] == "aerial" else 0.0) for k in dom}
    # ground truth: agents on circles crossing the center
    truth = {k: [(-30.0577 + 3e-4 * math.cos(i), -51.1729 + 3e-4 * math.sin(i), 10.0)]
             for i, k in enumerate(dom)}

    old = object.__new__(co.MissionDT)          # original logic without MQTT
    old.frame_s, old.swarm, old.sep_m = 0.125, True, 12.0
    old.stale_updates = old.dup_updates = old.avoid_events = 0
    old.agents = {k: co.AgentRecord(agent_id=k, domain=dom[k], kind="virtual") for k in dom}
    for k in dom:
        old.agents[k].goal = goals[k]

    P = md.Params(separation=True, d_s=12.0)
    HM = md.MissionHistory()
    seq = {k: 0 for k in dom}
    stash = {k: [] for k in dom}                # messages delayed for reordering
    affected = set()
    st = {"frames": FRAMES, "agents": N, "comparisons": 0, "max_dA": 0.0,
          "dA_d5": 0, "dA_state_differs": 0, "no_state_agent_frames": 0,
          "dr": 0, "dup": 0, "old_only": 0, "sep": 0}
    errors = []

    for t in range(FRAMES):
        I = {}
        for idx, k in enumerate(dom):
            lat, lon, alt = truth[k][-1]
            ang = 0.02 * t + idx
            lat += 2e-6 * math.cos(ang); lon += 2e-6 * math.sin(ang); alt += 0.05 * math.sin(ang)
            truth[k].append((lat, lon, alt))
            msgs = []
            r = random.random()
            if r >= 0.10:                                   # r < 0.10: no telemetry -> dead reckoning
                nmsg = 2 if r > 0.93 else 1                 # duplicates in a frame
                for _ in range(nmsg):
                    seq[k] += 1
                    msgs.append({"seq": seq[k], "t_pub": t * 0.125, "gps": [lat, lon, alt],
                                 "att": [0.0, 0.0, ang % 6.28 - 3.14], "vel": [1.0, 0.0, 0.0],
                                 "vb": 18.0})
            if reorder and random.random() < 0.03 and msgs:     # hold one message for a later frame
                stash[k].append(msgs.pop())
            elif stash[k] and random.random() < 0.5:
                msgs = stash[k][:] + msgs if msgs else stash[k][:]
                stash[k].clear()
            I[k] = msgs if t >= LATE.get(k, 0) else []
        # ---- original ----
        recs = list(old.agents.values())
        for rec in recs:
            tel = I[rec.agent_id]
            if tel and max(m["seq"] for m in tel) <= rec.last_seq:
                st["old_only"] += 1
                affected.add(rec.agent_id)
            st["dr"] += not tel
            st["dup"] += len(tel) > 1
            old._delta(rec, tel)
        A_old = {}
        for rec in recs:
            if rec.last_seq < 0:
                continue
            a = old._lambda(rec)
            hit = old._separation(rec, recs)
            if hit is not None:
                nb, dist, away = hit
                err = (away - rec.state.yaw + math.pi) % (2 * math.pi) - math.pi
                a["alpha"] = max(-1.0, min(1.0, 1.5 * err))
                a["tau"] = min(a["tau"], 0.5)
                st["sep"] += 1
            A_old[rec.agent_id] = a
        # ---- formal ----
        Im = {k: tuple(md.Msg.from_payload(m) for m in v) for k, v in I.items()}
        M, A_new, trig = md.mission_transition(HM, Im, goals, dom, P)
        # ---- compare ----
        with_state = {k for k, rec in old.agents.items() if rec.last_seq >= 0}
        st["no_state_agent_frames"] += N - len(with_state)
        if set(M.B) != with_state:
            errors.append(f"t={t}: agents with state differ: old {sorted(with_state)} new {sorted(M.B)}")
            continue
        if set(A_new) != set(A_old):
            errors.append(f"t={t}: agents with a command differ: old {sorted(A_old)} new {sorted(A_new)}")
            continue
        diff_state = {k for k in with_state if not _state_eq(old.agents[k], M.B[k])}
        for k in diff_state - affected:
            errors.append(f"t={t} {k}: state differs outside D5")
        for k in A_old:
            dk = max(abs(A_old[k].get(f, 0.0) - A_new[k].get(f, 0.0))
                     for f in set(A_old[k]) - {"t_pub"} | set(A_new[k]))
            st["comparisons"] += 1
            st["max_dA"] = max(st["max_dA"], dk)
            if dk == 0.0:
                continue
            read = {k} | {j for j in with_state if dom[j] == dom[k]}
            if read & diff_state:
                st["dA_state_differs"] += 1
                st["dA_d5"] += k in affected
            else:
                errors.append(f"t={t} {k}: |dA| = {dk} with identical states")
    return st, sorted(affected), errors


def _check(reorder):
    st, affected, errors = replay(reorder)
    print(f"[reorder={reorder}]", st)
    print(f"[reorder={reorder}] agents hit by D5: {affected}")
    assert st["no_state_agent_frames"] > 0, "the replay must exercise agents without state"
    if reorder:
        assert st["old_only"] > 0, "the replay must exercise the D5 case"
    else:
        assert st["old_only"] == 0 and not affected
        if st["max_dA"] != 0.0:
            errors.append(f"no reordering: max |dA| = {st['max_dA']!r}, expected exactly 0")
    for e in errors[:20]:
        print("FAIL", e)
    assert not errors, f"{len(errors)} disagreement(s) outside D5"
    return st


def test_equivalence_with_reordering():
    _check(1)


def test_equivalence_without_reordering():
    _check(0)


if __name__ == "__main__":
    modes = [int(sys.argv[1])] if len(sys.argv) > 1 else [1]
    try:
        for mode in modes:
            _check(mode)
    except AssertionError as e:
        print("FAILED:", e)
        sys.exit(1)
    print("PASS")
