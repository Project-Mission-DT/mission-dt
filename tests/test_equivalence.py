"""Replays the same synthetic telemetry through the original core logic
(core_orig.MissionDT._delta/_lambda/_separation) and through the formal
Delta_e, frame by frame, and compares the actuation A_k^t.
Cases covered: normal telemetry, missing frames (dead reckoning), duplicate
messages, reordered old messages, aerial and surface agents, separation."""
import math, random, sys
sys.path.insert(0, __import__('os').path.join(__import__('os').path.dirname(__file__), '..'))
from mission_dt import model as md
from mission_dt import core_orig as co

random.seed(7)
REORDER = int(sys.argv[1]) if len(sys.argv) > 1 else 1
N, FRAMES = 12, 600
dom = {f"a{k:02d}": ("aerial" if k % 2 else "surface") for k in range(N)}
goals = {k: (-30.0577 + random.uniform(-4e-4, 4e-4), -51.1729 + random.uniform(-4e-4, 4e-4),
             15.0 if dom[k] == "aerial" else 0.0) for k in dom}
# ground truth: agents on circles crossing the center
truth = {k: [(-30.0577 + 3e-4 * math.cos(i), -51.1729 + 3e-4 * math.sin(i), 10.0)] for i, k in enumerate(dom)}

old = object.__new__(co.MissionDT)          # original logic without MQTT
old.frame_s, old.swarm, old.sep_m = 0.125, True, 12.0
old.stale_updates = old.dup_updates = old.avoid_events = 0
old.agents = {k: co.AgentRecord(agent_id=k, domain=dom[k], kind="virtual") for k in dom}
for k in dom: old.agents[k].goal = goals[k]

P = md.Params(separation=True, d_s=12.0)
HM = md.MissionHistory()
seq = {k: 0 for k in dom}
stash = {k: [] for k in dom}                # messages delayed for reordering
affected=set(); diffs_unaff=0; diffs_aff=0
maxdiff, n_cmp, cases = 0.0, 0, {"dr": 0, "dup": 0, "old_only": 0, "sep": 0}

for t in range(FRAMES):
    I = {}
    for idx, k in enumerate(dom):
        lat, lon, alt = truth[k][-1]
        ang = 0.02 * t + idx
        lat += 2e-6 * math.cos(ang); lon += 2e-6 * math.sin(ang); alt += 0.05 * math.sin(ang)
        truth[k].append((lat, lon, alt))
        msgs = []
        r = random.random()
        if r < 0.10:
            pass                                          # no telemetry -> dead reckoning
        else:
            nmsg = 2 if r > 0.93 else 1                   # duplicates in a frame
            for _ in range(nmsg):
                seq[k] += 1
                msgs.append({"seq": seq[k], "t_pub": t * 0.125, "gps": [lat, lon, alt],
                             "att": [0.0, 0.0, ang % 6.28 - 3.14], "vel": [1.0, 0.0, 0.0], "vb": 18.0})
        if REORDER and random.random() < 0.03 and msgs:               # hold one message for a later frame
            stash[k].append(msgs.pop())
        elif stash[k] and random.random() < 0.5:
            msgs = stash[k][:] + msgs if msgs else stash[k][:]; stash[k].clear()
        I[k] = msgs
    # ---- original ----
    recs = list(old.agents.values())
    for rec in recs:
        before = rec.last_seq
        tel = I[rec.agent_id]
        if tel and max(m["seq"] for m in tel) <= before: cases["old_only"] += 1; affected.add(rec.agent_id)
        if not tel: cases["dr"] += 1
        if len(tel) > 1: cases["dup"] += 1
        old._delta(rec, tel)
    A_old = {}
    for rec in recs:
        if rec.last_seq < 0: continue
        a = old._lambda(rec)
        hit = old._separation(rec, recs)
        if hit is not None:
            nb, dist, away = hit
            err = (away - rec.state.yaw + math.pi) % (2 * math.pi) - math.pi
            a["alpha"] = max(-1.0, min(1.0, 1.5 * err)); a["tau"] = min(a["tau"], 0.5)
            cases["sep"] += 1
        A_old[rec.agent_id] = a
    # ---- formal ----
    Im = {k: tuple(md.Msg.from_payload(m) for m in v) for k, v in I.items()}
    M, A_new, trig = md.Delta_e(HM, Im, goals, dom, P)
    # ---- compare (skip agents frozen by bug D5 in the original) ----
    for k in A_old:
        if I[k] and max(m["seq"] for m in I[k]) <= (HM.H[k][-2].seq if len(HM.H.get(k, ())) >= 2 else -1):
            pass
        for f in ("tau", "alpha", "climb"):
            if f in A_old[k] or f in A_new.get(k, {}):
                d = abs(A_old[k].get(f, 0.0) - A_new[k].get(f, 0.0))
                maxdiff = max(maxdiff, d); n_cmp += 1
                if d > 1e-12:
                    if k in affected: diffs_aff += 1
                    else: diffs_unaff += 1

print("frames", FRAMES, "agents", N, "comparisons", n_cmp, "cases", cases)
print("max |A_old - A_new| =", maxdiff, "| diffs in agents hit by D5:", diffs_aff, "| diffs in other agents:", diffs_unaff, "| agents hit by D5:", sorted(affected))
