"""Deterministic unit tests of mission_dt/model.py on the desk-check scenario
of Section II (3 agents, 5 frames). Expected values come from the hand
computation of the desk check (phi_k^t with the code's metric, 111320 m/deg
and cos(lat_k); A_k^t = (tau, alpha[, climb]) with the code's lambda_d and
lambda^s_d, d_sep = 12 m).

Agents: 1 aerial, 2 aerial, 3 surface (alone in its domain).
Events: t=2 agent 1 receives two messages (s4, s3) in one frame, reversed;
t=3 agent 2 loses s4 (dead reckoning); t=4 agent 3 receives only s3, older
than the applied s5 (dead reckoning). Two further cases: an agent without
telemetry (no state, no command) and an exact tie in the nearest neighbour.

    python tests/test_model.py        # exit status 0 on pass, 1 on failure
pytest also collects the test_* functions.
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from mission_dt import model as md

AER, SUR = md.AERIAL, md.SURFACE
ATT = (0.0, 0.0, 0.0)              # heading north
ATT3 = (0.0, 0.0, math.pi / 2)     # heading east
V1, V3 = (8.9, 0.0, 0.0), (1.0, 0.0, 0.0)
TOL = 5e-5                         # hand values carry 4 decimals


def m(seq, t, lat, lon, alt=0.0, att=ATT, vel=V1):
    return md.Msg(seq, t, (lat, lon, alt), att, vel, 0.0)


def m3(seq, t, lat, lon):
    return m(seq, t, lat, lon, 0.0, ATT3, V3)


DOM = {1: AER, 2: AER, 3: SUR}
GOALS = {1: (-29.99900, -51.00000, 20.0), 2: (-29.99900, -51.00100, 20.0),
         3: (-30.00100, -50.99900, 0.0)}
INPUTS = [
    {1: (m(1, 0.00, -30.00000, -51.00000, 20.0),),
     2: (m(1, 0.00, -30.00000, -51.00008, 20.0),),
     3: (m3(1, 0.00, -30.00100, -51.00000),)},
    {1: (m(2, 0.12, -29.99999, -51.00000, 20.0),),
     2: (m(2, 0.12, -29.99999, -51.00010, 20.0),),
     3: (m3(2, 0.12, -30.00100, -50.99999),)},
    {1: (m(4, 0.24, -29.99998, -51.00000, 20.0),            # two messages, reversed
         m(3, 0.18, -29.999985, -51.00000, 20.0)),
     2: (m(3, 0.24, -29.99998, -51.00012, 20.0),),
     3: (m3(4, 0.24, -30.00100, -50.99997),)},              # s3 delayed
    {1: (m(5, 0.36, -29.99997, -51.00000, 20.0),),
     2: (),                                                 # s4 lost
     3: (m3(5, 0.36, -30.00100, -50.99996),)},
    {1: (m(6, 0.48, -29.99996, -51.00000, 20.0),),
     2: (m(5, 0.48, -29.99996, -51.00016, 20.0),),
     3: (m3(3, 0.18, -30.00100, -50.99998),)},              # older than s5
]

INF = math.inf
# desk_check.md, table "Desk check table": per frame and agent
# (lat, lon, alt), seq, stale, phi_k^t (code), j_k^t, separation?, A_k^t
EXPECTED = [
    {1: ((-30.00000, -51.00000, 20.0), 1, False, 7.7125, 2, True, (0.5, 1.0, 0.0)),
     2: ((-30.00000, -51.00008, 20.0), 1, False, 7.7125, 1, True, (0.5, -1.0, 0.0)),
     3: ((-30.00100, -51.00000, 0.0), 1, False, INF, None, False, (1.0, 0.0))},
    {1: ((-29.99999, -51.00000, 20.0), 2, False, 9.6406, 2, True, (0.5, 1.0, 0.0)),
     2: ((-29.99999, -51.00010, 20.0), 2, False, 9.6406, 1, True, (0.5, -1.0, 0.0)),
     3: ((-30.00100, -50.99999, 0.0), 2, False, INF, None, False, (1.0, 0.0))},
    {1: ((-29.99998, -51.00000, 20.0), 4, False, 11.5687, 2, True, (0.5, 1.0, 0.0)),
     2: ((-29.99998, -51.00012, 20.0), 3, False, 11.5687, 1, True, (0.5, -1.0, 0.0)),
     3: ((-30.00100, -50.99997, 0.0), 4, False, INF, None, False, (1.0, 0.0))},
    {1: ((-29.99997, -51.00000, 20.0), 5, False, 13.4968, 2, False, (1.0, 0.0, 0.0)),
     2: ((-29.99997, -51.00014, 20.0), 3, True, 13.4968, 1, False, (1.0, -0.7858, 0.0)),
     3: ((-30.00100, -50.99996, 0.0), 5, False, INF, None, False, (1.0, 0.0))},
    {1: ((-29.99996, -51.00000, 20.0), 6, False, 15.4250, 2, False, (1.0, 0.0, 0.0)),
     2: ((-29.99996, -51.00016, 20.0), 5, False, 15.4250, 1, False, (1.0, -0.7781, 0.0)),
     3: ((-30.00100, -50.99995, 0.0), 5, True, INF, None, False, (1.0, 0.0))},
]


def _act(a, d):
    return (a["tau"], a["alpha"], a["climb"]) if d == AER else (a["tau"], a["alpha"])


def _close(x, y, tol):
    return all(abs(a - b) <= tol for a, b in zip(x, y)) and len(x) == len(y)


def _run(inputs, dom, goals):
    P = md.Params(separation=True, d_s=12.0)
    HM = md.MissionHistory()
    return [md.Delta_e(HM, I, goals, dom, P) for I in inputs], HM


def test_desk_check_scenario():
    out, HM = _run(INPUTS, DOM, GOALS)
    for t, ((M, A, trig), exp) in enumerate(zip(out, EXPECTED)):
        assert sorted(M.B) == [1, 2, 3] and sorted(A) == [1, 2, 3], t
        assert dict(M.g) == GOALS, t                       # sigma = identity
        for k, (p, seq, stale, phi, j, sep, a) in exp.items():
            B = M.B[k]
            assert _close(B.p, p, 1e-9), (t, k, B.p, p)
            assert (B.seq, B.stale) == (seq, stale), (t, k, B.seq, B.stale)
            if math.isinf(phi):                           # alone in its domain
                assert math.isinf(M.phi.dist[k]) and M.phi.dist[k] > 0, (t, k)
            else:
                assert abs(M.phi.dist[k] - phi) <= TOL, (t, k, M.phi.dist[k], phi)
            assert M.phi.nb[k] == j, (t, k, M.phi.nb[k], j)
            assert (trig[k] is not None) == sep, (t, k, trig[k])   # lambda^s vs lambda
            assert _close(_act(A[k], DOM[k]), a, TOL), (t, k, A[k], a)
            assert set(A[k]) == ({"tau", "alpha", "climb"} if DOM[k] == AER else {"tau", "alpha"})
    assert len(HM.H[1]) == 5 and HM.H[1][-1] is out[-1][0].B[1]


def test_dead_reckoning_arithmetic():
    """Eq. (2) at t=3 (agent 2, I empty) and t=4 (agent 3, only an older message)."""
    out, _ = _run(INPUTS, DOM, GOALS)
    B2 = out[3][0].B[2]
    assert B2.p == (-29.99998 + (-29.99998 - -29.99999), -51.00012 + (-51.00012 - -51.00010), 20.0)
    assert (B2.t, B2.att, B2.vel) == (0.24, ATT, V1)       # attitude, velocity kept
    B3 = out[4][0].B[3]
    assert B3.p == (-30.00100, -50.99996 + (-50.99996 - -50.99997), 0.0)
    assert B3.seq == 5 and B3.t == 0.36                    # the older s3 is not applied


def test_no_state_before_first_message():
    """An agent with no state and an empty I_k^t has no B_k^t and receives no
    command: agent 2 is registered at t=0 and sends its first message at t=2."""
    I = [{1: (m(1, 0.0, -30.00000, -51.0, 20.0),)},
         {1: (m(2, 0.1, -29.99999, -51.0, 20.0),), 2: ()},
         {1: (m(3, 0.2, -29.99998, -51.0, 20.0),), 2: (m(1, 0.2, -29.99998, -51.00005, 20.0),)}]
    dom = {1: AER, 2: AER}
    goals = {1: (-29.999, -51.0, 20.0), 2: (-29.999, -51.0, 20.0)}
    P = md.Params(separation=True, d_s=12.0)
    HM = md.MissionHistory()
    for t in (0, 1):
        M, A, trig = md.Delta_e(HM, I[t], goals, dom, P)
        assert list(M.B) == [1] and list(M.phi.dist) == [1] and list(M.g) == [1], t
        assert list(A) == [1] and 2 not in HM.last and 2 not in HM.H, t
        assert math.isinf(M.phi.dist[1]) and M.phi.nb[1] is None, t   # 2 is not a neighbour
        assert _close(_act(A[1], AER), (1.0, 0.0, 0.0), 1e-12), (t, A[1])
    M, A, trig = md.Delta_e(HM, I[2], goals, dom, P)
    assert sorted(M.B) == [1, 2] and M.B[2].seq == 1 and not M.B[2].stale
    assert abs(M.phi.dist[1] - 4.8203) <= TOL and (M.phi.nb[1], M.phi.nb[2]) == (2, 1)
    assert _close(_act(A[1], AER), (0.5, 1.0, 0.0), 1e-12), A[1]
    assert _close(_act(A[2], AER), (0.5, -1.0, 0.0), 1e-12), A[2]
    assert len(HM.H[2]) == 1
    assert md.delta_e(AER, (), (), None, P) is None


def test_tie_goes_to_lowest_id():
    """Agent 1 exactly equidistant from 2 (north) and 3 (south), all aerial;
    registration order 1, 3, 2. j_1 = 2 (lowest id), A_1 = (0.5, -1.0, 0.0)."""
    for order in ((1, 3, 2), (3, 2, 1), (1, 2, 3)):
        dom = {k: AER for k in order}
        I = {1: (m(1, 0.0, -30.0, -51.0, 20.0),),
             3: (m(1, 0.0, -30.00005, -51.0, 20.0),),
             2: (m(1, 0.0, -29.99995, -51.0, 20.0),)}
        goals = {k: (-29.999, -51.0, 20.0) for k in (1, 2, 3)}
        (M, A, trig), = _run([I], dom, goals)[0]
        d12 = md.horiz_dist(M.B[1], M.B[2])[0]
        assert d12 == md.horiz_dist(M.B[1], M.B[3])[0]      # exact tie
        assert abs(d12 - 5.5660) <= TOL
        assert M.phi.nb[1] == 2 and trig[1] == 2, (order, M.phi.nb[1])
        assert _close(_act(A[1], AER), (0.5, -1.0, 0.0), 1e-12), (order, A[1])
        assert (M.phi.nb[2], M.phi.nb[3]) == (1, 1)
        assert _close(_act(A[2], AER), (0.5, 0.0, 0.0), 1e-12), (order, A[2])
        assert _close(_act(A[3], AER), (0.5, -1.0, 0.0), 1e-12), (order, A[3])


if __name__ == "__main__":
    tests = [v for n, v in sorted(globals().items()) if n.startswith("test_") and callable(v)]
    failed = 0
    for f in tests:
        try:
            f()
            print("PASS", f.__name__)
        except AssertionError as e:
            failed += 1
            print("FAIL", f.__name__, e)
    sys.exit(1 if failed else 0)
