"""
Mission-DT formal model (Section II of the paper, revised formalism).

Every function in this module is pure: it receives the state and returns
the next state, with no MQTT, clock or global access. The mission core
(core.py) only moves data between MQTT and Delta_e.

Notation (paper -> code)
  A = {1..N}                      agents (dict keys)
  t, vartheta_t = vartheta_0 + t*T_f  frame index, frame instant
  mu                              Msg (one telemetry message)
  I_k^t                           tuple[Msg, ...]  telemetry of k received in (vartheta_{t-1}, vartheta_t]
  B_k^t                           State
  H_k^t = (B_k^{t-L}..B_k^{t-1})  tuple[State, ...] with length <= L
  delta^e_d                       delta_e(d, H, I, prev, P)
  Phi, phi_k^t, j_k^t             phi_of(B, dom)
  sigma                           P.sigma(g, B, phi)
  lambda_d, lambda^s_d            lam(d, B_k, g_k, P), lam_s(d, B_k, B_j, g_k, P)
  M^t = <B^t, phi^t, g^t>         Mission
  (M^t, A^t) = Delta_e(H_M, I^t, g, P)
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Callable, Mapping, Optional

M_PER_DEG = 111_320.0
AERIAL, SURFACE = "aerial", "surface"


# ----------------------------------------------------------------------
# Types
# ----------------------------------------------------------------------
@dataclass(frozen=True)
class Msg:
    """One telemetry message mu = (s, t_pub, p, att, vel, vb)."""
    seq: int
    t_pub: float
    p: tuple[float, float, float]        # lat, lon, alt
    att: tuple[float, float, float]      # roll, pitch, yaw
    vel: tuple[float, float, float]      # u, v, w
    vb: float

    @staticmethod
    def from_payload(d: dict) -> "Msg":
        return Msg(d["seq"], d["t_pub"], tuple(d["gps"]), tuple(d["att"]),
                   tuple(d["vel"]), d["vb"])


@dataclass(frozen=True)
class State:
    """B_k^t."""
    p: tuple[float, float, float] = (0.0, 0.0, 0.0)
    att: tuple[float, float, float] = (0.0, 0.0, 0.0)
    vel: tuple[float, float, float] = (0.0, 0.0, 0.0)
    vb: float = 0.0
    t: float = 0.0          # t_pub of the message that produced this state
    seq: int = -1           # sequence number of that message (-1 = no message)
    stale: bool = True      # True when delta_e extrapolated (no new telemetry)

    @property
    def seeded(self) -> bool:
        """True for every state in M^t.B (an agent enters B^t with its
        first applied message); False only for the default State() that
        read-only views return for an agent without state."""
        return self.seq >= 0

    @property
    def yaw(self) -> float:
        return self.att[2]


Goal = tuple[float, float, float]        # g_k^t: target lat, lon, alt
Act = dict                               # A_k^t: {"tau", "alpha"[, "climb"]}


@dataclass(frozen=True)
class Phi:
    """phi^t: distance to the nearest same-domain agent and that agent."""
    dist: Mapping[str, float]            # phi_k^t
    nb: Mapping[str, Optional[str]]      # j_k^t


@dataclass(frozen=True)
class Mission:
    """M^t = <B^t, phi^t, g^t> (families indexed by agent id)."""
    B: Mapping[str, State]
    phi: Phi
    g: Mapping[str, Goal]


Sigma = Callable[[object, Mapping[str, State], Phi], Mapping[str, Goal]]


def fixed_waypoints(g_mission: Mapping[str, Goal], B, phi) -> Mapping[str, Goal]:
    """sigma used in E1-E4: the mission goal is one waypoint per agent,
    for the agents that have a state B_k^t."""
    return {k: g_mission[k] for k in B if k in g_mission}


@dataclass(frozen=True)
class Params:
    """P = <(delta^e_d, lambda_d, lambda^s_d)_d, Phi, sigma, d_sep (field d_s), T_f, L>.

    separation=False equals d_sep = 0 (E1, E2, E4).
    """
    T_f: float = 0.125
    L: int = 8
    d_s: float = 12.0
    separation: bool = True
    sigma: Sigma = fixed_waypoints
    arrive_m: float = 1.5


# ----------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------
def horiz_dist(a: State, b: State) -> tuple[float, float, float]:
    """||pi_h(p_a) - pi_h(p_b)|| and its components (dx east, dy north)."""
    dy = (b.p[0] - a.p[0]) * M_PER_DEG
    dx = (b.p[1] - a.p[1]) * M_PER_DEG * math.cos(math.radians(a.p[0]))
    return math.hypot(dx, dy), dx, dy


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def clip(x: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, x))


# ----------------------------------------------------------------------
# delta^e_d : agent transition
# ----------------------------------------------------------------------
def delta_e(d: str, H: tuple[State, ...], I: tuple[Msg, ...],
            prev: Optional[State], P: Params) -> Optional[State]:
    """B_k^t = delta^e_d(H_k^t, I_k^t).

    prev = B_k^{t-1} (the last element of H), or None when agent k has no
    state yet (no message applied).
    Case 1: some message in I has seq > prev.seq (any message when prev is
            None) -> apply the newest one.
    Case 2: prev is None and no message applies -> None: agent k has no
            B_k^t, so it is absent from B^t, Phi and A^t.
    Case 3: otherwise (I empty or only old messages) -> constant-displacement
            extrapolation p^t = p^{t-1} + (p^{t-1} - p^{t-2}); altitude only
            for aerial agents; attitude and velocity kept. With one state in
            H the position is held.
    """
    last = prev.seq if prev is not None else -math.inf
    fresh = [m for m in I if m.seq > last]
    if fresh:
        m = max(fresh, key=lambda x: x.seq)
        return State(p=m.p, att=m.att, vel=m.vel, vb=m.vb, t=m.t_pub,
                     seq=m.seq, stale=False)
    if prev is None:
        return None
    if len(H) >= 2:
        b1, b0 = H[-1], H[-2]
        dp = (b1.p[0] - b0.p[0], b1.p[1] - b0.p[1],
              (b1.p[2] - b0.p[2]) if d == AERIAL else 0.0)
        p = (prev.p[0] + dp[0], prev.p[1] + dp[1], prev.p[2] + dp[2])
        return replace(prev, p=p, stale=True)
    return replace(prev, stale=True)


# ----------------------------------------------------------------------
# Phi : mission context
# ----------------------------------------------------------------------
def phi_of(B: Mapping[str, State], dom: Mapping[str, str]) -> Phi:
    """phi_k^t = min_{j != k, d_j = d_k} ||pi_h(p_k) - pi_h(p_j)||, over the agents in B^t.

    j_k^t is the minimiser; ties go to the lowest agent id (ids are visited in
    sorted order and only a strictly smaller distance replaces the best).
    An agent alone in its domain gets phi_k^t = +inf and j_k^t = None, so
    lambda (not lambda^s) applies.
    """
    dist, nb = {}, {}
    ids = sorted(B)
    for k in ids:
        best, bd = None, math.inf
        for j in ids:
            if j == k or dom[j] != dom[k]:
                continue
            d, _, _ = horiz_dist(B[k], B[j])
            if d < bd:
                best, bd = j, d
        dist[k], nb[k] = bd, best
    return Phi(dist, nb)


# ----------------------------------------------------------------------
# lambda_d and lambda^s_d : decision functions
# ----------------------------------------------------------------------
def lam(d: str, Bk: State, gk: Goal, P: Params) -> Act:
    dist, dx, dy = horiz_dist(Bk, State(p=gk))
    bearing = math.atan2(dx, dy)
    err = wrap(bearing - Bk.yaw)
    a = {"tau": clip(dist / 20.0, 0.0, 1.0) if dist > P.arrive_m else 0.0,
         "alpha": clip(1.2 * err)}
    if d == AERIAL:
        a["climb"] = clip((gk[2] - Bk.p[2]) / 5.0)
    return a


def lam_s(d: str, Bk: State, Bj: State, gk: Goal, P: Params) -> Act:
    a = lam(d, Bk, gk, P)
    _, dx, dy = horiz_dist(Bk, Bj)
    away = math.atan2(-dx, -dy)
    a["alpha"] = clip(1.5 * wrap(away - Bk.yaw))
    a["tau"] = min(a["tau"], 0.5)
    return a


# ----------------------------------------------------------------------
# Delta^e : mission transition
# ----------------------------------------------------------------------
@dataclass
class MissionHistory:
    """H_M^t, stored as the per-agent histories H_k^t (length <= L)."""
    H: dict[str, tuple[State, ...]] = field(default_factory=dict)
    last: dict[str, State] = field(default_factory=dict)


def Delta_e(HM: MissionHistory, I: Mapping[str, tuple[Msg, ...]],
            g_mission, dom: Mapping[str, str], P: Params
            ) -> tuple[Mission, dict[str, Act], dict[str, Optional[str]]]:
    """(M^t, A^t) = Delta^e(H_M^t, I^t, g^t). Order: delta -> Phi -> sigma -> lambda.

    An agent of dom with no state and no applicable message has no B_k^t:
    it is absent from M^t.B, phi^t, g^t and A^t until its first applied message.
    Returns also the trigger neighbor of each separation command (for metrics).
    Updates HM in place with B^t (the only side effect).
    """
    B = {}
    for k in dom:
        b = delta_e(dom[k], HM.H.get(k, ()), I.get(k, ()), HM.last.get(k), P)
        if b is not None:           # an agent without state has no B_k^t
            B[k] = b
    ph = phi_of(B, dom)
    g = P.sigma(g_mission, B, ph)
    A, trig = {}, {}
    for k in B:
        if k not in g:
            continue
        j = ph.nb.get(k)
        if P.separation and j is not None and ph.dist[k] < P.d_s:
            A[k], trig[k] = lam_s(dom[k], B[k], B[j], g[k], P), j
        else:
            A[k], trig[k] = lam(dom[k], B[k], g[k], P), None
    for k, b in B.items():
        HM.last[k] = b
        HM.H[k] = (HM.H.get(k, ()) + (b,))[-P.L:]
    return Mission(B, ph, g), A, trig
