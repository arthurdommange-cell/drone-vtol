
import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAS_PLT = True
except ImportError:
    HAS_PLT = False

# ----------------------------------------------------------------------
# 1. PARAMETRES
# ----------------------------------------------------------------------
g, rho = 9.81, 1.225
m = 1.10                       # masse de dimensionnement [kg]       (Tableau II.1)
W = m * g
S = 0.22                       # surface alaire [m²]                 (Tableau II.1)
l, h = 0.55, 0.07              # demi-distance rotors, hauteur axe/CG [m]
P0 = W / 2                     # poussée par moteur en stationnaire [N]
T_max = 1.65 * g               # poussée max par moteur [N]         (SunnySky, [19])
T_u = 0.85 * T_max             # poussée utilisable                  (II.1.5)
CL_MAX = 1.1                   # hypothèse du rapport
It = 8e-5                      # inertie d'une nacelle / axe de basculement [kg·m²] (II.2.2)

# --- HYPOTHESES A REMPLACER (CAO, balancier, banc de poussée) ---
Jx, Jy, Jz = 0.08, 0.03, 0.11  # inerties du drone [kg·m²]
TAU_M = 0.050                  # constante de temps moteur+ESC+hélice [s]
TAU_S = 0.030                  # constante de temps servo de basculement [s]
T_DELAY = 0.006                # retard de calcul + filtrage [s]
CD = 0.08                      # coefficient de traînée global
FS = 250.0                     # fréquence de la boucle interne [Hz]
TE = 1.0 / FS

# Autorités maximales en couple [N·m] (Tableau II.3)
TAU_MAX = {"roulis": 0.59, "tangage": 0.13, "lacet": 0.52}


# ----------------------------------------------------------------------
# 2. REGLAGE DES BOUCLES ET MARGES
# ----------------------------------------------------------------------
def margins(L, w):
    """Pulsation de coupure et marge de phase de la boucle ouverte L(jw)."""
    mag = np.abs(L)
    idx = np.where(mag < 1)[0]
    if len(idx) == 0:
        return np.nan, np.nan
    i = idx[0]
    return w[i], 180.0 + np.degrees(np.angle(L[i]))


def design_axis(J, wc, lag, rhp_zero=None):
    """
    PI sur la vitesse angulaire (sortie = couple en N·m) :
        K_omega = J * wc            (coupure visée)
        K_i     = K_omega * wc / 5  (zéro du PI à wc/5)
    Plante : 1/(J s) * 1/(lag s + 1) * exp(-T_DELAY s) [* (1 - s²/z²)]
    """
    w = np.logspace(-1, 3, 40000)
    s = 1j * w
    K = J * wc
    Ki = K * wc / 5.0
    C = K + Ki / s
    G = 1.0 / (J * s) / (lag * s + 1) * np.exp(-T_DELAY * s)
    if rhp_zero is not None:
        G = G * (1 - (s / rhp_zero) ** 2)
    wco, pm = margins(C * G, w)
    return dict(K=K, Ki=Ki, wc_real=wco, PM=pm, K_angle=wc / 4.0)


def design_all():
    z = np.sqrt(P0 * h / It)   # zéro à partie réelle positive du tangage (voie nacelles)
    axes = {
        "roulis":  design_axis(Jx, 15.0, TAU_M),
        "tangage": design_axis(Jy, 10.0, TAU_S, rhp_zero=z),
        "lacet":   design_axis(Jz, 10.0, TAU_S),
    }
    print(f"Zéro RHP du tangage (voie nacelles) : z = {z:.1f} rad/s  (règle : wc < z/2 = {z/2:.0f} rad/s)\n")
    print(f"{'axe':8s} {'K_omega':>8s} {'Ki':>6s} {'wc [rad/s]':>11s} {'PM [°]':>7s} {'K_angle [1/s]':>14s}")
    for k, v in axes.items():
        print(f"{k:8s} {v['K']:8.3f} {v['Ki']:6.2f} {v['wc_real']:11.1f} {v['PM']:7.1f} {v['K_angle']:14.2f}")
    print()
    return axes


# ----------------------------------------------------------------------
# 3. SIMULATION TEMPORELLE DE LA CASCADE (un axe)
# ----------------------------------------------------------------------
def simulate_cascade(J, gains, lag, tau_max, step_deg=10.0, t_end=3.0,
                     rate_limit_deg=100.0, dist=None):
    """
    Cascade P(angle) -> PI(vitesse) -> couple saturé -> retard -> actionneur 1er ordre
    -> double intégrateur. Régulateur discret à FS Hz, anti-windup par écrêtage.
    dist(t) : couple perturbateur éventuel [N·m] (ex. moment gyroscopique).
    """
    dt = 1e-3
    n_per_ctrl = int(round(TE / dt))
    n = int(t_end / dt)
    K, Ki, Ka = gains["K"], gains["Ki"], gains["K_angle"]
    ref = np.radians(step_deg)
    rl = np.radians(rate_limit_deg)

    ang = rate = tau = 0.0
    integ = 0.0
    u_cmd = u_pending = 0.0           # retard d'un échantillon de commande
    t_log, a_log, u_log = [], [], []
    for k in range(n):
        t = k * dt
        if k % n_per_ctrl == 0:
            rate_ref = np.clip(Ka * (ref - ang), -rl, rl)
            e = rate_ref - rate
            u_unsat = K * e + integ
            u = np.clip(u_unsat, -tau_max, tau_max)
            if abs(u_unsat) < tau_max or np.sign(e) != np.sign(u_unsat):
                integ += Ki * TE * e          # intégration conditionnelle (anti-windup)
            u_cmd, u_pending = u_pending, u    # retard d'un échantillon
        tau += (u_cmd - tau) / lag * dt
        d = dist(t) if dist else 0.0
        rate += (tau + d) / J * dt
        ang += rate * dt
        t_log.append(t); a_log.append(np.degrees(ang)); u_log.append(tau)
    return np.array(t_log), np.array(a_log), np.array(u_log)


# ----------------------------------------------------------------------
# 4. TRANSITION
# ----------------------------------------------------------------------
def theta_n_max(V, cl_max=CL_MAX):
    """Angle max des nacelles (0° = vertical) pour tenir l'altitude, éq. (II.7) [deg]."""
    lift = 0.5 * rho * V**2 * S * cl_max
    ang = np.degrees(np.arccos(np.clip((W - lift) / (2 * T_u), -1.0, 1.0)))
    return np.minimum(ang, 90.0)   # au-delà de Vs, plus de contrainte : 90° atteignable


def simulate_transition(cl=0.9, margin_deg=3.0, tilt_rate_deg=90.0, t_end=4.0):
    """
    Modèle masse ponctuelle : les gaz tiennent l'altitude (2T cosθn + L = W),
        m dV/dt = (W - L) tan θn - D
    La consigne d'inclinaison suit θn,ref(V) = min(θn,max(V) - marge, 90°), limitée en vitesse.
    Modèle idéalisé (optimiste) : ignore la dynamique de tangage et le moment de réaction.
    """
    dt = 5e-3
    V = th = 0.0
    out = []
    for k in range(int(t_end / dt)):
        ref = np.radians(np.clip(theta_n_max(V, cl) - margin_deg, 0, 90))
        th += np.clip(ref - th, -np.radians(tilt_rate_deg) * dt, np.radians(tilt_rate_deg) * dt)
        L = 0.5 * rho * V**2 * S * cl
        D = 0.5 * rho * V**2 * S * CD
        T = (W - L) / (2 * np.cos(th))
        V += ((W - L) * np.tan(th) - D) / m * dt
        out.append((k * dt, V, np.degrees(th), T))
    return np.array(out)


# ----------------------------------------------------------------------
# MAIN
# ----------------------------------------------------------------------
if __name__ == "__main__":
    axes = design_all()

    # Moment gyroscopique parasite en roulis lors d'un basculement différentiel (I.2)
    Ih, w_h = 1.5e-4, 5700 * 2 * np.pi / 60
    Om = np.radians(90.0)
    print(f"Moment gyroscopique différentiel : {2 * Ih * w_h * Om:.2f} N·m "
          f"(autorité roulis : {TAU_MAX['roulis']} N·m)\n")

    # Réponses indicielles 10° (roulis, lacet) ; perturbation gyroscopique sur le roulis
    t, phi, u = simulate_cascade(Jx, axes["roulis"], TAU_M, TAU_MAX["roulis"])
    _, phi_d, _ = simulate_cascade(Jx, axes["roulis"], TAU_M, TAU_MAX["roulis"], step_deg=0.0,
                                   dist=lambda t: 0.28 if 0.5 < t < 1.2 else 0.0)
    _, psi, _ = simulate_cascade(Jz, axes["lacet"], TAU_S, TAU_MAX["lacet"])
    print(f"Roulis : dépassement {max(phi) / 10 * 100 - 100:.1f} %, "
          f"écart max sous perturbation gyro = {np.max(np.abs(phi_d)):.1f}°")
    print(f"Lacet  : dépassement {max(psi) / 10 * 100 - 100:.1f} %\n")

    # Loi de transition
    print("θn,max(V) [°]   V:   0     4     6     8     9.4")
    for cl in (1.1, 0.9):
        print(f"  CL,max={cl}: " + "  ".join(f"{theta_n_max(v, cl):5.1f}" for v in (0, 4, 6, 8, 9.4)))
    tr = simulate_transition()
    i = np.argmax(tr[:, 1] > 0.99 * tr[-1, 1])
    print(f"\nTransition (idéalisée) : V = {tr[-1, 1]:.1f} m/s atteinte en ≈ {tr[i, 0]:.1f} s, "
          f"T/Tu max = {tr[:, 3].max() / T_u:.2f}")

    if HAS_PLT:
        fig, ax = plt.subplots(2, 2, figsize=(11, 7))
        ax[0, 0].plot(t, phi, label="roulis"); ax[0, 0].plot(t, psi, label="lacet")
        ax[0, 0].axhline(10, ls=":", c="gray"); ax[0, 0].set_title("Échelon d'angle 10°")
        ax[0, 0].set_xlabel("t [s]"); ax[0, 0].set_ylabel("angle [°]"); ax[0, 0].legend()
        ax[0, 1].plot(t, phi_d); ax[0, 1].set_title("Roulis sous perturbation gyroscopique (0,28 N·m)")
        ax[0, 1].set_xlabel("t [s]"); ax[0, 1].set_ylabel("φ [°]")
        Vs = np.linspace(0, 12, 200)
        for cl in (1.1, 0.9):
            ax[1, 0].plot(Vs, theta_n_max(Vs, cl), label=f"CL,max={cl}")
        ax[1, 0].set_title("θn,max(V) (II.7), 0° = vertical"); ax[1, 0].set_xlabel("V [m/s]")
        ax[1, 0].set_ylabel("θn [°]"); ax[1, 0].legend()
        ax[1, 1].plot(tr[:, 0], tr[:, 1], label="V [m/s]")
        ax[1, 1].plot(tr[:, 0], tr[:, 2] / 10, label="θn / 10 [°/10]")
        ax[1, 1].set_title("Transition (masse ponctuelle, optimiste)"); ax[1, 1].set_xlabel("t [s]")
        ax[1, 1].legend()
        plt.tight_layout(); plt.savefig("asservissement_tiltrotor.png", dpi=130)
        print("Figure enregistrée : asservissement_tiltrotor.png")
