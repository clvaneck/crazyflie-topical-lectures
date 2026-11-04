"""E1A (day 2, 01_tracking): Kalman track fit in a parallel-plane detector.

Run:  .venv/bin/python code/day2/e1a_tracking.py
Prints the results and saves figures + summary.json to code/day2/data/e1a/.
"""

import matplotlib
matplotlib.use("Agg")

# # E1A: Kalman track fit in a parallel-plane detector
#
# Our own solution to exercise E1A from [01_tracking.ipynb](01_tracking.ipynb).
#
# **Setup.** There are 10 planes at z = 1,\dots,10 m. Each measures x with \sigma = 0.01 m. After its measurement, each plane gives the slope a Gaussian kick with std 0.02 (multiple scattering). The track starts at z=0 with x_0 \sim N(0, 0.1) and t_0 \sim N(0, 0.1).
#
# **Notation:** state x = (x, t_x), covariance C, transport F, process noise Q, projection H = (1, 0), measurement variance V, gain K.
#
# **Contents**
# 1. Simulation and Kalman filter (vectorised over all tracks)
# 2. One predict/update step, worked out
# 3. 1,000 tracks: pulls, resolution, predicted residuals
# 4. How the resolution depends on \sigma and on Q (what if Q \gg \sigma?)
# 5. Extra: track picture with the green widening prediction bands
# 6. Extra: binary pixel readout instead of Gaussian hits
# 7. Check against the supplied reference

from pathlib import Path
import json, sys
import numpy as np
import matplotlib.pyplot as plt

notebook_dir = Path(__file__).resolve().parent
sys.path.insert(0, str(notebook_dir.resolve().parent))

SEED = 2024
output = notebook_dir / "data" / "e1a"
output.mkdir(parents=True, exist_ok=True)
print("Figures go to:", output.resolve())

# ## 1. Simulation and Kalman filter
#
# **Truth.** We start at z=0 and fly straight to plane 1; there's no material before it. At each plane k we record a hit m_k = x_k + \epsilon, and *afterwards* the slope gets a kick \delta t \sim N(0, q^2). The kick only shows up in position after the next gap.
#
# **Filter.** We start from \hat x = (0, 0) with a broad prior C = \mathrm{diag}(1, 1). Then, at each plane:
#
# - **predict:** \hat x^- = F\hat x and C^- = F\,(C + Q_\text{plane})\,F^T with F = \begin{pmatrix}1 & d\\ 0 & 1\end{pmatrix} and Q_\text{plane} = \mathrm{diag}(0, q^2). The previous plane's kick is added before transport, and only from plane 2 onwards.
# - **update:** r = m - H\hat x^-, R = HC^-H^T + V, K = C^-H^T/R, \hat x = \hat x^- + Kr, C = (I-KH)C^-(I-KH)^T + KVK^T (Joseph form).
#
# C doesn't depend on the measurements, so one covariance serves all tracks. That lets us vectorise the states over the whole ensemble. The final estimate is the **incoming** state at plane 10, before its kick.

def simulate(n, sigma=0.01, q=0.02, planes=10, d=1.0, rng=None, pixel_pitch=None):
    # Returns true incoming states (n, planes, 2) and hits (n, planes).
    rng = rng or np.random.default_rng(SEED)
    x = rng.normal(0, 0.1, n)
    t = rng.normal(0, 0.1, n)
    truth = np.empty((n, planes, 2))
    for k in range(planes):
        x = x + d * t                      # fly to plane k
        truth[:, k] = np.c_[x, t]          # incoming state at plane k
        t = t + rng.normal(0, q, n)        # scatter after measuring
    if pixel_pitch is None:
        hits = truth[:, :, 0] + rng.normal(0, sigma, (n, planes))
    else:                                  # binary readout: centre of the closest pixel
        hits = (np.floor(truth[:, :, 0] / pixel_pitch) + 0.5) * pixel_pitch
    return truth, hits


def kalman_fit(hits, sigma=0.01, q=0.02, d=1.0, prior_std=(1.0, 1.0)):
    # Vectorised over tracks. Returns per-plane predicted/filtered states, covariances and residuals.
    n, planes = hits.shape
    F = np.array([[1.0, d], [0.0, 1.0]])
    H = np.array([1.0, 0.0])
    Q_plane = np.diag([0.0, q**2])
    V = sigma**2
    state = np.zeros((n, 2))
    C = np.diag(np.square(prior_std)).astype(float)
    out = {k: [] for k in ("x_pred", "C_pred", "x", "C", "r", "R", "K")}
    for k in range(planes):
        # predict: add the previous plane's kick (none before plane 1), then transport
        C_in = C + Q_plane if k > 0 else C
        state = state @ F.T
        C = F @ C_in @ F.T
        out["x_pred"].append(state.copy()); out["C_pred"].append(C.copy())
        # update
        r = hits[:, k] - state @ H
        R = H @ C @ H + V
        K = C @ H / R
        state = state + np.outer(r, K)
        A = np.eye(2) - np.outer(K, H)
        C = A @ C @ A.T + np.outer(K, K) * V
        out["x"].append(state.copy()); out["C"].append(C.copy())
        out["r"].append(r); out["R"].append(R); out["K"].append(K)
    out = {k: np.asarray(v) for k, v in out.items()}
    out["x"] = out["x"].transpose(1, 0, 2)          # (n, planes, 2)
    out["x_pred"] = out["x_pred"].transpose(1, 0, 2)
    out["r"] = out["r"].T                           # (n, planes)
    return out

# ## 2. One predict/update step, worked out
#
# At **plane 1** the prior is broad, so K \approx (1, 1) for the slope. The first hit fixes the position, but the slope is only constrained through the prior. At **plane 2**, transport has coupled them: C^-_{tx} \neq 0. The position residual then also corrects the slope, through \Delta t = C^-_{tx}\, r / (C^-_{xx} + V). This is the same mechanism as "position measurements reveal velocity" in E2B.

truth1, hits1 = simulate(1, rng=np.random.default_rng(SEED))
fit1 = kalman_fit(hits1)
for k in (0, 1):
    print(f"--- plane {k+1} (z = {k+1} m)")
    print("  predicted state   :", np.round(fit1['x_pred'][0, k], 4))
    print("  predicted C       :", np.round(fit1['C_pred'][k], 6).tolist())
    print(f"  hit m = {hits1[0, k]:.4f}  residual r = {fit1['r'][0, k]:+.4f}  R = {fit1['R'][k]:.6f}")
    print("  gain K (x, slope) :", np.round(fit1['K'][k], 4))
    print("  filtered state    :", np.round(fit1['x'][0, k], 4), "  truth:", np.round(truth1[0, k], 4))
    print("  filtered sigma    :", np.round(np.sqrt(np.diag(fit1['C'][k])), 5))

# ## 3. 1,000 tracks: pulls, resolution and predicted residuals
#
# The pull is (\hat x_i - x_i^\text{true})/\sqrt{C_{ii}} at the last plane. If the filter is consistent, the pulls follow N(0,1). The **predicted residual** r_k = m_k - H\hat x_k^- should have variance R_k, so r_k/\sqrt{R_k} should also follow N(0,1) at every plane.

def ensemble(n=1000, sigma=0.01, q=0.02, seed=SEED, pixel_pitch=None, V_sigma=None):
    truth, hits = simulate(n, sigma, q, rng=np.random.default_rng(seed), pixel_pitch=pixel_pitch)
    fit = kalman_fit(hits, sigma if V_sigma is None else V_sigma, q)
    err = fit["x"][:, -1] - truth[:, -1]
    C = fit["C"][-1]
    pulls = err / np.sqrt(np.diag(C))
    return dict(truth=truth, hits=hits, fit=fit, err=err, C=C, pulls=pulls)

def gauss_fit(values):
    # Simple Gaussian 'fit': mean and std with their statistical errors.
    n = len(values); m, s = values.mean(), values.std(ddof=1)
    return m, s / np.sqrt(n), s, s / np.sqrt(2 * (n - 1))

E = ensemble()
names = ["x", "slope"]
print(f"Tracks: {len(E['err'])}, sigma = 0.01 m, q = 0.02")
for i, name in enumerate(names):
    m, dm, s, ds = gauss_fit(E["pulls"][:, i])
    print(f"pull {name:5s}: mean = {m:+.3f} ± {dm:.3f}   std = {s:.3f} ± {ds:.3f}   "
          f"(N(0,1) within 2 sigma: {abs(m) < 2*dm and abs(s-1) < 2*ds})")
for i, name, unit in ((0, "x", "m"), (1, "slope", "")):
    print(f"resolution {name:5s}: empirical RMS = {E['err'][:, i].std(ddof=1):.5f} {unit}   "
          f"filter sqrt(C_ii) = {np.sqrt(E['C'][i, i]):.5f} {unit}")
print(f"Correlation of the x/slope errors: empirical {np.corrcoef(E['err'].T)[0,1]:+.3f}, "
      f"from C {E['C'][0,1]/np.sqrt(E['C'][0,0]*E['C'][1,1]):+.3f}")
contained = np.all(np.abs(E["truth"][:, :, 0]) <= 0.5, axis=1).mean()
print(f"Tracks fully inside the 1 m active height: {contained:.1%} (we use all planes as a control, as the reference does)")

gx = np.linspace(-4.5, 4.5, 200)
unit_gauss = np.exp(-gx**2 / 2) / np.sqrt(2 * np.pi)
fig, axes = plt.subplots(2, 3, figsize=(13, 7))
for i, name in enumerate(names):
    ax = axes[0, i]
    m, dm, s, ds = gauss_fit(E["pulls"][:, i])
    ax.hist(E["pulls"][:, i], bins=40, range=(-4.5, 4.5), density=True, alpha=0.6, color="C0")
    ax.plot(gx, unit_gauss, "k-", lw=1.5, label="N(0,1)")
    ax.set_title(f"pull {name}: μ = {m:+.3f} ± {dm:.3f}, σ = {s:.3f} ± {ds:.3f}", fontsize=10)
    ax.legend(fontsize=8)
    ax = axes[1, i]
    sd = np.sqrt(E["C"][i, i])
    ax.hist(E["err"][:, i], bins=40, range=(-4.5*sd, 4.5*sd), density=True, alpha=0.6, color="C1")
    ax.plot(gx * sd, unit_gauss / sd, "k-", lw=1.5, label="N(0, C_ii)")
    unit = " [m]" if i == 0 else ""
    ax.set_title(f"resolution {name}: RMS = {E['err'][:, i].std(ddof=1):.4f}", fontsize=10)
    ax.set_xlabel(f"x̂ − x_true{unit}" if i == 0 else "t̂ − t_true")
    ax.legend(fontsize=8)
# predicted residuals: normalised and pooled, plus their width per plane
nr = E["fit"]["r"] / np.sqrt(E["fit"]["R"])
ax = axes[0, 2]
ax.hist(nr[:, 1:].ravel(), bins=40, range=(-4.5, 4.5), density=True, alpha=0.6, color="C2")
ax.plot(gx, unit_gauss, "k-", lw=1.5, label="N(0,1)")
ax.set_title(f"r/√R, planes 2–10 pooled: σ = {nr[:, 1:].std():.3f}", fontsize=10)
ax.legend(fontsize=8)
ax = axes[1, 2]
planes = np.arange(1, 11)
ax.semilogy(planes, E["fit"]["r"].std(axis=0), "o", color="C2", label="empirical RMS of r")
ax.semilogy(planes, np.sqrt(E["fit"]["R"]), "k-", label="√R from the filter")
ax.axhline(0.01, color="grey", ls=":", label="hit σ")
ax.set_xlabel("plane"); ax.set_ylabel("predicted residual [m]")
ax.set_title("predicted residual vs plane", fontsize=10)
ax.legend(fontsize=8)
fig.tight_layout()
fig.savefig(output / "pulls_resolution_residuals.png", dpi=140)
plt.close(fig)

# **Reading this.** The pulls have width ≈ 1 and mean ≈ 0, so the filter's covariance is honest. At plane 1 the predicted residual is huge: the prior is 1 m wide and *is* what the residual measures. From plane 2 onwards, \sqrt R drops to roughly \sqrt{\sigma^2 + \sigma^2_\text{pred}}, with the scattering of the last gap feeding \sigma_\text{pred}. The plane-1 residual is consistent too (r/√R stays N(0,1)), but it measures the prior, not the detector. That's why it's excluded from the pooled histogram.

# ## 4. Resolution versus hit resolution σ and scattering Q
#
# We scan σ for a few values of q, and q for a few values of σ. The markers are the empirical RMS from 1,000 tracks; the lines are \sqrt{C_{ii}} from the filter. Because C doesn't depend on the data, the lines are cheap to compute densely.

def model_sd(sigma, q):
    C = kalman_fit(np.zeros((1, 10)), sigma, q)["C"][-1]
    return np.sqrt(np.diag(C))

sigmas = np.logspace(-3.5, -0.5, 13)
qs = np.r_[0, np.logspace(-3.5, -0.3, 13)]
fig, axes = plt.subplots(2, 2, figsize=(12, 8))
for j, q in enumerate([0.0, 0.002, 0.02, 0.2]):
    emp = np.array([ensemble(1000, s, q, seed=SEED + j)["err"].std(axis=0, ddof=1) for s in sigmas])
    dense = np.logspace(-3.5, -0.5, 80)
    mod = np.array([model_sd(s, q) for s in dense])
    for i in range(2):
        axes[0, i].loglog(dense, mod[:, i], "-", color=f"C{j}", label=f"q = {q}")
        axes[0, i].loglog(sigmas, emp[:, i], "o", color=f"C{j}", ms=4)
for j, s in enumerate([0.001, 0.01, 0.1]):
    emp = np.array([ensemble(1000, s, q, seed=SEED + 10 + j)["err"].std(axis=0, ddof=1) for q in qs[1:]])
    dense = np.logspace(-3.5, -0.3, 80)
    mod = np.array([model_sd(s, q) for q in dense])
    for i in range(2):
        axes[1, i].loglog(dense, mod[:, i], "-", color=f"C{j}", label=f"σ = {s} m")
        axes[1, i].loglog(qs[1:], emp[:, i], "o", color=f"C{j}", ms=4)
for i, name in enumerate(["position resolution [m]", "slope resolution"]):
    axes[0, i].set_xlabel("hit resolution σ [m]"); axes[0, i].set_ylabel(name)
    axes[1, i].set_xlabel("scattering std per plane q"); axes[1, i].set_ylabel(name)
    for row in range(2):
        axes[row, i].grid(alpha=0.3, which="both"); axes[row, i].legend(fontsize=8)
axes[0, 0].plot(sigmas, sigmas, "k:", lw=1); axes[0, 0].text(sigmas[2], sigmas[2]*1.6, "= σ", fontsize=8)
axes[0, 1].plot(sigmas, sigmas, "k:", lw=1); axes[0, 1].text(sigmas[2], sigmas[2]*1.6, "= σ / 1 m", fontsize=8)
fig.suptitle("Markers: empirical RMS (1000 tracks). Lines: √C_ii from the Kalman filter.")
fig.tight_layout()
fig.savefig(output / "resolution_scan.png", dpi=140)
plt.close(fig)

for label, s, q in (("q ≪ σ (σ=0.01, q=0.0001)", 0.01, 1e-4), ("nominal (σ=0.01, q=0.02)", 0.01, 0.02),
                    ("q ≫ σ (σ=0.01, q=0.5)", 0.01, 0.5)):
    sx, st = model_sd(s, q)
    print(f"{label:28s}: σ_x = {sx:.4f} m, σ_t = {st:.4f}")
print("Straight-line fit with 10 hits, no scattering: σ_t =", f"{0.01/np.sqrt(np.sum((np.arange(1,11)-5.5)**2)):.4f}")

# **What happens if Q ≫ σ?** Then the track forgets its past at every plane. Old hits are about a direction that scattering has since changed, so the filter down-weights them.
#
# - **Position at the last plane → σ.** Only the last hit still knows where the track is.
# - **Slope → about √2·σ/d.** The slope is mostly set by the last two hits (t \approx (m_{10} - m_9)/d). The random kick at plane 9, which sits between them, also adds to it. So the slope resolution grows with q until it saturates at the two-hit value.
# - **With Q \to 0** we recover a plain least-squares straight-line fit. Position and slope both scale ∝ σ, and the slope is \sigma/\sqrt{\sum (z_i - \bar z)^2} \approx 0.0011 for σ = 1 cm.
#
# In between, for small q, the resolution is *measurement dominated* (∝ σ). For large q it's *scattering dominated*: the slope stays flat in q because the last two hits pin it, and the position stays ≈ σ. Real detectors have exactly this trade-off: thinner planes (smaller q) only help until scattering drops below the hit resolution.

# ## 5. Extra: the track picture
#
# This mimics `tracking_reference.svg`. It shows the true path with its kinks, the hits with error bars, the dotted initial direction, and **green prediction bands** that widen between planes. After each update we transport the filtered state forward. Its ±1σ band grows as \sqrt{C_{xx} + 2(z-z_k)C_{xt} + (z-z_k)^2(C_{tt} + q^2)}, and it collapses again at the next hit.

def draw_track(sigma=0.01, q=0.02, seed=11):
    truth, hits = simulate(1, sigma, q, rng=np.random.default_rng(seed))
    fit = kalman_fit(hits, sigma, q)
    z = np.arange(1, 11.0)
    # true path from z=0 (reconstruct the start from plane-1 state)
    x0 = truth[0, 0, 0] - truth[0, 0, 1]
    zt = np.r_[0, z]; xt = np.r_[x0, truth[0, :, 0]]
    fig, ax = plt.subplots(figsize=(10, 4.5))
    # prediction bands: from each filtered state to the next plane
    for k in range(9):
        xs, ts = fit["x"][0, k]
        C = fit["C"][k] + np.diag([0, q**2])
        dz = np.linspace(0, 1, 30)
        mean = xs + dz * ts
        sd = np.sqrt(C[0, 0] + 2 * dz * C[0, 1] + dz**2 * C[1, 1])
        ax.fill_between(z[k] + dz, mean - sd, mean + sd, color="green", alpha=0.25, lw=0,
                        label="prediction ±1σ" if k == 0 else None)
        ax.plot(z[k] + dz, mean, color="green", lw=1)
    ax.plot(zt, xt, "k-", lw=1, label="true track")
    ax.errorbar(z, hits[0], yerr=sigma, fmt="o", color="C3", ms=4, capsize=2, label="hits (±σ)")
    xf = fit["x"][0, :, 0]; sf = np.sqrt(fit["C"][:, 0, 0])
    ax.errorbar(z, xf, yerr=sf, fmt="s", color="C0", ms=3, capsize=2, label="filtered (±1σ)")
    ax.plot([0, 10], [x0, x0 + 10 * truth[0, 0, 1]], "k:", lw=1, label="initial direction")
    for zz in z:
        ax.axvline(zz, color="grey", lw=0.6, alpha=0.5)
    ax.set_xlabel("z [m]"); ax.set_ylabel("x [m]")
    ax.set_title(f"Kalman track fit: σ = {sigma} m, q = {q}")
    ax.legend(fontsize=8, loc="best")
    fig.tight_layout()
    return fig

fig = draw_track()
fig.savefig(output / "track.png", dpi=140); plt.close(fig)
# A zoom where the bands are visible: larger scattering
fig = draw_track(sigma=0.01, q=0.1, seed=3)
fig.savefig(output / "track_large_q.png", dpi=140); plt.close(fig)

# ## 6. Extra: binary pixel readout
#
# Now each plane reports only the **centre of the closest 1 cm pixel**. The true hit residual is uniform on [-p/2, p/2] with RMS p/\sqrt{12}; the exercise text writes "√12", but the RMS is p/\sqrt{12}. We feed the filter V = p^2/12 and look at whether the track parameters stay Gaussian. For the pixel to be the only effect, we turn scattering down to q = 10^{-4}, then repeat at the nominal q = 0.02.

p = 0.01
fig, axes = plt.subplots(2, 3, figsize=(13, 7))
for row, q in enumerate([1e-4, 0.02]):
    P = ensemble(5000, sigma=p / np.sqrt(12), q=q, pixel_pitch=p, seed=SEED + 50 + row)
    single = (P["hits"] - P["truth"][:, :, 0]).ravel()
    ax = axes[row, 0]
    ax.hist(single / p, bins=40, density=True, alpha=0.6, color="C4")
    ax.set_title(f"hit residual / p (q = {q}): RMS = {single.std()/p:.3f} (1/√12 = {1/np.sqrt(12):.3f})", fontsize=9)
    for i, name in enumerate(names):
        ax = axes[row, i + 1]
        pull = P["pulls"][:, i]
        m, dm, s, ds = gauss_fit(pull)
        kurt = np.mean(((pull - pull.mean()) / pull.std())**4) - 3
        ax.hist(pull, bins=50, range=(-4.5, 4.5), density=True, alpha=0.6, color="C4")
        ax.plot(gx, unit_gauss, "k-", lw=1.5)
        ax.set_title(f"pull {name}, q={q}: σ = {s:.3f} ± {ds:.3f}, excess kurtosis = {kurt:+.2f}", fontsize=9)
fig.tight_layout()
fig.savefig(output / "pixel_readout.png", dpi=140)
plt.close(fig)

# **Answer: not always.** Each single hit residual is uniform (RMS p/√12 ✓). With V = p²/12 the
# pull *widths* stay ≈ 1, but the *shape* depends on how many hits really contribute:
#
# - **Nominal scattering (q = 0.02):** the position at the last plane is set almost entirely by the last
#   hit, so the position pull is **flat** (excess kurtosis ≈ −1.2, the uniform value). The slope uses
#   about two hits, giving a triangle-like distribution that's already close to Gaussian (≈ −0.5).
# - **No scattering (q ≈ 0):** all 10 hits count, so you'd expect the central limit theorem to make it
#   Gaussian. Instead the pulls are **too peaked, with tails** (kurtosis ≈ +0.6). The rounding errors of
#   hits along one straight line aren't independent. A track with a small slope sees nearly the same
#   pixel offset in neighbouring planes, so the errors are correlated and don't average out. Sometimes
#   they cancel well (the peak), sometimes they all push the same way (the tails).
#
# So the Gaussian/Kalman description of the uncertainty is only approximate for binary readout. The
# variance is right on average, but the error is not Gaussian.

# ## 7. Check against the supplied reference
#
# The course ships a reference fit (`day2/tracking.py`), so we compare the covariance at the last plane and the pull statistics. The filter covariances must match exactly, because they don't depend on random numbers. The pull statistics only have to agree within statistical error, since the random streams differ.

from day2 import tracking
ref = tracking.ensemble(tracks=1000, seed=7, detector=tracking.Detector(scattering_slope_std=0.02))
ours = ensemble(1000, 0.01, 0.02)
print("C at last plane, reference:", np.round(ref["covariance"][0], 9).tolist())
print("C at last plane, ours     :", np.round(ours["C"], 9).tolist())
print("max |difference|:", float(np.abs(ref["covariance"][0] - ours["C"]).max()))
print("reference pull mean / std:", np.round(ref["summary"]["mean_pull"], 3), np.round(ref["summary"]["std_pull"], 3))
print("our pull mean / std      :", np.round(ours["pulls"].mean(0), 3), np.round(ours["pulls"].std(0, ddof=1), 3))
json.dump(dict(seed=SEED, sigma=0.01, q=0.02, tracks=1000,
               pull_mean=ours["pulls"].mean(0).tolist(), pull_std=ours["pulls"].std(0, ddof=1).tolist(),
               resolution_rms=ours["err"].std(0, ddof=1).tolist(),
               filter_sd=np.sqrt(np.diag(ours["C"])).tolist()),
          open(output / "summary.json", "w"), indent=2)
print("Saved", output / "summary.json")

# ## Completion: one predict/update in words
#
# Take plane 2. The filtered state at plane 1 is transported 1 m: \hat x^- = \hat x_1 + d\,\hat t_1. Its covariance becomes F(C_1 + Q_\text{plane})F^T; the kick at plane 1 adds q^2 to the slope variance, and transport turns it into d^2q^2 of position variance plus dq^2 of cross-covariance. The hit gives a residual r = m_2 - \hat x^- with expected variance R = C^-_{xx} + V. The gain K = (C^-_{xx}, C^-_{tx})/R moves the position *and* the slope, the slope through the cross term. The covariance shrinks to (I-KH)C^-.
#
# Saved in `data/e1a/`: seed, figures and pull statistics (`summary.json`).

print("\nFigures saved in", output.resolve())
