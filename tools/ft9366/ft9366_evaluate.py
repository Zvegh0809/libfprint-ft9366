#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-or-later
"""
FT9366 matcher evaluation on the captures under ~/ft9366/data.

Method: enhancement (background subtraction, Gaussian, high-pass) -> SIFT
keypoints (3x upscaled) -> ratio test -> RANSAC similarity -> NCC over the
aligned overlap (only where both images contain ridges).

Output: per-finger cross-validated FRR / FAR for several enrollment sizes.

Requires: python-numpy, python-opencv
Usage:    python3 ft9366_evaluate.py [--data ~/ft9366/data]
"""
import argparse, glob, os, sys
import numpy as np
import cv2

H, W = 77, 60


def read16(p):
    b = open(p, "rb").read()
    i = b.index(b"4095\n") + 5
    return np.frombuffer(b[i:], ">u2").astype(np.float32).reshape(80, 64)


def box(a, k):
    p = np.pad(a, k // 2, mode="edge"); h, w = a.shape
    c = np.cumsum(np.cumsum(np.pad(p, ((1, 0), (1, 0))), 0), 1)
    return (c[k:k + h, k:k + w] - c[:h, k:k + w] - c[k:k + h, :w] + c[:h, :w]) / (k * k)


def gauss(a):
    p = np.pad(a, 1, mode="edge"); h, w = a.shape; k = np.array([1, 2, 1])
    return sum(k[i] * k[j] * p[i:i + h, j:j + w] for i in range(3) for j in range(3)) / 16


def enhance(raw, bg):
    d = (raw - bg)[3:, 2:62]
    g = gauss(d)
    h = g - box(g, 9)
    return ((h - h.mean()) / (h.std() + 1e-6)).astype(np.float32)


SIFT = cv2.SIFT_create(contrastThreshold=0.01, edgeThreshold=20)
BF = cv2.BFMatcher(cv2.NORM_L2)


def features(e, up=3):
    u = np.clip((e + 3) / 6 * 255, 0, 255).astype(np.uint8)
    u = cv2.resize(u, (W * up, H * up), interpolation=cv2.INTER_CUBIC)
    kp, des = SIFT.detectAndCompute(u, None)
    pts = np.float32([k.pt for k in kp]) / up if kp else np.zeros((0, 2), np.float32)
    return pts, des


def align(fa, fb, ratio=0.9, thr=2.0, min_inl=3):
    pa, da = fa; pb, db = fb
    if da is None or db is None or len(pa) < 3 or len(pb) < 3:
        return None
    m = BF.knnMatch(da, db, k=2)
    good = [x[0] for x in m if len(x) == 2 and x[0].distance < ratio * x[1].distance]
    if len(good) < min_inl:
        return None
    A = pa[[g.queryIdx for g in good]]; B = pb[[g.trainIdx for g in good]]
    M, inl = cv2.estimateAffinePartial2D(A, B, method=cv2.RANSAC,
                                         ransacReprojThreshold=thr, maxIters=2000)
    if M is None or inl.sum() < min_inl or not 0.9 < np.hypot(M[0, 0], M[1, 0]) < 1.1:
        return None
    return M


def content(e):
    return cv2.blur((e * e).astype(np.float32), (9, 9)) > 0.25


def aligned_ncc(ea, eb, M, min_overlap=0.2):
    wa = cv2.warpAffine(ea, M, (W, H), flags=cv2.INTER_LINEAR, borderValue=np.nan)
    mask = ~np.isnan(wa) & content(eb) & content(np.nan_to_num(wa))
    if mask.mean() < min_overlap:
        return 0.0
    x = wa[mask] - wa[mask].mean(); y = eb[mask] - eb[mask].mean()
    return float((x * y).sum() / np.sqrt((x * x).sum() * (y * y).sum()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=os.path.expanduser("~/ft9366/data"))
    a = ap.parse_args()

    fingers, E = [], []
    for fdir in sorted(glob.glob(os.path.join(a.data, "*"))):
        for sess in sorted(glob.glob(os.path.join(fdir, "*"))):
            bgs = sorted(glob.glob(os.path.join(sess, "bg_*.pgm")))
            imgs = sorted(glob.glob(os.path.join(sess, "img_*.pgm")))
            if not bgs or not imgs:
                continue
            bg = np.mean([read16(p) for p in bgs], 0)
            for p in imgs:
                fingers.append(os.path.basename(fdir)); E.append(enhance(read16(p), bg))
    n = len(E)
    if n < 4:
        sys.exit("Not enough images.")
    fing = np.array(fingers)
    idx = np.zeros(n, int)
    for f in set(fingers):
        sel = np.where(fing == f)[0]; idx[sel] = np.arange(len(sel))
    print(f"{n} images, {len(set(fingers))} fingers: {sorted(set(fingers))}")

    F = [features(e) for e in E]
    C = np.zeros((n, n))
    for i in range(n):
        for j in range(n):
            if i != j:
                M = align(F[i], F[j])
                if M is not None:
                    C[i, j] = aligned_ncc(E[i], E[j], M)

    imp_pairs = C[fing[:, None] != fing[None, :]]
    print(f"Impostor pairs: {imp_pairs.size}, highest score: {imp_pairs.max():.3f}")
    per = min(np.bincount(np.unique(fing, return_inverse=True)[1]))
    for nen in (5, 10, 15):
        if nen >= per:
            continue
        G, I = [], []
        for s in range(per):
            en = {(s + k) % per for k in range(nen)}
            pr = {(s + nen + k) % per for k in range(per - nen)}
            for f in sorted(set(fingers)):
                tm = [t for t in range(n) if fing[t] == f and idx[t] in en]
                for p in range(n):
                    if idx[p] in pr:
                        (G if fing[p] == f else I).append(max(C[p, t] for t in tm))
        G = np.array(G); I = np.array(I)
        for thr in (0.75, 0.8, 0.85):
            print(f"  enroll {nen:2d} images, threshold {thr}: FRR {(G < thr).mean() * 100:5.1f}%  "
                  f"FAR {(I >= thr).mean() * 100:.2f}%")


if __name__ == "__main__":
    main()
