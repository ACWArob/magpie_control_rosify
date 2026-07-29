#!/usr/bin/env python3
"""
Point cloud utilities for segmented object grasping.

Builds a world-frame point cloud from a SAM3 mask + RealSense depth image,
then computes centroid, principal axes, and gripper orientation via PCA.

References magpie_perception.pcd (correlllab/magpie_perception) for the full
6D PCA pose frame — use get_full_pose() when angled grasps are needed.
"""

import numpy as np


def build_segmented_pcd(mask, depth_mm, caminfo_k, tcp_matrix, tcp_to_cam):
    """
    Build a world-frame point cloud from a SAM3 mask using open3d's RGBD pipeline.

    Args:
        mask:        (H, W) bool   — SAM3 segmentation mask
        depth_mm:    (H, W) uint16 — RealSense depth in millimetres
        caminfo_k:   9-element flat camera intrinsics (CameraInfo.k)
        tcp_matrix:  (4, 4) TCP pose in world frame
        tcp_to_cam:  (4, 4) fixed TCP→camera extrinsic

    Returns:
        points: (N, 3) float64 XYZ array in world frame
        pcd:    open3d.geometry.PointCloud
    """
    import open3d as o3d

    fx, fy = caminfo_k[0], caminfo_k[4]
    cx, cy = caminfo_k[2], caminfo_k[5]

    ys, xs = np.where(mask)
    zs = depth_mm[ys, xs].astype(np.float64) / 1000.0  # mm → m

    valid = zs > 0
    xs, ys, zs = xs[valid], ys[valid], zs[valid]

    if len(zs) == 0:
        pcd = o3d.geometry.PointCloud()
        return np.zeros((0, 3)), pcd

    # Back-project each masked pixel to camera frame
    x_cam = (xs - cx) * zs / fx
    y_cam = (ys - cy) * zs / fy
    pts_cam = np.stack([x_cam, y_cam, zs, np.ones_like(zs)], axis=1)  # (N, 4)

    # Transform to world frame
    T = tcp_matrix @ tcp_to_cam
    pts_world = (T @ pts_cam.T).T[:, :3]  # (N, 3)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts_world)

    return pts_world, pcd


def denoise_pcd(pcd, nb_neighbors=30, std_ratio=0.5, hint=None,
                max_radius_m=0.12, z_pad_below=0.08, z_pad_above=0.03,
                bg_pts=None, bg_thresh_m=0.008):
    """
    Remove outliers and crop to object region.

    hint (world-frame object centroid XYZ):
      - XY crop to max_radius_m around hint in horizontal plane.
      - Z crop to [hint_z - z_pad_below, hint_z + z_pad_above].

    bg_pts (background cloud from measure_table, no object present):
      - Background subtraction: drop foreground points within bg_thresh_m
        of any background point. Useful for debugging segmentation quality
        in a fixed lab setup.
      - WARNING: do NOT use for VLA training data collection. At inference
        time the model won't have a background scan — using it creates a
        distribution shift between training data and deployment observations.
        The hint-based spatial crop already handles the table-leakage problem
        without this dependency.
    """
    import open3d as o3d

    pts = np.asarray(pcd.points)
    if hint is not None and len(pts) > 0:
        h = np.asarray(hint)
        xy_dist = np.linalg.norm(pts[:, :2] - h[:2], axis=1)
        z_ok    = (pts[:, 2] >= h[2] - z_pad_below) & (pts[:, 2] <= h[2] + z_pad_above)
        keep    = (xy_dist < max_radius_m) & z_ok
        if keep.sum() >= 10:
            pts = pts[keep]
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pts)

    if bg_pts is not None and len(pts) > 0 and len(bg_pts) > 0:
        # For each foreground point find its nearest background point.
        # Points close to background are table/environment — discard them.
        bg_pcd = o3d.geometry.PointCloud()
        bg_pcd.points = o3d.utility.Vector3dVector(np.asarray(bg_pts))
        fg_pcd = o3d.geometry.PointCloud()
        fg_pcd.points = o3d.utility.Vector3dVector(pts)
        dists = np.asarray(fg_pcd.compute_point_cloud_distance(bg_pcd))
        novel = dists > bg_thresh_m
        if novel.sum() >= 10:
            pts = pts[novel]
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pts)

    n = len(np.asarray(pcd.points))
    if n < nb_neighbors + 1:
        return pcd, np.asarray(pcd.points)

    cleaned, _ = pcd.remove_statistical_outlier(
        nb_neighbors=nb_neighbors, std_ratio=std_ratio
    )
    return cleaned, np.asarray(cleaned.points)


def check_view_quality(pts_cleaned, mask, depth_mm, min_points=80, min_fill=0.25):
    """
    Check whether the camera has a clean view of the object.

    Detects two obstruction modes:
      1. Too few valid depth pixels inside the SAM3 mask  (edge/cable blocking)
      2. Too few points surviving denoise  (sparse / noisy scan)

    Args:
        pts_cleaned: (N, 3) array after denoise_pcd
        mask:        (H, W) bool SAM3 mask
        depth_mm:    (H, W) uint16 RealSense depth
        min_points:  minimum denoised points to be considered usable
        min_fill:    minimum fraction of mask pixels with valid depth

    Returns dict:
        ok      bool   — True if view is good
        reason  str    — human-readable diagnosis
        n_pts   int    — denoised point count
        fill    float  — fraction of mask pixels with valid depth
    """
    mask_px = int(mask.sum())
    if mask_px == 0:
        return dict(ok=False, reason='mask is empty', n_pts=0, fill=0.0)

    valid_depth = (depth_mm[mask] > 0).sum()
    fill = float(valid_depth) / mask_px
    n_pts = len(pts_cleaned)

    if fill < min_fill:
        return dict(ok=False, reason=f'depth fill {fill:.0%} < {min_fill:.0%} — view blocked',
                    n_pts=n_pts, fill=fill)
    if n_pts < min_points:
        return dict(ok=False, reason=f'only {n_pts} pts after denoise — view blocked or object tiny',
                    n_pts=n_pts, fill=fill)

    return dict(ok=True, reason='ok', n_pts=n_pts, fill=fill)


def top_layer(pts, percentile=70):
    """Return only points in the top Z percentile — strips residual table layer.

    percentile=70 means keep top 30% by height. After denoise_pcd's spatial crop,
    the cloud should already be mostly object; this removes any remaining low points
    (table edge or mat) without over-cropping.

    Falls through if height range < 20mm (flat object — no stratification to remove).
    """
    if len(pts) == 0:
        return pts
    z = pts[:, 2]
    z_range = z.max() - z.min()
    if z_range < 0.020:
        return pts   # already a single layer, no filtering needed
    return pts[z >= np.percentile(z, percentile)]


def mask_grasp_angle(mask):
    """
    Compute grasp angle from the 2D SAM3 mask shape (image-space PCA).

    This is more robust than point-cloud PCA when the depth camera produces
    stripe artifacts (e.g. RealSense IR structured light on flat surfaces).
    Returns angle_deg in the same convention as analyse_pcd (0–180°).

    Args:
        mask: (H, W) bool — SAM3 segmentation mask

    Returns:
        angle_deg: float — PCA major axis angle (0–180°)
        ratio:     float — major/minor extent ratio
    """
    ys, xs = np.where(mask)
    if len(xs) < 10:
        return 0.0, 1.0
    pts2d = np.stack([xs, ys], axis=1).astype(np.float64)
    mean2d = pts2d.mean(axis=0)
    cov2d  = np.cov((pts2d - mean2d).T)
    eigvals, eigvecs = np.linalg.eigh(cov2d)
    idx = np.argsort(eigvals)[::-1]
    eigvals, eigvecs = eigvals[idx], eigvecs[:, idx]
    major_vec = eigvecs[:, 0]   # (dx_img, dy_img)
    # Image x → world X, image y → world Y (approximate for top-down view)
    # Negate dy because image Y increases downward, world Y increases away from robot
    angle_rad = np.arctan2(-major_vec[1], major_vec[0]) + np.pi / 2
    angle_deg = float(np.degrees(angle_rad) % 180)
    ratio = float(np.sqrt(eigvals[0] / max(eigvals[1], 1e-9)))
    return angle_deg, ratio


def analyse_pcd(pcd_or_points):
    """
    Compute centroid and principal axes (PCA) for grasp orientation.

    Follows magpie_perception.pcd.get_segment:
      - Uses open3d's compute_mean_and_covariance() for accuracy
      - Applies the -π/2 XY coordinate swap on centroid:
          grasp_pos = [Y, -X, Z]  (accounts for camera mount rotation)
      - get_full_pose() for the full 6D frame (angled grasps)

    Args:
        pcd_or_points: open3d PointCloud OR (N, 3) numpy array

    Returns dict:
        centroid        (3,)   grasp centroid in robot frame (XY-swapped)
        axes            (3, 3) eigenvectors sorted by variance descending
        eigenvalues     (3,)   variance along each axis
        extent_m        (3,)   object extent in metres [major, minor, normal]
        grasp_angle_deg float  wrist Z rotation (fingers perpendicular to major axis)
    """
    import open3d as o3d

    if isinstance(pcd_or_points, o3d.geometry.PointCloud):
        pcd = pcd_or_points
        points = np.asarray(pcd.points)
    else:
        points = np.asarray(pcd_or_points, dtype=np.float64)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

    if len(points) < 3:
        raise ValueError(f'Need at least 3 points for PCA, got {len(points)}')

    # Use open3d built-in — more accurate than np.cov on large point clouds
    mean, cov = pcd.compute_mean_and_covariance()

    # Points are already in world frame (transformed in build_segmented_pcd).
    # No coordinate swap needed — that only applies in mentor's wrist-frame pipeline.
    centroid = np.asarray(mean)

    eigenvalues, eigenvectors = np.linalg.eigh(cov)  # eigh: symmetric → real eigenvalues

    # Sort descending — largest variance (longest axis) first
    idx = np.argsort(eigenvalues)[::-1]
    eigenvalues  = eigenvalues[idx]
    eigenvectors = eigenvectors[:, idx]

    major_axis = eigenvectors[:, 0]

    # Gripper rotation: fingers close perpendicular to major axis in XY plane
    angle_rad = np.arctan2(major_axis[1], major_axis[0]) + np.pi / 2
    grasp_angle_deg = float(np.degrees(angle_rad) % 180)

    centered = points - mean  # use raw mean for extent, not swapped centroid
    projections = centered @ eigenvectors
    extent_m = projections.max(axis=0) - projections.min(axis=0)

    return dict(
        centroid=centroid,
        axes=eigenvectors,
        eigenvalues=eigenvalues,
        extent_m=extent_m,
        grasp_angle_deg=grasp_angle_deg,
    )


def get_full_pose(pcd_or_points):
    """
    Full 6D PCA pose frame following magpie_perception.pcd.get_segment.
    Handles axis alignment to world frame and right-hand rule enforcement.
    Use this for angled grasps where all three axes matter.

    Returns 4×4 transform with PCA axes as rotation + grasp centroid.
    """
    import io
    import contextlib
    import open3d as o3d
    from magpie_perception.pcd import get_pca_frame

    if isinstance(pcd_or_points, o3d.geometry.PointCloud):
        pcd = pcd_or_points
    else:
        points = np.asarray(pcd_or_points, dtype=np.float64)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

    mean, cov = pcd.compute_mean_and_covariance()

    # Suppress debug prints inside get_pca_frame
    with contextlib.redirect_stdout(io.StringIO()):
        _, tmat = get_pca_frame(np.array(mean), cov, scale=1.0)

    # Apply the same -π/2 XY centroid swap as get_segment
    tmat[:3, 3] = [mean[1], -mean[0], mean[2]]

    return tmat


def smart_grasp_angle(pca_result, object_name='', image_rgb=None,
                      gemini_client=None, gemini_model='gemini-2.5-flash',
                      mask=None):
    """
    Compute the best grasp angle using shape geometry + optional Gemini classification.

    All strategies grip ACROSS the narrow width (perpendicular to the long axis). The
    ANGLE comes from analyse_pcd's WORLD cloud angle for elongated objects (frame-correct,
    accounts for the camera clock); flat symmetric objects fall back to the 2D mask angle.
      symmetric  — cube/ball: any angle works (grips nearest flat face)
      short_side — rectangle/book/pen: grip perpendicular to the long axis
      long_side  — kept for compatibility (label only; also grips across)

    Steps:
      1. Geometry rule: if major/minor < 1.3 → symmetric
      2. Gemini override: classify shape for complex objects
      3. Angle source: mask 2D PCA (stripe-free) if mask provided, else point cloud PCA

    Returns (angle_deg, strategy, reason).
    """
    import cv2

    major = pca_result['extent_m'][0]
    minor = pca_result['extent_m'][1]
    ratio = major / max(minor, 1e-6)
    # analyse_pcd's grasp_angle_deg is the WORLD across-grasp (perpendicular to the
    # major axis). Its cloud is built with the full camera transform, so it already
    # accounts for the Rz(-90) camera mounting clock — the SAME world frame the wrist
    # / grasp_rotation_matrix use. This is the frame-correct angle for elongated objects.
    cloud_angle = pca_result['grasp_angle_deg']

    # Mask 2D PCA gives a clean RATIO (no IR stripe artifacts) and a fallback angle for
    # flat SYMMETRIC objects whose 3D PCA is stripe-degenerate. CAUTION: mask_grasp_angle
    # is in the raw CAMERA-IMAGE frame and does NOT apply the camera clock, so it is ~90°
    # off from world (measured -88.5° on real screwdriver/tube clouds). It must NOT drive
    # the angle of an elongated object — doing so was the "long objects grip end-to-end"
    # bug. Use it only for the ratio and the symmetric fallback.
    mask_angle = None
    if mask is not None:
        mask_angle, mask_ratio = mask_grasp_angle(mask)
        if ratio < 1.3:
            ratio = mask_ratio   # cleaner ratio for near-symmetric shapes

    # ── 1. Geometry default ───────────────────────────────────────────────────
    # If extents are too small the point cloud is noise (e.g. IR-opaque object
    # on a textured mat) — can't trust PCA angle, force symmetric.
    if major < 0.025:
        strategy = 'symmetric'
        reason   = f'geometry: extent too small ({major*1000:.0f}mm) — point cloud unreliable, forcing symmetric'
    elif ratio < 1.3:
        strategy = 'symmetric'
        reason   = f'geometry: ratio={ratio:.2f} < 1.3 (cube/cylinder)'
    else:
        strategy = 'short_side'
        reason   = f'geometry: ratio={ratio:.2f} (rectangle)'

    # ── 2. Gemini override ────────────────────────────────────────────────────
    if gemini_client is not None and image_rgb is not None and object_name:
        try:
            from google.genai import types as gtypes
            _, _buf = cv2.imencode('.jpg', cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR))
            img_bytes = _buf.tobytes()

            prompt = (
                f'Object: "{object_name}"\n'
                f'Point cloud extents: major={major*1000:.0f}mm, minor={minor*1000:.0f}mm '
                f'(ratio={ratio:.2f})\n\n'
                f'Choose the best gripper strategy — reply with EXACTLY one word:\n'
                f'  symmetric  — shape is round/square, any angle works (cube, ball, cylinder, bottle cap)\n'
                f'  short_side — grip perpendicular to longest dimension (box, book, phone, brick)\n'
                f'  long_side  — grip parallel to longest dimension (pen, banana, screwdriver, remote)\n'
            )
            r = gemini_client.models.generate_content(
                model=gemini_model,
                contents=[
                    gtypes.Part.from_bytes(data=img_bytes, mime_type='image/jpeg'),
                    prompt,
                ],
                config=gtypes.GenerateContentConfig(
                    thinking_config=gtypes.ThinkingConfig(thinking_budget=0)))
            word = r.text.strip().lower().split()[0]
            if word in ('symmetric', 'short_side', 'long_side'):
                reason = f'gemini: {word} (ratio={ratio:.2f})'
                strategy = word
        except Exception as e:
            reason += f' [gemini failed: {e}]'

    # ── 3. Apply strategy ─────────────────────────────────────────────────────
    # ROOT-CAUSE FIX (2026-07-28, proven on real screwdriver/tube clouds): long
    # objects gripped end-to-end because the elongated branch drove the wrist with the
    # MASK angle (raw image frame, measured ~89° off world) instead of analyse_pcd's
    # world-correct angle — and a downstream min(ang,90) clamp then mangled the rest.
    # analyse_pcd's cloud_angle IS the across-the-width WORLD grasp (perpendicular to
    # the major axis), so trust it for elongated objects. For a flat SYMMETRIC object
    # the 3D PCA is stripe-degenerate, so fall back to the 2D mask (its exact angle
    # barely matters — any face works — and the notebook's flat-face snap, which DOES
    # apply the camera clock, refines it).
    if strategy == 'symmetric':
        angle = mask_angle if mask_angle is not None else cloud_angle
    else:  # short_side / long_side — elongated: world cloud angle = grip ACROSS
        angle = cloud_angle

    angle = float(angle) % 180.  # normalise; the notebook wraps to [-90,90] (keeps direction)
    return angle, strategy, reason


def grasp_rotation_matrix(grasp_angle_deg):
    """
    3×3 rotation matrix for straight-down gripper approach with wrist rotated
    by grasp_angle_deg around world Z so fingers grip perpendicular to major axis.

    Derivation: start from base straight-down orientation (tool Z = world -Z,
    tool X = world +X), apply rotation by θ around world Z.

        R = [[cos θ,  sin θ,  0],
             [sin θ, -cos θ,  0],
             [0,      0,     -1]]

    θ=0 → fingers along world X, θ=90 → fingers along world Y.
    """
    theta = np.radians(grasp_angle_deg)
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [ c,  s, 0],
        [ s, -c, 0],
        [ 0,  0, -1],
    ])


def project_world_to_pixel(world_pts, tcp, tcp_to_cam, K_flat):
    """Project Nx3 world points back to (u, v) image pixels — inverse of the camera
    chain used in build_segmented_pcd. Accounts for the full tcp @ tcp_to_cam
    transform, so a world-frame grasp axis lands correctly on the camera image."""
    K = np.asarray(K_flat).ravel()
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    T    = np.asarray(tcp) @ np.asarray(tcp_to_cam)
    Tinv = np.linalg.inv(T)
    P    = np.atleast_2d(np.asarray(world_pts, dtype=float))
    Ph   = np.hstack([P, np.ones((len(P), 1))])
    cam  = (Tinv @ Ph.T).T[:, :3]
    z    = np.where(np.abs(cam[:, 2]) < 1e-6, 1e-6, cam[:, 2])
    u    = cx + cam[:, 0] * fx / z
    v    = cy + cam[:, 1] * fy / z
    return np.stack([u, v], axis=1)


def draw_grasp_on_image(image_rgb, cen_world, angle_deg, tcp, tcp_to_cam, K_flat,
                        half_len_m=0.04, center_px=None):
    """Draw the gripper closing axis onto the camera image.

    The grasp-axis DIRECTION comes from projecting the world grasp line (robust to
    translation calibration error — a relative direction). The CENTRE is anchored at
    `center_px` (the SAM3 mask centroid = where the object actually is in the image)
    when provided, so the overlay can't drift off the object due to scan-to-scan
    registration error. Green = closing axis, orange = finger faces, red = grip centre.
    """
    import cv2
    a = np.radians(angle_deg)
    d = np.array([np.cos(a), np.sin(a), 0.]) * half_len_m
    px = project_world_to_pixel(
        np.array([cen_world, cen_world - d, cen_world + d]), tcp, tcp_to_cam, K_flat)
    c_proj, p1_proj, p2_proj = px
    direction = p2_proj - p1_proj                       # image-space axis direction
    nrm = float(np.linalg.norm(direction))
    if nrm < 1e-6:
        direction, nrm = np.array([1., 0.]), 1.
    centre = np.asarray(center_px, float) if center_px is not None else np.asarray(c_proj, float)
    half   = direction / 2.
    p1, p2 = centre - half, centre + half
    perp   = np.array([-direction[1], direction[0]]) / nrm * 12.   # finger faces ⟂
    _i = lambda p: tuple(np.round(p).astype(int))
    vis = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR).copy()
    cv2.line(vis, _i(p1), _i(p2), (0, 255, 0), 3)                  # closing axis
    for endp in (p1, p2):
        cv2.line(vis, _i(endp - perp), _i(endp + perp), (0, 165, 255), 2)
    cv2.circle(vis, _i(centre), 5, (0, 0, 255), -1)               # grip centre
    return vis


def verify_grasp_angle_visual(image_rgb, cen_world, angle_deg, tcp, tcp_to_cam,
                              K_flat, object_name, gemini_client,
                              gemini_model='gemini-2.5-flash', half_len_m=0.04,
                              mask=None):
    """Render the grasp axis on the camera image and ask Gemini to confirm/correct it.

    The grasp-axis DIRECTION is projected from WORLD coords; the CENTRE is anchored at
    the SAM3 mask centroid (when `mask` given) so the overlay sits on the object even
    if scan-to-scan calibration drifts. Gemini's correction is a rotation applied in
    WORLD frame — no image→world angle ambiguity.

    Returns (corrected_angle_deg, verdict, reason, annotated_bgr).
    """
    import cv2
    from google.genai import types as gtypes

    center_px = None
    if mask is not None and np.asarray(mask).any():
        _ys, _xs = np.where(np.asarray(mask))
        center_px = (float(_xs.mean()), float(_ys.mean()))

    vis = draw_grasp_on_image(image_rgb, cen_world, angle_deg,
                              tcp, tcp_to_cam, K_flat, half_len_m, center_px=center_px)
    ok, buf = cv2.imencode('.jpg', vis)
    prompt = (
        f'The GREEN line is the axis the two-finger gripper will CLOSE along to grasp '
        f'the "{object_name}". Orange ticks = finger contact faces, red dot = grip centre.\n'
        f'A GOOD grasp line is PARALLEL to the object two flat opposing faces '
        f'(perpendicular to its edges), closing across its narrowest width.\n'
        f'Judge the PRECISE alignment with the object actual orientation. If the line '
        f'is slightly off the faces, give the small rotation that makes it parallel to '
        f'them — this is usually only a FEW degrees to match how the object is sitting. '
        f'Do NOT snap to 90; only use a large rotation if the line is genuinely across '
        f'the wrong (widest/diagonal) dimension.\n'
        f'Reply in EXACTLY this format:\n'
        f'VERDICT: <GOOD|ROTATE>\n'
        f'ROTATE_DEG: <precise signed degrees to rotate the green line so it is parallel '
        f'to the faces, -90 to 90; 0 if already aligned>\n'
        f'REASON: <one short sentence>'
    )
    try:
        txt = gemini_client.models.generate_content(
            model=gemini_model,
            contents=[gtypes.Part.from_bytes(data=buf.tobytes(),
                                             mime_type='image/jpeg'), prompt],
            config=gtypes.GenerateContentConfig(
                thinking_config=gtypes.ThinkingConfig(thinking_budget=0))).text
    except Exception as e:
        return float(angle_deg), 'skipped', f'gemini failed: {e}', vis

    verdict, rot, reason = 'GOOD', 0., ''
    for line in txt.splitlines():
        L = line.strip()
        up = L.upper()
        if up.startswith('VERDICT:'):
            verdict = L.split(':', 1)[1].strip().upper()
        elif up.startswith('ROTATE_DEG:'):
            try:
                rot = float(L.split(':', 1)[1].strip().split()[0])
            except Exception:
                rot = 0.
        elif up.startswith('REASON:'):
            reason = L.split(':', 1)[1].strip()

    corrected = float((angle_deg + (rot if verdict.startswith('ROTATE') else 0.)) % 180.)
    vis2 = draw_grasp_on_image(image_rgb, cen_world, corrected,
                               tcp, tcp_to_cam, K_flat, half_len_m, center_px=center_px)
    return corrected, verdict, reason, vis2


def compare_grasp_angles_visual(image_rgb, cen_world, angle_a_deg, angle_b_deg,
                                tcp, tcp_to_cam, K_flat, object_name, gemini_client,
                                label_a='A', label_b='B', gemini_model='gemini-2.5-flash',
                                half_len_m=0.04, mask=None):
    """Draw TWO candidate gripper closing axes (A and B) and let Gemini ARBITRATE.

    Use when two planners disagree (e.g. PCA vs GraspGenX): PCA is unreliable on a
    near-square top, GraspGenX is starved by a flat single-view cloud, so neither is
    trustworthy alone. Gemini looks at the real object and picks the line that grips
    the narrowest width across two flat faces, with a small parallel correction.

    Returns (chosen_angle_deg, chosen_label, reason, annotated_bgr).
    """
    import cv2
    from google.genai import types as gtypes

    center_px = None
    if mask is not None and np.asarray(mask).any():
        _ys, _xs = np.where(np.asarray(mask))
        center_px = (float(_xs.mean()), float(_ys.mean()))

    def _axis_pts(angle):
        a = np.radians(angle)
        d = np.array([np.cos(a), np.sin(a), 0.]) * half_len_m
        return project_world_to_pixel(
            np.array([cen_world, cen_world - d, cen_world + d]), tcp, tcp_to_cam, K_flat)

    _i = lambda p: tuple(np.round(np.asarray(p)).astype(int))
    vis = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR).copy()
    centre_px = np.asarray(center_px, float) if center_px is not None else None
    colours = {'A': (0, 255, 0), 'B': (255, 0, 0)}           # A green, B blue (BGR)
    for lbl, ang in (('A', angle_a_deg), ('B', angle_b_deg)):
        c_proj, p1_proj, p2_proj = _axis_pts(ang)
        direction = p2_proj - p1_proj
        nrm = float(np.linalg.norm(direction)) or 1.
        ctr = centre_px if centre_px is not None else np.asarray(c_proj, float)
        half = direction / 2.
        a1, a2 = ctr - half, ctr + half
        cv2.line(vis, _i(a1), _i(a2), colours[lbl], 3)
        cv2.putText(vis, lbl, _i(a2 + direction / nrm * 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, colours[lbl], 2)
    if centre_px is not None:
        cv2.circle(vis, _i(centre_px), 5, (0, 0, 255), -1)

    ok, buf = cv2.imencode('.jpg', vis)
    prompt = (
        f'Two candidate gripper CLOSING axes are drawn over the "{object_name}": '
        f'line A (GREEN, {angle_a_deg:.0f} deg) and line B (BLUE, {angle_b_deg:.0f} deg). '
        f'The two-finger gripper closes ALONG the chosen line, its fingers pressing the '
        f'two object faces that the line points at.\n'
        f'The BETTER grasp closes across the object NARROWEST width, with the line PARALLEL '
        f'to two flat opposing faces (perpendicular to the long edges). For a near-square '
        f'top either axis can work — pick the one most parallel to a real pair of faces.\n'
        f'Reply in EXACTLY this format:\n'
        f'CHOICE: <A|B>\n'
        f'ROTATE_DEG: <small signed deg to rotate the chosen line parallel to the faces, '
        f'-90 to 90; 0 if already aligned>\n'
        f'REASON: <one short sentence>'
    )
    try:
        txt = gemini_client.models.generate_content(
            model=gemini_model,
            contents=[gtypes.Part.from_bytes(data=buf.tobytes(),
                                             mime_type='image/jpeg'), prompt],
            config=gtypes.GenerateContentConfig(
                thinking_config=gtypes.ThinkingConfig(thinking_budget=0))).text
    except Exception as e:
        return float(angle_a_deg), label_a, f'gemini failed: {e}', vis

    choice, rot, reason = 'A', 0., ''
    for line in txt.splitlines():
        L = line.strip(); up = L.upper()
        if up.startswith('CHOICE:'):
            c = L.split(':', 1)[1].strip().upper()
            choice = 'B' if c.startswith('B') else 'A'
        elif up.startswith('ROTATE_DEG:'):
            try:
                rot = float(L.split(':', 1)[1].strip().split()[0])
            except Exception:
                rot = 0.
        elif up.startswith('REASON:'):
            reason = L.split(':', 1)[1].strip()

    base = angle_a_deg if choice == 'A' else angle_b_deg
    chosen_label = label_a if choice == 'A' else label_b
    chosen = float((base + rot) % 180.)
    vis2 = draw_grasp_on_image(image_rgb, cen_world, chosen,
                               tcp, tcp_to_cam, K_flat, half_len_m, center_px=center_px)
    return chosen, chosen_label, reason, vis2


def rank_grasp_angles_visual(image_rgb, cen_world, candidates, tcp, tcp_to_cam, K_flat,
                              object_name='object', gemini_client=None, gemini_model='gemini-2.5-flash',
                              half_len_m=0.06, mask=None):
    """Show up to 4 angle candidates as labelled lines and let Gemini rank/pick the best.

    candidates : list of (label_str, angle_deg) e.g.
                 [('PCA-short', 30.), ('PCA-long', 120.), ('GGX-1', 45.), ('GGX-2', 80.)]
                 All angles should already be wrist-clamped (0–90°).

    Returns (chosen_angle_deg, chosen_label, reason, annotated_bgr).
    """
    import cv2
    from google.genai import types as gtypes

    LETTERS = ['A', 'B', 'C', 'D']
    cands = list(candidates[:4])

    center_px = None
    if mask is not None and np.asarray(mask).any():
        _ys, _xs = np.where(np.asarray(mask))
        center_px = (float(_xs.mean()), float(_ys.mean()))

    # Render EACH candidate as its own clean image (green closing axis + orange finger
    # ticks), instead of overlapping thin lines on one frame — Gemini was picking
    # "blindly" because 4 lines on a small object are unreadable. Separate annotated
    # images per option let it actually compare where the fingers land.
    parts = []
    for i, (lbl, ang) in enumerate(cands):
        _img = draw_grasp_on_image(image_rgb, cen_world, ang, tcp, tcp_to_cam, K_flat,
                                   half_len_m=half_len_m, center_px=center_px)
        _, _buf = cv2.imencode('.jpg', _img)
        parts.append(gtypes.Part.from_bytes(data=_buf.tobytes(), mime_type='image/jpeg'))
        parts.append(f'Image {i + 1} = option {LETTERS[i]} ({ang:.0f}°).')

    prompt = (
        f'Each image shows ONE candidate grasp for the "{object_name}". The GREEN line is '
        f'the two-finger gripper CLOSING axis; the ORANGE ticks mark where the two fingers '
        f'press. A GOOD grasp closes across the NARROWEST span onto two FLAT opposing faces. '
        f'A BAD grasp runs diagonally into corners, or across the widest span. '
        f'For a square/round top, prefer a grasp square to a pair of faces, never a diagonal. '
        f'If several options are equally face-aligned, any of them is fine — just never pick a diagonal.\n'
        f'Reply EXACTLY:\n'
        f'CHOICE: <{"|".join(LETTERS[:len(cands)])}>\n'
        f'ROTATE_DEG: <signed degrees to fine-tune, -15 to 15; 0 if already aligned>\n'
        f'REASON: <one sentence>'
    )
    try:
        # thinking_budget=0: this was taking ~40s with extended reasoning that wasn't
        # improving the pick. Disable it → ~2-3s. The clearer per-option images carry
        # the accuracy now, not slow chain-of-thought.
        txt = gemini_client.models.generate_content(
            model=gemini_model,
            contents=parts + [prompt],
            config=gtypes.GenerateContentConfig(
                thinking_config=gtypes.ThinkingConfig(thinking_budget=0))).text
    except Exception as e:
        _fb = draw_grasp_on_image(image_rgb, cen_world, cands[0][1], tcp, tcp_to_cam,
                                  K_flat, half_len_m=half_len_m, center_px=center_px)
        return float(cands[0][1]), cands[0][0], f'gemini failed: {e}', _fb

    choice_idx, rot, reason = 0, 0., ''
    for line in txt.splitlines():
        L = line.strip(); up = L.upper()
        if up.startswith('CHOICE:'):
            c = L.split(':', 1)[1].strip().upper()
            for k, letter in enumerate(LETTERS[:len(cands)]):
                if c.startswith(letter):
                    choice_idx = k; break
        elif up.startswith('ROTATE_DEG:'):
            try: rot = float(L.split(':', 1)[1].strip().split()[0])
            except Exception: rot = 0.
        elif up.startswith('REASON:'):
            reason = L.split(':', 1)[1].strip()

    chosen_lbl = cands[choice_idx][0]
    chosen_ang = float((cands[choice_idx][1] + rot) % 180.)
    vis2 = draw_grasp_on_image(image_rgb, cen_world, chosen_ang,
                               tcp, tcp_to_cam, K_flat, half_len_m, center_px=center_px)
    return chosen_ang, chosen_lbl, reason, vis2


def augment_cloud_for_graspgenx(pts, table_z, n_side=400, n_bottom=200,
                                extra_height_m=0.0, seed=0):
    """Augment a flat top-down object cloud with synthetic side + bottom faces.

    GraspGenX was trained on partial but 3D rendered clouds (multiple viewpoints).
    A single top-down scan gives only the top face — a flat disk that starves the
    model of shape info and pushes grasps to the rim.  This function fits a 2D OBB
    (via PCA on the XY footprint), infers object height from TABLE_Z, then samples
    uniform points on the 4 side faces and bottom, giving GraspGenX a realistic 3D
    volume that matches its training distribution.

    Args:
        pts:            (N, 3) float — real top-face points in world frame
        table_z:        float — world-frame Z of the table surface (TABLE_Z constant)
        n_side:         int — total synthetic points across all 4 side faces (100 per face)
        n_bottom:       int — synthetic points on the bottom face
        extra_height_m: float — extend the bottom this many metres below table_z.
                        Use CAMERA_MOUNT_Z_OFFSET_M to correct for camera mount error
                        (camera reads the floor ~24mm high), giving GraspGenX the true
                        object height without touching arm movement constants.
        seed:           int — RNG seed for reproducibility

    Returns:
        (M, 3) float32 — augmented cloud (real pts stacked with synthetic faces)
    """
    pts = np.asarray(pts, dtype=np.float64)
    if len(pts) < 6:
        return pts.astype(np.float32)

    rng = np.random.default_rng(seed)

    # 2D PCA on the horizontal footprint → OBB axes in XY
    xy = pts[:, :2]
    mu_xy = xy.mean(axis=0)
    cov_xy = np.cov((xy - mu_xy).T)
    vals, vecs = np.linalg.eigh(cov_xy)
    axes_xy = vecs[:, np.argsort(vals)[::-1]]   # (2,2) columns = principal XY axes

    # Object extents in the OBB frame
    local_xy = (xy - mu_xy) @ axes_xy            # (N, 2)
    lo = local_xy.min(axis=0)
    hi = local_xy.max(axis=0)

    z_top = pts[:, 2].max()
    z_bot = table_z - extra_height_m             # extend below camera-measured floor

    synth = []
    per_face = max(n_side // 4, 1)

    # 4 side faces: fix one OBB horizontal dim, free the other + world Z
    for face_dim in (0, 1):
        other = 1 - face_dim
        for val in (lo[face_dim], hi[face_dim]):
            local_f = np.zeros((per_face, 2))
            local_f[:, face_dim] = val
            local_f[:, other] = rng.uniform(lo[other], hi[other], per_face)
            world_xy = local_f @ axes_xy.T + mu_xy
            world_z  = rng.uniform(z_bot, z_top, per_face)
            synth.append(np.column_stack([world_xy, world_z]))

    # Bottom face: full XY footprint at z_bot (true floor estimate)
    local_b = np.column_stack([
        rng.uniform(lo[0], hi[0], n_bottom),
        rng.uniform(lo[1], hi[1], n_bottom),
    ])
    world_b_xy = local_b @ axes_xy.T + mu_xy
    synth.append(np.column_stack([world_b_xy, np.full(n_bottom, z_bot)]))

    return np.vstack([pts, *synth]).astype(np.float32)


def print_pcd_results(result, n_points):
    """Pretty-print analyse_pcd output."""
    c = result['centroid']
    e = result['extent_m']
    axes = result['axes']

    print(f'\n=== POINT CLOUD ANALYSIS ===')
    print(f'  Points (after denoise): {n_points}')
    print(f'  Centroid (world):  x={c[0]:.3f}  y={c[1]:.3f}  z={c[2]:.3f} m')
    print(f'  Extent:   major={e[0]*100:.1f} cm  minor={e[1]*100:.1f} cm  depth={e[2]*100:.1f} cm')
    print(f'  Major axis direction: [{axes[0,0]:.2f}, {axes[1,0]:.2f}, {axes[2,0]:.2f}]')
    print(f'  Grasp angle (wrist Z): {result["grasp_angle_deg"]:.1f} deg')
    print(f'============================')
