"""Animate the README figure from a folded-chain run of physics_runner.

The CSV was produced by:

    build/physics_runner --example folded-chain --bending 40 --duration 0.7 --integrator adaptive --rtol 1e-10 --atol 1e-10 --dt 0.0005 --csv folded-chain.csv

Then:

    python docs/images/readme_figure.py folded-chain.csv

writes docs/images/folded-chain.gif in slow motion: the chain beside a ball
released level with its free end, which follows the exact free-fall solution
y = y0 - g t^2 / 2. Live graphs show the free end's height against that parabola
and |energy-balance error| on a log scale. Requires numpy and matplotlib
(Pillow, which writes the GIF, is a matplotlib dependency).
"""

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from PIL import Image

G = 9.81
FPS = 20
SPEED = 0.05  # slow motion: one GIF second shows 0.05 s
HOLD_SECONDS = 2.0
TRAIL_SECONDS = 0.06
BALL_OFFSET = 0.4  # m to the right of the free end
COLORS = 64
# Each link is drawn as a steel ring: alternately face-on (an oval) and edge-on
# (a bar), each a little longer than the link so neighbours interlock.
STEEL_DARK = "#3d434b"
STEEL_LIGHT = "#b4bcc6"
RING_LENGTH = 1.24  # times the link length
RING_WIDTH = 0.6
TIP = "#eb6834"   # categorical slot 2
BALL = "#2a78d6"  # categorical slot 1
INK = "#2b2b2b"
MUTED_INK = "#6b6b6b"
GUIDE = "#c8c8c8"


def style(axes):
    axes.tick_params(colors=MUTED_INK, labelsize=9)
    axes.grid(True, which="major", color="#ececec", linewidth=0.7)
    for side in ("top", "right"):
        axes.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        axes.spines[side].set_color("#bdbdbd")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("csv", type=Path, help="physics_runner CSV output")
    parser.add_argument("--output", type=Path,
                        default=Path(__file__).with_name("folded-chain.gif"))
    args = parser.parse_args()

    data = np.genfromtxt(args.csv, delimiter=",", names=True)
    count = int(data["particle"].max()) + 1
    # Arrays indexed [sample, particle], with the pivot prepended.
    time = data["time"].reshape(-1, count)[:, 0]
    tip_y = data["y"].reshape(-1, count)[:, -1]
    # End when the ball reaches the lowest point of the free end's fall.
    lowest = tip_y.min()
    arrival = time[np.argmin(tip_y)]
    ball_arrival = np.sqrt(2 * (tip_y[0] - lowest) / G)
    keep = time <= ball_arrival + 1e-3
    time = time[keep]
    x = np.hstack([np.zeros((len(time), 1)), data["x"].reshape(-1, count)[keep]])
    y = np.hstack([np.zeros((len(time), 1)), data["y"].reshape(-1, count)[keep]])
    error = np.abs(data["energy_balance_error"].reshape(-1, count)[keep, 0])

    free_fall = y[0, -1] - 0.5 * G * time**2
    ball_x = x[0, -1] + BALL_OFFSET
    ball_y = np.maximum(free_fall, lowest)

    frames = np.searchsorted(time, np.arange(0.0, time[-1] + 1e-9, SPEED / FPS))
    trail_samples = int(np.searchsorted(time, TRAIL_SECONDS))
    link = np.hypot(x[0, 1] - x[0, 0], y[0, 1] - y[0, 0])
    ring = np.linspace(0.0, 2 * np.pi, 25)

    # Left: the chain, at a fixed scale; right: the two graphs.
    margin = 0.15
    side = 0.35
    xmin, xmax = min(x.min(), ball_x) - side, max(x.max(), ball_x) + side
    ymin, ymax = lowest - margin, max(y.max(), 0.0) + margin
    height, dpi = 8.0, 100  # 800 px
    chain_width = height * (xmax - xmin) / (ymax - ymin)
    graph_width = 5.6
    width = chain_width + graph_width
    fig = plt.figure(figsize=(width, height), dpi=dpi, facecolor="white")
    points_per_metre = height * 72 / (ymax - ymin)

    ax = fig.add_axes([0, 0, chain_width / width, 1])
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_aspect("equal")
    ax.axis("off")

    # The level the free end reaches when the chain is straight.
    ax.axhline(lowest, color=GUIDE, linewidth=1.0, zorder=1)
    trail = LineCollection([], linewidths=2.0, capstyle="butt", zorder=2)
    ax.add_collection(trail)
    # Each ring is a dark stroke under a light one, which reads as round wire.
    dark, light = 0.16 * link * points_per_metre, 0.07 * link * points_per_metre
    faces = [LineCollection([], colors=color, linewidths=width, capstyle="round",
                            joinstyle="round", zorder=3 + k)
             for k, (color, width) in enumerate([(STEEL_DARK, dark), (STEEL_LIGHT, light)])]
    bars = [LineCollection([], colors=color, linewidths=width, capstyle="round", zorder=5 + k)
            for k, (color, width) in enumerate([(STEEL_DARK, 1.4 * dark),
                                                (STEEL_LIGHT, 1.4 * light)])]
    for collection in faces + bars:
        ax.add_collection(collection)
    (tip,) = ax.plot([], [], "o", color=TIP, markersize=22, markeredgecolor="white",
                     markeredgewidth=2.0, zorder=8)
    (ball,) = ax.plot([], [], "o", color=BALL, markersize=22, markeredgecolor="white",
                      markeredgewidth=2.0, zorder=8)
    # The pivot is at the origin.
    ax.plot(0.0, 0.0, "o", color=INK, markersize=12, zorder=9)
    ax.plot(0.0, 0.0, "o", color="white", markersize=4.5, zorder=10)

    left = (chain_width + 0.85) / width
    graph_span = (graph_width - 1.05) / width

    # Top right: height of the free end against the free-fall parabola.
    heights = fig.add_axes([left, 0.555, graph_span, 0.38])
    style(heights)
    heights.set_xlim(0.0, time[-1])
    heights.set_ylim(lowest - 0.15, y[0, -1] + 0.1)
    heights.set_ylabel("height (m)", color=MUTED_INK, fontsize=10)
    heights.tick_params(labelbottom=False)
    heights.axhline(lowest, color=GUIDE, linewidth=1.0, zorder=1)
    (fall_line,) = heights.plot([], [], color=BALL, linewidth=2.0,
                                label="free fall, y₀ − gt²/2")
    (tip_line,) = heights.plot([], [], color=TIP, linewidth=2.0, label="chain's free end")
    (fall_head,) = heights.plot([], [], "o", color=BALL, markersize=7, markeredgecolor="white",
                                markeredgewidth=1.2, clip_on=False, zorder=4)
    (tip_head,) = heights.plot([], [], "o", color=TIP, markersize=7, markeredgecolor="white",
                               markeredgewidth=1.2, clip_on=False, zorder=4)
    legend = heights.legend(loc="lower left", bbox_to_anchor=(0.0, 0.1), frameon=False,
                            fontsize=9.5, handlelength=1.6)
    for text in legend.get_texts():
        text.set_color(INK)
    # Arrival at the lowest level, revealed when it happens.
    marks = []
    for when, color in ((arrival, TIP), (ball_arrival, BALL)):
        line = heights.axvline(when, color=color, linewidth=1.0, linestyle=(0, (3, 3)),
                               visible=False)
        label = heights.text(when - 0.005, y[0, -1], f"{when:.3f} s", color=INK, fontsize=9,
                             ha="right", va="top", visible=False)
        marks.append((when, line, label))

    # Bottom right: |energy-balance error| on a log scale.
    errors = fig.add_axes([left, 0.085, graph_span, 0.38])
    style(errors)
    shown = error > 0
    errors.set_xlim(0.0, time[-1])
    errors.set_yscale("log")
    errors.set_ylim(10 ** np.floor(np.log10(error[shown].min())),
                    10 ** np.ceil(np.log10(error.max())))
    errors.yaxis.set_minor_locator(matplotlib.ticker.NullLocator())
    errors.set_xlabel("time (s)", color=MUTED_INK, fontsize=10)
    errors.set_ylabel("|energy-balance error| (J)", color=MUTED_INK, fontsize=10)
    (error_line,) = errors.plot([], [], color=INK, linewidth=1.4)
    (error_head,) = errors.plot([], [], "o", color=INK, markersize=6, markeredgecolor="white",
                                markeredgewidth=1.0, clip_on=False, zorder=4)

    def draw(i):
        start = max(0, i - trail_samples)
        path = np.column_stack([x[start:i + 1, -1], y[start:i + 1, -1]])
        segments = np.stack([path[:-1], path[1:]], axis=1)
        trail_colors = np.tile(matplotlib.colors.to_rgba(TIP), (len(segments), 1))
        trail_colors[:, 3] = 0.7 * np.linspace(0.0, 1.0, len(segments)) ** 2
        trail.set_segments(segments)
        trail.set_color(trail_colors)

        points = np.column_stack([x[i], y[i]])
        centres = 0.5 * (points[:-1] + points[1:])
        along = points[1:] - points[:-1]
        along /= np.linalg.norm(along, axis=1, keepdims=True)
        across = np.column_stack([-along[:, 1], along[:, 0]])
        half = 0.5 * RING_LENGTH * link
        ovals = (centres[:, None, :]
                 + half * np.cos(ring)[None, :, None] * along[:, None, :]
                 + 0.5 * RING_WIDTH * link * np.sin(ring)[None, :, None] * across[:, None, :])
        edges = np.stack([centres - half * along, centres + half * along], axis=1)
        for collection in faces:
            collection.set_segments(ovals[0::2])
        for collection in bars:
            collection.set_segments(edges[1::2])
        tip.set_data([x[i, -1]], [y[i, -1]])
        ball.set_data([ball_x], [ball_y[i]])

        tip_line.set_data(time[:i + 1], y[:i + 1, -1])
        tip_head.set_data([time[i]], [y[i, -1]])
        fall_line.set_data(time[:i + 1], ball_y[:i + 1])
        fall_head.set_data([time[i]], [ball_y[i]])
        for when, line, label in marks:
            line.set_visible(time[i] >= when - 1e-9)
            label.set_visible(time[i] >= when - 1e-9)
        recent = shown[:i + 1]
        error_line.set_data(time[:i + 1][recent], error[:i + 1][recent])
        error_head.set_data([time[i]] if shown[i] else [], [error[i]] if shown[i] else [])

        fig.canvas.draw()
        return np.asarray(fig.canvas.buffer_rgba())[..., :3].copy()

    rgb_frames = [draw(i) for i in frames]
    # One palette for every frame lets the GIF store only what changes.
    montage = np.concatenate(rgb_frames[::len(rgb_frames) // 16 or 1])
    quantized = Image.fromarray(montage).quantize(COLORS)
    palette = np.array(quantized.getpalette(), dtype=np.int32).reshape(-1, 3)
    palette = palette[:quantized.getextrema()[1] + 1]
    # Keep the background pure white, not the average of its nearest colours.
    palette[np.argmin(((palette - 255) ** 2).sum(axis=1))] = 255

    def indexed(rgb):
        rgb = rgb.astype(np.int32)
        packed = (rgb[..., 0] << 16) | (rgb[..., 1] << 8) | rgb[..., 2]
        unique, inverse = np.unique(packed, return_inverse=True)
        unique_rgb = np.stack([unique >> 16, (unique >> 8) & 255, unique & 255], axis=1)
        nearest = ((unique_rgb[:, None, :] - palette) ** 2).sum(axis=2).argmin(axis=1)
        image = Image.fromarray(nearest[inverse].reshape(rgb.shape[:2]).astype(np.uint8), "P")
        image.putpalette(palette.astype(np.uint8).ravel().tolist())
        return image

    images = [indexed(rgb) for rgb in rgb_frames]
    # Hold the last frame before the loop restarts.
    durations = [1000 // FPS] * (len(images) - 1) + [int(1000 * HOLD_SECONDS)]
    images[0].save(args.output, save_all=True, append_images=images[1:],
                   duration=durations, loop=0)

    print(f"wrote {args.output}: {len(images)} frames")
    print(f"free end reaches its lowest point, y = {lowest:.3f} m, at t = {arrival:.3f} s; "
          f"free fall reaches it at t = {ball_arrival:.3f} s")
    print(f"max |energy-balance error| = {error.max():.3g} J")


if __name__ == "__main__":
    main()
