from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class BatteryDrainConfig:
    """Configuration values used by the UAV LiHV battery discharge model."""

    # Nominal full-discharge time at reference conditions.
    tau_base_s: float = 2000.0

    # Speed used as the baseline for speed-dependent battery consumption.
    reference_speed_mps: float = 15.0

    # Larger values make high-speed flight consume battery more aggressively.
    speed_exponent: float = 1.3

    # Keeps a small minimum discharge load even when the UAV is moving slowly.
    min_speed_factor: float = 0.2

    # Mission-role weight. Larger weight means a shorter effective discharge time.
    role_weight: dict[str, float] = field(
        default_factory=lambda: {
            "recon": 1.40,
            "strike": 1.20,
            "decoy": 1.10,
        }
    )

    # WebPlotDigitizer anchor points from a manufacturer 1C LCO LiHV discharge curve.
    # x: discharge progress percent, y: single-cell voltage.
    voltage_curve_points: tuple[tuple[float, float], ...] = (
        (0.0, 4.283333333333333),
        (1.963636363636363, 4.252723232323232),
        (5.454545454545455, 4.208181818181818),
        (8.872727272727273, 4.163642424242424),
        (13.890909090909092, 4.102391919191919),
        (17.163636363636364, 4.068967676767676),
        (20.0, 4.035555555555555),
        (24.21818181818182, 3.988216161616161),
        (28.509090909090915, 3.9464303030303025),
        (35.12727272727274, 3.8823575757575752),
        (40.36363636363637, 3.8433232323232316),
        (47.41818181818182, 3.7959050505050502),
        (54.472727272727276, 3.7568202020202017),
        (60.14545454545455, 3.7288848484848476),
        (69.01818181818182, 3.6980828282828275),
        (75.12727272727273, 3.6784686868686864),
        (79.27272727272728, 3.664464646464646),
        (84.14545454545454, 3.64210707070707),
        (89.52727272727273, 3.6197353535353534),
        (92.07272727272728, 3.605775757575757),
        (93.38181818181819, 3.5918505050505045),
        (94.9090909090909, 3.555696969696969),
        (96.21818181818182, 3.505660606060606),
        (97.23636363636365, 3.4472989898989894),
        (98.18181818181819, 3.3556060606060596),
        (98.83636363636363, 3.2500323232323227),
        (99.27272727272728, 3.1527979797979793),
        (99.41818181818181, 3.069460606060606),
        (32.14545454545455, 3.907440404040403),
    )
