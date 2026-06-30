#!/usr/bin/env python3

from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
import yaml


class TrafficLightHsvParamGeneratorNode(Node):
    def __init__(self) -> None:
        super().__init__("traffic_light_hsv_param_generator_node")

        self.declare_parameter("green_dataset_path", "")
        self.declare_parameter("red_dataset_path", "")
        self.declare_parameter("saturation_threshold", 60)
        self.declare_parameter("value_threshold", 80)
        self.declare_parameter("bright_quantile", 0.8)
        self.declare_parameter("saturation_quantile", 0.5)
        self.declare_parameter("fallback_bright_quantile", 0.97)
        self.declare_parameter("fallback_saturation_quantile", 0.7)
        self.declare_parameter("center_weight_factor", 20.0)
        self.declare_parameter("yellow_hue_min", 16.0)
        self.declare_parameter("yellow_hue_max", 34.0)
        self.declare_parameter("hue_percentile_low", 10.0)
        self.declare_parameter("hue_percentile_high", 90.0)

        self.green_dataset_path = Path(
            self.get_parameter("green_dataset_path").value
        ).expanduser()
        self.red_dataset_path = Path(
            self.get_parameter("red_dataset_path").value
        ).expanduser()
        self.saturation_threshold = int(
            self.get_parameter("saturation_threshold").value
        )
        self.value_threshold = int(self.get_parameter("value_threshold").value)
        self.bright_quantile = float(self.get_parameter("bright_quantile").value)
        self.saturation_quantile = float(
            self.get_parameter("saturation_quantile").value
        )
        self.fallback_bright_quantile = float(
            self.get_parameter("fallback_bright_quantile").value
        )
        self.fallback_saturation_quantile = float(
            self.get_parameter("fallback_saturation_quantile").value
        )
        self.center_weight_factor = float(
            self.get_parameter("center_weight_factor").value
        )
        self.yellow_hue_min = float(self.get_parameter("yellow_hue_min").value)
        self.yellow_hue_max = float(self.get_parameter("yellow_hue_max").value)
        self.hue_percentile_low = float(
            self.get_parameter("hue_percentile_low").value
        )
        self.hue_percentile_high = float(
            self.get_parameter("hue_percentile_high").value
        )

    def run(self) -> int:
        if not self.green_dataset_path.exists():
            self.get_logger().error(
                f"green_dataset_path does not exist: {self.green_dataset_path}"
            )
            return 1
        if not self.red_dataset_path.exists():
            self.get_logger().error(
                f"red_dataset_path does not exist: {self.red_dataset_path}"
            )
            return 1

        green_samples = self._collect_samples(self.green_dataset_path, label=1)
        red_samples = self._collect_samples(self.red_dataset_path, label=0)
        samples = green_samples + red_samples

        if not green_samples or not red_samples:
            self.get_logger().error("Both green and red datasets must contain samples.")
            return 1

        green_hue_min, green_hue_max = self._compute_green_hue_range(green_samples)
        red_hue_low_max, red_hue_high_min = self._compute_red_hue_range(red_samples)

        thresholds = self._fit_thresholds(samples)
        yaml_output = {
            "traffic_light_hsv_roi_classifier_node": {
                "ros__parameters": {
                    "saturation_threshold": self.saturation_threshold,
                    "value_threshold": self.value_threshold,
                    "bright_quantile": self.bright_quantile,
                    "saturation_quantile": self.saturation_quantile,
                    "fallback_bright_quantile": self.fallback_bright_quantile,
                    "fallback_saturation_quantile": self.fallback_saturation_quantile,
                    "center_weight_factor": self.center_weight_factor,
                    "green_hue_min": green_hue_min,
                    "green_hue_max": green_hue_max,
                    "red_hue_low_max": red_hue_low_max,
                    "red_hue_high_min": red_hue_high_min,
                    "yellow_hue_min": self.yellow_hue_min,
                    "yellow_hue_max": self.yellow_hue_max,
                    "g_over_r_split_1": thresholds["g_over_r_split_1"],
                    "g_over_r_split_2": thresholds["g_over_r_split_2"],
                    "s_mean_split_1": thresholds["s_mean_split_1"],
                    "s_mean_split_2": thresholds["s_mean_split_2"],
                    "s_mean_split_3": thresholds["s_mean_split_3"],
                    "s_mean_split_4": thresholds["s_mean_split_4"],
                    "r_minus_g_split": thresholds["r_minus_g_split"],
                    "yellow_ratio_split_1": thresholds["yellow_ratio_split_1"],
                    "yellow_ratio_split_2": thresholds["yellow_ratio_split_2"],
                    "green_ratio_weight": 0.6,
                    "green_g_over_r_weight": 0.4,
                    "red_ratio_weight": 0.6,
                    "red_r_minus_g_weight": 0.4,
                    "green_g_over_r_norm": 2.0,
                    "red_r_minus_g_bias": 255.0,
                    "red_r_minus_g_norm": 510.0,
                    "label_score_floor": 0.55,
                    "opposite_score_ceiling": 0.45,
                }
            }
        }

        print(yaml.safe_dump(yaml_output, sort_keys=False), end="")
        return 0

    def _collect_samples(self, dataset_path: Path, label: int) -> list[dict]:
        image_paths = []
        for suffix in ("*.png", "*.jpg", "*.jpeg", "*.bmp", "*.tif", "*.tiff"):
            image_paths.extend(dataset_path.rglob(suffix))

        samples = []
        for image_path in sorted(image_paths):
            image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
            if image is None:
                continue
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            mask = self._build_signal_mask(hsv)
            if not np.any(mask):
                continue
            features, hue, weights = self._extract_feature_vector(image, hsv, mask)
            samples.append(
                {
                    "label": label,
                    "features": features,
                    "hue": hue,
                    "weights": weights,
                }
            )
        return samples

    def _build_prefilter_mask(self, hsv: np.ndarray) -> np.ndarray:
        s_channel, v_channel = hsv[:, :, 1], hsv[:, :, 2]
        mask = (s_channel > self.saturation_threshold) & (
            v_channel > self.value_threshold
        )
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_OPEN, kernel)
        return mask.astype(bool)

    def _build_signal_mask(self, hsv: np.ndarray) -> np.ndarray:
        base_mask = self._build_prefilter_mask(hsv)
        if not np.any(base_mask):
            return self._build_fallback_mask(hsv)

        value_channel = hsv[:, :, 2]
        saturation_channel = hsv[:, :, 1]
        bright_threshold = max(
            self.value_threshold,
            int(np.quantile(value_channel[base_mask], self.bright_quantile)),
        )
        saturation_threshold = max(
            self.saturation_threshold,
            int(np.quantile(saturation_channel[base_mask], self.saturation_quantile)),
        )
        signal_mask = (
            base_mask
            & (value_channel >= bright_threshold)
            & (saturation_channel >= saturation_threshold)
        )
        return signal_mask if np.any(signal_mask) else base_mask

    def _build_fallback_mask(self, hsv: np.ndarray) -> np.ndarray:
        saturation_channel = hsv[:, :, 1]
        value_channel = hsv[:, :, 2]
        bright_threshold = int(
            np.quantile(value_channel, self.fallback_bright_quantile)
        )
        saturation_threshold = int(
            np.quantile(saturation_channel, self.fallback_saturation_quantile)
        )
        fallback_mask = (value_channel >= bright_threshold) & (
            saturation_channel >= saturation_threshold
        )
        if np.any(fallback_mask):
            return fallback_mask
        return value_channel >= int(value_channel.max())

    def _extract_feature_vector(
        self, bgr_image: np.ndarray, hsv: np.ndarray, mask: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        y_coords, x_coords = np.where(mask)
        image_height, image_width = mask.shape
        center_x = (image_width - 1) / 2.0
        center_y = (image_height - 1) / 2.0
        distance = ((x_coords - center_x) / max(image_width, 1)) ** 2 + (
            (y_coords - center_y) / max(image_height, 1)
        ) ** 2
        weights = 1.0 / (1.0 + self.center_weight_factor * distance)

        hue = hsv[:, :, 0][y_coords, x_coords].astype(np.float32)
        saturation = hsv[:, :, 1][y_coords, x_coords].astype(np.float32)
        value = hsv[:, :, 2][y_coords, x_coords].astype(np.float32)
        bgr_pixels = bgr_image[y_coords, x_coords].astype(np.float32)
        green = bgr_pixels[:, 1]
        red = bgr_pixels[:, 2]

        hue_histogram = np.zeros(18, dtype=np.float32)
        hue_bins = np.clip((hue // 10).astype(np.int32), 0, 17)
        for hue_bin, weight in zip(hue_bins, weights):
            hue_histogram[hue_bin] += weight
        if hue_histogram.sum() > 0.0:
            hue_histogram /= hue_histogram.sum()

        green_ratio = np.average(
            ((hue >= 35.0) & (hue <= 95.0)).astype(np.float32), weights=weights
        )
        red_ratio = np.average(
            ((hue <= 15.0) | (hue >= 165.0)).astype(np.float32), weights=weights
        )
        yellow_ratio = np.average(
            ((hue >= 16.0) & (hue <= 34.0)).astype(np.float32), weights=weights
        )

        extra_features = np.array(
            [
                green_ratio,
                red_ratio,
                yellow_ratio,
                np.average(green - red, weights=weights),
                np.average(red - green, weights=weights),
                np.average(green / (red + 1.0), weights=weights),
                np.average(red / (green + 1.0), weights=weights),
                np.average(saturation, weights=weights),
                np.average(value, weights=weights),
            ],
            dtype=np.float32,
        )
        return np.concatenate([hue_histogram, extra_features]).astype(
            np.float32
        ), hue, weights

    def _weighted_percentile(
        self, values: np.ndarray, weights: np.ndarray, percentile: float
    ) -> float:
        order = np.argsort(values)
        sorted_values = values[order]
        sorted_weights = weights[order]
        cumulative = np.cumsum(sorted_weights)
        target = percentile / 100.0 * cumulative[-1]
        return float(sorted_values[np.searchsorted(cumulative, target, side="left")])

    def _compute_green_hue_range(self, samples: list[dict]) -> tuple[float, float]:
        hue = np.concatenate([sample["hue"] for sample in samples]).astype(np.float32)
        weights = np.concatenate([sample["weights"] for sample in samples]).astype(
            np.float32
        )
        low = self._weighted_percentile(hue, weights, self.hue_percentile_low)
        high = self._weighted_percentile(hue, weights, self.hue_percentile_high)
        return round(low, 2), round(high, 2)

    def _compute_red_hue_range(self, samples: list[dict]) -> tuple[float, float]:
        hue = np.concatenate([sample["hue"] for sample in samples]).astype(np.float32)
        weights = np.concatenate([sample["weights"] for sample in samples]).astype(
            np.float32
        )
        low_mask = hue <= 60.0
        high_mask = hue >= 120.0

        low_value = 15.0
        high_value = 165.0
        if np.any(low_mask):
            low_value = self._weighted_percentile(
                hue[low_mask], weights[low_mask], self.hue_percentile_high
            )
        if np.any(high_mask):
            high_value = self._weighted_percentile(
                hue[high_mask], weights[high_mask], self.hue_percentile_low
            )
        return round(low_value, 2), round(high_value, 2)

    def _candidate_values(
        self, values: np.ndarray, defaults: list[float], percentiles: list[int]
    ) -> list[float]:
        candidates = [float(v) for v in defaults]
        if values.size > 0:
            for percentile in percentiles:
                candidates.append(float(np.percentile(values, percentile)))
        return sorted({round(value, 4) for value in candidates})

    def _predict_label(self, features: np.ndarray, params: dict) -> int:
        yellow_ratio = float(features[20])
        r_minus_g = float(features[22])
        g_over_r = float(features[23])
        s_mean = float(features[25])

        if g_over_r <= params["g_over_r_split_1"]:
            if s_mean <= params["s_mean_split_1"]:
                return 1 if r_minus_g <= params["r_minus_g_split"] else 0
            if s_mean <= params["s_mean_split_2"]:
                if yellow_ratio <= params["yellow_ratio_split_1"]:
                    return 0 if s_mean <= params["s_mean_split_3"] else 1
                return 1
            return 0
        if s_mean <= params["s_mean_split_4"]:
            return 0
        if g_over_r <= params["g_over_r_split_2"]:
            return 0 if yellow_ratio <= params["yellow_ratio_split_2"] else 1
        return 1

    def _accuracy(self, samples: list[dict], params: dict) -> float:
        correct = 0
        for sample in samples:
            correct += int(self._predict_label(sample["features"], params) == sample["label"])
        return correct / max(len(samples), 1)

    def _fit_thresholds(self, samples: list[dict]) -> dict:
        feature_matrix = np.array([sample["features"] for sample in samples], dtype=np.float32)
        g_over_r_values = feature_matrix[:, 23]
        s_mean_values = feature_matrix[:, 25]
        yellow_ratio_values = feature_matrix[:, 20]
        r_minus_g_values = feature_matrix[:, 22]

        g_candidates = self._candidate_values(
            g_over_r_values, [1.03, 1.08], [15, 35, 50, 65, 85]
        )
        s_candidates = self._candidate_values(
            s_mean_values, [77.7, 110.5, 125.27, 125.61], [15, 35, 50, 65, 85]
        )
        y_candidates = self._candidate_values(
            yellow_ratio_values, [0.44, 0.78], [15, 35, 50, 65, 85]
        )
        r_candidates = self._candidate_values(
            r_minus_g_values, [48.41], [15, 35, 50, 65, 85]
        )

        best_params = None
        best_accuracy = -1.0

        for g_split_1 in g_candidates:
            for s_split_1 in s_candidates:
                low_left = [
                    sample
                    for sample in samples
                    if sample["features"][23] <= g_split_1
                    and sample["features"][25] <= s_split_1
                ]
                if not low_left:
                    continue
                best_r_split = self._best_binary_threshold(
                    low_left, r_candidates, feature_index=22, green_if_leq=True
                )

                for s_split_2 in s_candidates:
                    if s_split_2 < s_split_1:
                        continue

                    mid_left = [
                        sample
                        for sample in samples
                        if sample["features"][23] <= g_split_1
                        and s_split_1 < sample["features"][25] <= s_split_2
                    ]
                    if not mid_left:
                        continue
                    yellow_split_1, s_split_3 = self._best_mid_left_rule(
                        mid_left, y_candidates, s_candidates
                    )

                    for s_split_4 in s_candidates:
                        right_low = [
                            sample
                            for sample in samples
                            if sample["features"][23] > g_split_1
                            and sample["features"][25] <= s_split_4
                        ]
                        if not right_low:
                            continue

                        for g_split_2 in g_candidates:
                            if g_split_2 < g_split_1:
                                continue
                            right_mid = [
                                sample
                                for sample in samples
                                if sample["features"][23] > g_split_1
                                and sample["features"][25] > s_split_4
                                and sample["features"][23] <= g_split_2
                            ]
                            if not right_mid:
                                continue
                            yellow_split_2 = self._best_right_mid_rule(
                                right_mid, y_candidates
                            )

                            params = {
                                "g_over_r_split_1": round(g_split_1, 4),
                                "g_over_r_split_2": round(g_split_2, 4),
                                "s_mean_split_1": round(s_split_1, 4),
                                "s_mean_split_2": round(s_split_2, 4),
                                "s_mean_split_3": round(s_split_3, 4),
                                "s_mean_split_4": round(s_split_4, 4),
                                "r_minus_g_split": round(best_r_split, 4),
                                "yellow_ratio_split_1": round(yellow_split_1, 4),
                                "yellow_ratio_split_2": round(yellow_split_2, 4),
                            }
                            accuracy = self._accuracy(samples, params)
                            if accuracy > best_accuracy:
                                best_accuracy = accuracy
                                best_params = params

        if best_params is None:
            return {
                "g_over_r_split_1": 1.03,
                "g_over_r_split_2": 1.08,
                "s_mean_split_1": 110.5,
                "s_mean_split_2": 125.61,
                "s_mean_split_3": 125.27,
                "s_mean_split_4": 77.7,
                "r_minus_g_split": 48.41,
                "yellow_ratio_split_1": 0.78,
                "yellow_ratio_split_2": 0.44,
            }
        return best_params

    def _best_binary_threshold(
        self,
        samples: list[dict],
        candidates: list[float],
        feature_index: int,
        green_if_leq: bool,
    ) -> float:
        best_threshold = candidates[0]
        best_accuracy = -1.0
        for threshold in candidates:
            correct = 0
            for sample in samples:
                prediction = int(sample["features"][feature_index] <= threshold)
                if not green_if_leq:
                    prediction = 1 - prediction
                correct += int(prediction == sample["label"])
            accuracy = correct / len(samples)
            if accuracy > best_accuracy:
                best_accuracy = accuracy
                best_threshold = threshold
        return best_threshold

    def _best_mid_left_rule(
        self,
        samples: list[dict],
        yellow_candidates: list[float],
        s_candidates: list[float],
    ) -> tuple[float, float]:
        best_yellow = yellow_candidates[0]
        best_s = s_candidates[0]
        best_accuracy = -1.0
        for yellow_threshold in yellow_candidates:
            for s_threshold in s_candidates:
                correct = 0
                for sample in samples:
                    yellow_ratio = sample["features"][20]
                    s_mean = sample["features"][25]
                    if yellow_ratio <= yellow_threshold and s_mean <= s_threshold:
                        prediction = 0
                    else:
                        prediction = 1
                    correct += int(prediction == sample["label"])
                accuracy = correct / len(samples)
                if accuracy > best_accuracy:
                    best_accuracy = accuracy
                    best_yellow = yellow_threshold
                    best_s = s_threshold
        return best_yellow, best_s

    def _best_right_mid_rule(
        self, samples: list[dict], yellow_candidates: list[float]
    ) -> float:
        best_yellow = yellow_candidates[0]
        best_accuracy = -1.0
        for yellow_threshold in yellow_candidates:
            correct = 0
            for sample in samples:
                prediction = 0 if sample["features"][20] <= yellow_threshold else 1
                correct += int(prediction == sample["label"])
            accuracy = correct / len(samples)
            if accuracy > best_accuracy:
                best_accuracy = accuracy
                best_yellow = yellow_threshold
        return best_yellow


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrafficLightHsvParamGeneratorNode()
    exit_code = 1
    try:
        exit_code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
