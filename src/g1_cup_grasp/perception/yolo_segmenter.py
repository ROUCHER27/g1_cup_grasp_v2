from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from ..domain import Detection


class YoloCupSegmenter:
    """Thin adapter around Ultralytics YOLO11 segmentation inference."""

    def __init__(
        self,
        model_path: str | Path,
        class_id: int = 0,
        class_name: str = "cup",
        imgsz: int = 640,
        device: str | int = 0,
    ) -> None:
        self.model_path = Path(model_path)
        if not self.model_path.exists():
            raise FileNotFoundError(
                f"YOLO weights not found: {self.model_path}. "
                "Train/export yolo11s-seg and copy the weights to models/."
            )
        try:
            from ultralytics import YOLO
        except ImportError as exc:  # pragma: no cover - depends on runtime
            raise RuntimeError("Install ultralytics before running YOLO inference") from exc
        self._model = YOLO(str(self.model_path))
        if self._model.task != "segment":
            raise ValueError(
                f"YOLO weights {self.model_path} have task {self._model.task!r}; "
                "use an instance segmentation model."
            )
        self.class_id = int(class_id)
        self.class_name = class_name
        self.imgsz = int(imgsz)
        self.device = device

    @staticmethod
    def _names_for(result: Any) -> dict[int, str]:
        names = getattr(result, "names", {})
        if isinstance(names, dict):
            return {int(k): str(v) for k, v in names.items()}
        return {index: str(value) for index, value in enumerate(names)}

    def predict(self, image_bgr: np.ndarray, confidence: float = 0.65) -> list[Detection]:
        if image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
            raise ValueError("YOLO input must be an HxWx3 BGR image")
        results = self._model.predict(
            source=image_bgr,
            conf=float(confidence),
            imgsz=self.imgsz,
            device=self.device,
            retina_masks=True,
            verbose=False,
        )
        if not results:
            return []
        result = results[0]
        boxes = getattr(result, "boxes", None)
        masks = getattr(result, "masks", None)
        if boxes is None or len(boxes) == 0:
            return []
        if masks is None:
            raise RuntimeError(
                "The segmentation model returned boxes without instance masks."
            )
        xyxy = boxes.xyxy.detach().cpu().numpy()
        scores = boxes.conf.detach().cpu().numpy()
        classes = boxes.cls.detach().cpu().numpy().astype(int)
        mask_data = masks.data.detach().cpu().numpy()
        names = self._names_for(result)

        detections: list[Detection] = []
        for index, class_id in enumerate(classes):
            if int(class_id) != self.class_id or float(scores[index]) < confidence:
                continue
            mask = mask_data[index].astype(np.uint8)
            if mask.shape != image_bgr.shape[:2]:
                import cv2

                mask = cv2.resize(
                    mask,
                    (image_bgr.shape[1], image_bgr.shape[0]),
                    interpolation=cv2.INTER_NEAREST,
                )
            label = names.get(int(class_id), self.class_name)
            detections.append(
                Detection(
                    class_id=int(class_id),
                    label=label,
                    confidence=float(scores[index]),
                    bbox_xyxy=tuple(float(value) for value in xyxy[index]),
                    mask=mask.astype(bool),
                )
            )
        return sorted(detections, key=lambda item: item.confidence, reverse=True)

