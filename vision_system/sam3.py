import cv2
import numpy as np
from ultralytics.models.sam import SAM3SemanticPredictor

CLASSES = ['wheat grain', 'black seed', 'pea grain']
COLORS = {CLASSES[0]: (255, 0, 0), CLASSES[1]: (0, 255, 0), CLASSES[2]: (0, 0, 255)}


def segment(img_path, draw=False):
    """ Promptable concept segmentation with Segment Anything Model 3 (SAM 3) """
    overrides = dict(
        conf=0.25, task='segment', mode='predict', model='weights/sam3.1_multiplex.pt', half=False, save=False
    )
    predictor = SAM3SemanticPredictor(overrides=overrides)
    predictor.set_image(img_path)
    results = predictor(text=CLASSES)

    img = cv2.imread(img_path)
    h, w = img.shape[:2]

    masks_by_class = {}
    result = results[0]
    if result.masks is not None:
        for i, cls_idx in enumerate(result.boxes .cls.int().tolist()):
            mask = result.masks.data[i]
            binary = mask.cpu().numpy().astype(np.uint8)
            if binary.shape != (h, w):
                binary = cv2.resize(binary, dsize=(w, h), interpolation=cv2.INTER_NEAREST)

            masks_by_class.setdefault(CLASSES[cls_idx], []).append(binary.astype(bool))

    overlay = img.copy()
    if draw:
        alpha = 0.5
        for cls_name, masks in masks_by_class.items():
            color = np.array(COLORS[cls_name])
            for mask in masks:
                overlay[mask] = (alpha * color + (1 - alpha) * img[mask]).astype(np.uint8)

    return masks_by_class, overlay


img_path = 'images/boxes/mix.jpg'
_, output_img = segment(img_path, draw=True)
cv2.imshow('', output_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
