import cv2
import joblib
import numpy as np
from sklearn.ensemble import RandomForestClassifier
from skimage.measure import regionprops
from skimage.segmentation import quickshift, find_boundaries

MODEL_PATH = 'weights/random_forest.joblib'
CLASSES = ['wheat', 'rapeseed', 'pea']
COLORS = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]


def segment(img):
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    segments = quickshift(img, kernel_size=3, max_dist=6, ratio=0.5)

    feats, labels = [], []
    for r in regionprops(segments):
        ys, xs = r.coords.T
        pixels = lab[ys, xs].astype(np.float32)
        feats.append([
            r.area, r.eccentricity, r.solidity,
            r.major_axis_length, r.minor_axis_length,
            *pixels.mean(axis=0), *pixels.std(axis=0),
        ])
        labels.append(r.label)
    return np.array(feats), segments, np.array(labels)


def apply_overlay(img, segments, labels, pred, alpha=0.5, draw_edges=True):
    lut = np.full(segments.max() + 1, -1, dtype=int)
    lut[labels] = pred
    class_map = lut[segments]

    color_layer = img.copy()
    for i, color in enumerate(COLORS):
        color_layer[class_map == i] = color

    painted = (class_map >= 0)[..., None]
    blended = np.where(
        painted, (img * (1 - alpha) + color_layer * alpha).astype(np.uint8), img
    )
    if draw_edges:
        blended[find_boundaries(segments) & painted[..., 0]] = (255, 255, 255)
    return blended, class_map


def add_legend(img, count, area):
    output_img = img.copy()
    cv2.rectangle(output_img, (0, 0), (350, 100), color=(0, 0, 0), thickness=-1)
    for i, (grain_type, color) in enumerate(zip(CLASSES, COLORS)):
        y = 30 + i * 32
        cv2.rectangle(output_img, (10, y - 18), (34, y + 6), color, thickness=-1)
        text = f'{grain_type}: {count[i]} bins, {area[i] * 100:.1f}%'
        params = dict(
            fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.8, color=(255, 255, 255), thickness=1, lineType=cv2.LINE_AA
        )
        cv2.putText(output_img, text, org=(44, y), **params)
    return output_img


def train(img_path):
    X, y = [], []
    for label, grain_type in enumerate(CLASSES):
        for i in range(3):
            feats, _, _ = segment(cv2.imread(img_path + grain_type + '/' + grain_type + '_00' + str(i + 1) + '.jpg'))
            X.append(feats)
            y.append(np.full(len(feats), label))

    X, y = np.vstack(X), np.concatenate(y)

    clf = RandomForestClassifier(
        n_estimators=300, class_weight='balanced', random_state=42, n_jobs=-1
    ).fit(X, y)
    joblib.dump(
        {'model': clf, 'classes': CLASSES}, MODEL_PATH, compress=3
    )

img_path = 'images/boxes/'

# train(img_path)

clf = joblib.load(MODEL_PATH)['model']
input_img = cv2.imread(img_path + 'sorted.jpg')
feats, segments, labels = segment(input_img)
pred = clf.predict(feats)

overlay, class_map = apply_overlay(input_img, segments, labels, pred)
count = np.bincount(pred, minlength=3)
area = np.bincount(pred, weights=feats[:, 0], minlength=3) / feats[:, 0].sum()
overlay = add_legend(overlay, count, area)
cv2.imwrite('results/sorted.png', overlay)
cv2.imshow('', overlay)
cv2.waitKey(0)
cv2.destroyAllWindows()
