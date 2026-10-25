from pathlib import Path
from roboflow import Roboflow
from ultralytics import YOLO

# rf = Roboflow(api_key='gJQbiPWBOb1GDvnOLenu')
# project = rf.workspace('dnnproject').project('grain-types')
# version = project.version(1)
# dataset = version.download('folder')


def main():
    data_dir = Path('grain-types-1')

    if (data_dir / 'valid').exists() and not (data_dir / 'val').exists():
        (data_dir / 'valid').rename(data_dir / 'val')
    print('dataset:', data_dir, [p.name for p in data_dir.iterdir()])

    model = YOLO('yolo26n-cls.pt')
    model.train(
        data=str(data_dir),
        imgsz=128,                     # same as the patch size (try 224 too)
        epochs=50,
        patience=10,                     # stop early if val accuracy stops improving
        batch=32,
        workers=2,                       # keep low on Windows; 0 if you get errors
        project='runs', name='grains', exist_ok=True,
        hsv_h=0.01, hsv_s=0.4, hsv_v=0.5,
        fliplr=0.5, flipud=0.5, degrees=180,
        erasing=0.0,                     # random erasing can hide the tiny rapeseed
    )

    metrics = model.val(split='test')    # held-out photo
    print('test top-1 accuracy:', metrics.top1)


if __name__ == '__main__':      # this guard is the actual fix
    main()
