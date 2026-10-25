import torch
from PIL import Image
from transformers import AutoModelForImageTextToText, AutoProcessor

model_id = 'Qwen/Qwen2.5-VL-7B-Instruct'

model = AutoModelForImageTextToText.from_pretrained(
    model_id, dtype=torch.bfloat16, device_map='auto', attn_implementation='sdpa'
)
processor = AutoProcessor.from_pretrained(
    model_id, min_pixels=256 * 28 * 28, max_pixels=1536 * 28 * 28
)

img_dir = 'images/'
references = [
    ('wheat', img_dir + 'wheat.jpg'),
    ('rapeseed', img_dir + 'rapeseed.jpg'),
    ('peas', img_dir + 'rapeseed.jpg'),
]


def load_img(path, max_side=None):
    img = Image.open(path).convert('RGB')
    if max_side:
        img.thumbnail((max_side, max_side))
    return img


def main(img_path, references, prompt, max_new_tokens=300):
    content, images = [], []

    content.append({'type': 'text', 'text': 'Reference photos of the pure grains:'})
    for name, path in references:
        img = load_img(path, max_side=448)
        content.append({'type': 'text', 'text': f'Reference - {name}:'})
        content.append({'type': 'image', 'image': img})
        images.append(img)

    input_img = load_img(img_path)
    content.append({'type': 'text', 'text': 'Now the mixture to analyze:'})
    content.append({'type': 'image', 'image': input_img})
    images.append(input_img)

    content.append({'type': 'text', 'text': prompt})

    messages = [{'role': 'user', 'content': content}]
    text = processor.apply_chat_template(
        messages, tokenize=False, add_generation_prompt=True
    )
    inputs = processor(text=[text], images=images, return_tensors='pt').to(model.device)

    with torch.inference_mode():
        out = model.generate(**inputs, max_new_tokens=max_new_tokens, do_sample=False)

    gen = out[:, inputs.input_ids.shape[1]:]
    return processor.batch_decode(gen, skip_special_tokens=True)[0]


prompt = (
    'The last image is a mixture of wheat, rapeseed and peas. Using the reference photos '
    'to identify each grain, estimate the share of each in the mixture by visible area. '
    'Answer with JSON only, percentages summing to 100: '
    '{"wheat": <int>, "rapeseed": <int>, "peas": <int>}'
)
print(main(img_path=img_dir + 'sorted.jpg', references=references, prompt=prompt))
print(main(img_path=img_dir + 'wheat.jpg', references=references, prompt=prompt))
print(main(img_path=img_dir + 'rapeseed.jpg', references=references, prompt=prompt))
print(main(img_path=img_dir + 'rapeseed.jpg', references=references, prompt=prompt))
