import io
import json

from PIL import Image

from recorder import RunRecorder


def make_jpeg():
    buf = io.BytesIO()
    Image.new('RGB', (320, 240)).save(buf, format='JPEG')
    return buf.getvalue()


def test_save_image_writes_original_and_marked_image(tmp_path):
    recorder = RunRecorder(tmp_path)
    recorder.save_image('001', make_jpeg(), target=(0.75, 0.35))
    assert (recorder.dir / '001.jpg').exists()
    assert (recorder.dir / '001_target.jpg').exists()


def test_save_image_without_target_writes_only_original(tmp_path):
    recorder = RunRecorder(tmp_path)
    recorder.save_image('002', make_jpeg(), target=None)
    assert (recorder.dir / '002.jpg').exists()
    assert not (recorder.dir / '002_target.jpg').exists()


def test_append_event_writes_one_json_per_line(tmp_path):
    recorder = RunRecorder(tmp_path)
    recorder.append_event({'state': 'look'})
    recorder.append_event({'state': 'turn'})
    lines = (recorder.dir / 'events.jsonl').read_text().splitlines()
    assert [json.loads(line)['state'] for line in lines] == ['look', 'turn']
