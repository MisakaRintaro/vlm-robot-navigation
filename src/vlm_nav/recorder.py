"""1回の実行の記録（AI に送った画像・指した点・イベント）をフォルダに保存する。"""
import io
import json
import time
from pathlib import Path

from PIL import Image, ImageDraw


class RunRecorder:
    """実行ごとに `<root>/<日時>/` を作り、そこへ記録を書き出す。"""

    def __init__(self, root: Path):
        self.dir = Path(root) / time.strftime('%Y%m%d_%H%M%S')
        self.dir.mkdir(parents=True, exist_ok=True)

    def save_image(self, name: str, jpeg: bytes, target: tuple[float, float] | None) -> None:
        """AI に送った画像と、AI が指した点を描いた画像を保存する。

        Args:
            name: ファイル名（拡張子なし。例："003"、"005_07"）。
            jpeg: AI に送った JPEG 画像。
            target: AI が指した位置 (x, y)（0〜1）。見えなかった場合は None。

        Side Effects:
            `<name>.jpg` を書き出す。target があれば `<name>_target.jpg` も書き出す。
        """
        (self.dir / f'{name}.jpg').write_bytes(jpeg)
        if target is None:
            return
        image = Image.open(io.BytesIO(jpeg)).convert('RGB')
        x, y = target[0] * image.width, target[1] * image.height
        r = 8
        ImageDraw.Draw(image).ellipse((x - r, y - r, x + r, y + r), outline=(255, 0, 0), width=3)
        image.save(self.dir / f'{name}_target.jpg')

    def append_event(self, event: dict) -> None:
        """イベントを1行の JSON として `events.jsonl` に追記する。

        Side Effects:
            `events.jsonl` に1行追記する。
        """
        with open(self.dir / 'events.jsonl', 'a', encoding='utf-8') as f:
            f.write(json.dumps(event, ensure_ascii=False) + '\n')
