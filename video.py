# video.py
import cv2
import os
import numpy as np
from pathlib import Path
import time

# ==========================================================
# CONFIG
# ==========================================================
FOLDER    = r"C:\Users\dinht\Downloads\261011-kit"       # ← FIX
RECURSIVE = False
DRY_RUN   = False

MAX_W = 1400
MAX_H = 800
BAR_H = 64

VIDEO_EXTS = {".mp4", ".mov", ".avi", ".mkv", ".webm", ".m4v"}

VIDEO_SPEED_DEFAULT = 2.0
MAX_VIDEO_FPS_SKIP  = 120.0

# ==========================================================
# HELPERS
# ==========================================================
def resize_to_fit(img, max_w, max_h):
    h, w = img.shape[:2]
    s = min(max_w / w, max_h / h, 1.0)
    if s < 1.0:
        img = cv2.resize(img, (int(w * s), int(h * s)), interpolation=cv2.INTER_AREA)
    return img

def shorten(name, limit=80):
    if len(name) <= limit: return name
    return name[:40] + "..." + name[-35:]

def build_header(width, idx, total, name, info, color, progress=None):
    header = np.full((BAR_H, width, 3), (35, 35, 35), dtype=np.uint8)
    cv2.line(header, (0, BAR_H - 1), (width, BAR_H - 1), (90, 90, 90), 1)
    cv2.putText(header, f"[{idx}/{total}]  {shorten(name, 90)}",
                (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(header, info,
                (12, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                color, 1, cv2.LINE_AA)
    if progress is not None:
        bw = width - 24
        cv2.rectangle(header, (12, BAR_H - 8), (12 + bw, BAR_H - 4), (70, 70, 70), -1)
        cv2.rectangle(header, (12, BAR_H - 8),
                      (12 + int(bw * progress), BAR_H - 4), (0, 210, 120), -1)
    return header

# ==========================================================
# REVIEW 1 VIDEO
# ==========================================================
def review_video(path, idx, total, window, folder, start_speed=VIDEO_SPEED_DEFAULT):
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        return 'unreadable'

    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps <= 0 or fps > MAX_VIDEO_FPS_SKIP:
        fps = 30.0
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    vw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    vh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    duration = total_frames / fps if total_frames > 0 else 0.0

    rel = path.relative_to(folder)
    parent = rel.parent.as_posix() if rel.parent.as_posix() != "." else "."

    speed  = start_speed
    paused = False
    frame_idx = 0
    frame = None
    action = None

    while True:
        if not paused:
            ret, frame = cap.read()
            if not ret:
                break
            frame_idx += 1

        if frame is None:
            break

        disp = resize_to_fit(frame, MAX_W, MAX_H)

        cur_t = frame_idx / fps
        time_str = (f"{cur_t:.1f}s / {duration:.1f}s"
                    if duration > 0 else f"{cur_t:.1f}s")
        sp_str = f"{speed:.0f}x" if speed == int(speed) else f"{speed:.1f}x"
        paused_tag = "  [PAUSED]" if paused else ""

        info = (f"VIDEO  |  {vw}x{vh}  |  {fps:.0f}fps  |  "
                f"{time_str}  |  Speed {sp_str}  |  {parent}{paused_tag}")

        progress = (frame_idx / total_frames) if total_frames > 0 else None
        header = build_header(disp.shape[1], idx, total, path.name,
                              info, (120, 255, 120), progress)

        cv2.imshow(window, np.vstack([header, disp]))

        delay = max(1, int(1000.0 / (fps * speed))) if not paused else 40
        k = cv2.waitKey(delay) & 0xFF

        if   k in (ord('k'), ord('K')):                     action = 'keep';   break
        elif k in (ord('d'), ord('D'), ord('x'), ord('X')): action = 'delete'; break
        elif k in (ord('q'), ord('Q'), 27):                 action = 'quit';   break
        elif k == ord(' '):                                 paused = not paused
        elif k == ord('1'):                                 speed = 1.0
        elif k == ord('2'):                                 speed = 2.0
        elif k == ord('3'):                                 speed = 4.0
        elif k == ord('4'):                                 speed = 8.0
        elif k == ord('r'):
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            frame_idx = 0
            paused = False

    # ---- Hết video tự nhiên ----
    if action is None and frame is not None:
        disp = resize_to_fit(frame, MAX_W, MAX_H)
        info = f"VIDEO END  |  {vw}x{vh}  |  {duration:.1f}s  |  {parent}"
        header = build_header(disp.shape[1], idx, total, path.name,
                              info, (255, 180, 100))
        canvas = np.vstack([header, disp])

        ov = canvas.copy()
        cv2.rectangle(ov, (0, BAR_H), (canvas.shape[1], canvas.shape[0]), (0, 0, 0), -1)
        canvas = cv2.addWeighted(ov, 0.45, canvas, 0.55, 0)
        cv2.putText(canvas, "END - K=Giu / D=Xoa / Q=Thoat",
                    (24, BAR_H + 44), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                    (255, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow(window, canvas)

        while True:
            k = cv2.waitKey(0) & 0xFF
            if   k in (ord('k'), ord('K')):                     action = 'keep';   break
            elif k in (ord('d'), ord('D'), ord('x'), ord('X')): action = 'delete'; break
            elif k in (ord('q'), ord('Q'), 27):                 action = 'quit';   break

    cap.release()
    return action or 'skip'

# ==========================================================
# MAIN
# ==========================================================
def main():
    folder = Path(FOLDER)
    if not folder.exists():
        raise FileNotFoundError(f"Folder khong ton tai: {folder}")

    files = ([p for p in folder.rglob("*")
              if p.suffix.lower() in VIDEO_EXTS and p.is_file()]
             if RECURSIVE else
             [p for p in folder.iterdir()
              if p.suffix.lower() in VIDEO_EXTS and p.is_file()])
    files.sort(key=lambda p: p.name.lower())
    total = len(files)

    print(f"Folder : {folder}")
    print(f"Videos : {total}")
    if total == 0:
        raise SystemExit("Khong co video nao.")

    if not DRY_RUN:
        if input("\nChe do XOA THAT. Go YES de tiep tuc: ").strip().upper() != "YES":
            print("Da huy."); return

    print("\nPhim tat:")
    print("   K        = Giu + next")
    print("   D / X    = Xoa + next")
    print("   Space    = Pause / Resume")
    print("   R        = Replay")
    print("   1/2/3/4  = 1x / 2x / 4x / 8x")
    print("   Q / ESC  = Thoat\n")

    WINDOW = "Review Video | K=Next D=Xoa Space=Pause Q=Thoat"
    cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)

    kept, deleted, skipped, unreadable = [], [], [], []
    t0 = time.time()

    try:
        for idx, path in enumerate(files, 1):
            act = review_video(path, idx, total, WINDOW, folder)

            if   act == 'keep':  kept.append(path);    print(f"[{idx}/{total}] KEEP : {path.name}")
            elif act == 'skip':  skipped.append(path); print(f"[{idx}/{total}] SKIP : {path.name}")
            elif act == 'delete':
                if DRY_RUN:
                    print(f"[{idx}/{total}] DRY  : {path.name}")
                else:
                    try:
                        os.remove(path)
                        print(f"[{idx}/{total}] DEL  : {path.name}")
                    except Exception as e:
                        print(f"[{idx}/{total}] ERR  : {path.name} - {e}")
                deleted.append(path)
            elif act == 'unreadable':
                unreadable.append(path); print(f"[{idx}/{total}] BAD  : {path.name}")
            elif act == 'quit':
                print("\nThoat som."); break
    except KeyboardInterrupt:
        print("\nCtrl+C.")
    finally:
        cv2.destroyAllWindows()

    elapsed = time.time() - t0
    print("\n" + "=" * 62)
    print("TONG KET")
    print("=" * 62)
    print(f"Tong video       : {total}")
    print(f"Giu              : {len(kept)}")
    print(f"Da xoa           : {len(deleted)}{'  (DRY RUN)' if DRY_RUN else ''}")
    print(f"Bo qua           : {len(skipped)}")
    print(f"Khong doc        : {len(unreadable)}")
    print(f"Thoi gian        : {elapsed:.1f}s")

    if deleted and not DRY_RUN:
        log = folder / "_deleted_videos_log.txt"
        mode = "a" if log.exists() else "w"
        with open(log, mode, encoding="utf-8") as f:
            if mode == "w":
                f.write("# Danh sach video da xoa\n")
            for p in deleted:
                f.write(f"{p}\n")
        print(f"\nLog: {log}")

if __name__ == "__main__":
    main()