"""
Robot decision from the camera: STOP / SLOW_DOWN / CONTINUE.

Two layers, so the robot is always safe and still "smart":

  1. decide_from_people(people, frame.shape)   FAST, local, every frame (no internet)
     Rules on the YOLO tracks from people.detect_people(): is a person in the path
     (|bearing| <= 15 deg), how close (box height / frame height), approaching?

  2. ask_groq(frame, people, rule_decision)     ONE Groq vision call per captured image
     The vision model sees the image + the YOLO summary and returns JSON:
     a short scene summary, the obstacles, and the decision.

  merge(): the MORE CAREFUL of the two wins. Groq can make the robot stop or slow down,
  never overrule a STOP from the rules. If Groq fails (no key, 429 rate limit, timeout,
  bad JSON) the rule decision is used and the reason is printed.

Decision (a plain dict, JSON-ready):
  {"action": "STOP" | "SLOW_DOWN" | "CONTINUE",
   "speed_factor": 0.0 .. 1.0,          multiply the robot's normal speed by this
   "steer": "none" | "left" | "right",  side to pass a person/obstacle on
   "reason": "...", "source": "rules" | "rules+llm",
   "scene": "...", "obstacles": [...],  (from Groq, when it answered)
   "people_in_path": int, "closest_person": {...} | None}

Live demo (webcam):   python robot_decision.py
  SPACE or ENTER = capture this image and ask Groq     q = quit
  The window always shows the fast rule decision; each capture prints a short report
  and saves the image to captures/.
Env (.env works): GROQ_API_KEY, GROQ_VISION_MODEL, CAMERA_INDEX (default 0)
"""
import base64
import json
import os
import re
import threading
import time

import requests

try:
    from dotenv import load_dotenv
    load_dotenv()
except ImportError:
    pass

# ---------------- SETTINGS ----------------

PATH_HALF_ANGLE_DEG = 15.0   # |bearing| below this = the person is in the robot's path
VERY_CLOSE_RATIO = 0.60      # person box height / frame height: ~1 m or closer
NEAR_RATIO = 0.35            # ~2 m
GROQ_URL = "https://api.groq.com/openai/v1/chat/completions"
GROQ_TIMEOUT_S = 30

ACTIONS = ("STOP", "SLOW_DOWN", "CONTINUE")
CAUTION = {"STOP": 2, "SLOW_DOWN": 1, "CONTINUE": 0}   # higher = more careful


def _decision(action, speed, reason, source="rules", steer="none", in_path=0, closest=None):
    return {
        "action": action,
        "speed_factor": round(max(0.0, min(1.0, speed)), 2),
        "steer": steer,
        "reason": reason,
        "source": source,
        "scene": "",
        "obstacles": [],
        "people_in_path": in_path,
        "closest_person": closest,
    }


# ---------------- LAYER 1: rules from YOLO ----------------

def _person_info(p, frame_h):
    x1, y1, x2, y2 = p["box"]
    ratio = max(0.0, (y2 - y1) / frame_h)
    return {
        "id": p.get("id"),
        "bearing_deg": p.get("bearing_deg", 0.0),
        "size_ratio": round(ratio, 2),
        "distance": "very close" if ratio >= VERY_CLOSE_RATIO else "near" if ratio >= NEAR_RATIO else "far",
        "in_path": abs(p.get("bearing_deg", 0.0)) <= PATH_HALF_ANGLE_DEG,
        "approaching": bool(p.get("approaching", False)),
    }


def decide_from_people(people, frame_shape):
    """Fast safety decision from detect_people() output. frame_shape = frame.shape."""
    if frame_shape is None or len(frame_shape) < 2 or frame_shape[0] <= 0:
        return _decision("STOP", 0.0, "no valid camera frame: stopping to be safe")
    if not people:
        return _decision("CONTINUE", 1.0, "no people visible")

    info = [_person_info(p, frame_shape[0]) for p in people]
    closest = max(info, key=lambda i: i["size_ratio"])
    in_path = [i for i in info if i["in_path"]]
    n = len(in_path)
    steer = "right" if closest["bearing_deg"] < 0 else "left"   # pass on the side away from them

    for i in sorted(in_path, key=lambda i: -i["size_ratio"]):
        if i["distance"] == "very close":
            return _decision("STOP", 0.0, f"person {i['id']} very close in front", in_path=n, closest=closest)
        if i["distance"] == "near" and i["approaching"]:
            return _decision("STOP", 0.0, f"person {i['id']} near and coming towards the robot",
                             in_path=n, closest=closest)
        if i["distance"] == "near":
            return _decision("SLOW_DOWN", 0.4, f"person {i['id']} near in the path", steer=steer,
                             in_path=n, closest=closest)
        if i["approaching"]:
            return _decision("SLOW_DOWN", 0.6, f"person {i['id']} ahead and approaching", steer=steer,
                             in_path=n, closest=closest)

    if in_path:
        return _decision("CONTINUE", 0.8, "people far ahead: continue carefully", in_path=n, closest=closest)
    if closest["distance"] == "very close":
        return _decision("SLOW_DOWN", 0.5, f"person {closest['id']} very close at the side", steer=steer,
                         in_path=0, closest=closest)
    return _decision("CONTINUE", 1.0, "people visible but not in the path", in_path=0, closest=closest)


def people_summary(people, frame_shape):
    """YOLO summary for the Groq prompt and the printed report."""
    if not people or frame_shape is None:
        return []
    return [_person_info(p, frame_shape[0]) for p in people]


# ---------------- LAYER 2: one Groq vision call ----------------

PROMPT = """You are the eyes of an indoor service robot (college building, drives forward slowly, max 0.2 m/s).
Look at this camera image and decide what the robot should do RIGHT NOW.

Person detector (YOLO), bearing <0 = left, >0 = right, in_path = in front of the robot:
{people}
Safety layer already decided: {rule_action} ({rule_reason}).

STOP if anything blocks the path ahead (person, chair, box, wall, door, stairs, wet floor) or someone may step in.
SLOW_DOWN if people or obstacles are near the path. CONTINUE only if the path ahead is clear.
Never be less careful than the safety layer.

Reply with ONLY this JSON, nothing else:
{{"scene": "one short sentence",
  "obstacles": ["short names of things in or near the path"],
  "action": "STOP" | "SLOW_DOWN" | "CONTINUE",
  "speed_factor": 0.0-1.0,
  "steer": "none" | "left" | "right",
  "reason": "one short sentence"}}"""


def parse_llm_reply(text):
    """Turn the model's reply into a decision dict; None if it isn't usable."""
    m = re.search(r"\{.*\}", text or "", re.S)
    if not m:
        return None
    try:
        data = json.loads(m.group(0))
    except ValueError:
        return None
    action = str(data.get("action", "")).upper().replace(" ", "_")
    if action not in ACTIONS:
        return None
    try:
        speed = float(data.get("speed_factor", 0.5))
    except (TypeError, ValueError):
        speed = 0.5
    if action == "STOP":
        speed = 0.0
    steer = str(data.get("steer", "none")).lower()
    if steer not in ("none", "left", "right"):
        steer = "none"
    d = _decision(action, speed, str(data.get("reason", ""))[:200], source="llm", steer=steer)
    d["scene"] = str(data.get("scene", ""))[:300]
    obstacles = data.get("obstacles", [])
    d["obstacles"] = [str(o)[:40] for o in obstacles][:8] if isinstance(obstacles, list) else []
    return d


def ask_groq(frame, people, rule_decision):
    """One Groq vision call. Returns (decision or None, error text or None)."""
    import cv2
    api_key = os.getenv("GROQ_API_KEY")
    model = os.getenv("GROQ_VISION_MODEL")
    if not api_key:
        return None, "GROQ_API_KEY is missing (.env)"
    if not model:
        return None, "GROQ_VISION_MODEL is missing (.env)"

    ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not ok:
        return None, "could not encode the image"
    image_b64 = base64.b64encode(jpg.tobytes()).decode()
    summary = people_summary(people, frame.shape)
    prompt = PROMPT.format(people=json.dumps(summary) if summary else "no people detected",
                           rule_action=rule_decision["action"], rule_reason=rule_decision["reason"])
    try:
        r = requests.post(
            GROQ_URL,
            headers={"Authorization": f"Bearer {api_key}", "Content-Type": "application/json"},
            json={
                "model": model,
                "messages": [{"role": "user", "content": [
                    {"type": "text", "text": prompt},
                    {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64," + image_b64}},
                ]}],
                "temperature": 0.1,
                "max_tokens": 300,
            },
            timeout=GROQ_TIMEOUT_S,
        )
    except requests.RequestException as e:
        return None, f"no answer from Groq ({e.__class__.__name__}): check the internet"
    if r.status_code == 429:
        wait = r.headers.get("retry-after", "a few")
        return None, f"Groq rate limit (429): wait {wait} s before the next capture"
    if r.status_code != 200:
        return None, f"Groq error {r.status_code}: {r.text[:200]}"
    try:
        text = r.json()["choices"][0]["message"]["content"]
    except (KeyError, IndexError, ValueError):
        return None, "could not read Groq's response"
    d = parse_llm_reply(text)
    if d is None:
        return None, f"Groq did not return the decision JSON: {text[:150]!r}"
    return d, None


# ---------------- MERGE ----------------

def merge(rule_decision, llm_decision):
    """The more careful action wins; speed is the lower of the two."""
    if llm_decision is None:
        return rule_decision
    if CAUTION[llm_decision["action"]] > CAUTION[rule_decision["action"]]:
        out = dict(llm_decision)
    else:
        out = dict(rule_decision)
        if llm_decision["action"] == rule_decision["action"] and llm_decision["reason"]:
            out["reason"] = f"{rule_decision['reason']}; Groq: {llm_decision['reason']}"
    out["speed_factor"] = min(rule_decision["speed_factor"], llm_decision["speed_factor"])
    if out["action"] == "STOP":
        out["speed_factor"] = 0.0
    out["source"] = "rules+llm"
    out["scene"] = llm_decision.get("scene", "")
    out["obstacles"] = llm_decision.get("obstacles", [])
    out["people_in_path"] = rule_decision["people_in_path"]
    out["closest_person"] = rule_decision["closest_person"]
    return out


def decide(frame, people, use_groq=True):
    """One full decision for one image. Returns (decision, groq_error or None)."""
    shape = None if frame is None or getattr(frame, "size", 0) == 0 else frame.shape
    rule = decide_from_people(people, shape)
    if not use_groq or shape is None:
        return rule, None
    llm, err = ask_groq(frame, people, rule)
    return merge(rule, llm), err


# ---------------- REPORT ----------------

def report(n, people, frame_shape, decision, error):
    """Short printout for one capture."""
    lines = [f"\n=== CAPTURE {n}  ({time.strftime('%H:%M:%S')}) ==="]
    summ = people_summary(people, frame_shape)
    if summ:
        lines.append("People (YOLO): " + "; ".join(
            f"id{p['id']} {abs(p['bearing_deg']):.0f} deg {'left' if p['bearing_deg'] < 0 else 'right'}, "
            f"{p['distance']}{', IN PATH' if p['in_path'] else ''}{', approaching' if p['approaching'] else ''}"
            for p in summ))
    else:
        lines.append("People (YOLO): none")
    if decision.get("scene"):
        lines.append("Scene (Groq):  " + decision["scene"])
    if decision.get("obstacles"):
        lines.append("Obstacles:     " + ", ".join(decision["obstacles"]))
    if error:
        lines.append("Groq:          not used, " + error)
    steer = f", pass on the {decision['steer']}" if decision["steer"] != "none" else ""
    lines.append(f">>> ROBOT DECISION: {decision['action']}  (speed x{decision['speed_factor']:.1f}{steer})"
                 f"  - {decision['reason']}  [{decision['source']}]")
    return "\n".join(lines)


# ---------------- LIVE DEMO (webcam) ----------------

COLOURS = {"STOP": (0, 0, 255), "SLOW_DOWN": (0, 200, 255), "CONTINUE": (0, 200, 0)}
KEYS_CAPTURE = (32, 13, 10)        # SPACE, ENTER (Windows), ENTER (Linux)


def _draw(frame, people, decision, status):
    import cv2
    for p in people:
        x1, y1, x2, y2 = p["box"]
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 200, 0), 2)
        tag = f"id{p['id']} {p['bearing_deg']:+.0f}deg" + (" APPROACH" if p.get("approaching") else "")
        cv2.putText(frame, tag, (x1, max(15, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 1)
    c = COLOURS[decision["action"]]
    cv2.rectangle(frame, (0, 0), (frame.shape[1], 58), (0, 0, 0), -1)
    cv2.putText(frame, f"{decision['action']}  speed x{decision['speed_factor']:.1f}  steer {decision['steer']}"
                       f"  [{decision['source']}]", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 2)
    cv2.putText(frame, decision["reason"][:90], (8, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    cv2.putText(frame, status, (8, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)


def main():
    import cv2
    from people import detect_people   # loads YOLO

    cam = cv2.VideoCapture(int(os.getenv("CAMERA_INDEX", "0")))
    if not cam.isOpened():
        raise SystemExit("Cannot open the camera (try: set CAMERA_INDEX=1)")
    os.makedirs("captures", exist_ok=True)

    state = {"busy": False, "n": 0, "last": None, "last_t": 0.0}

    def capture(frame, people):
        try:
            decision, err = decide(frame, people)
            state["n"] += 1
            cv2.imwrite(os.path.join("captures", f"capture_{state['n']:03d}.jpg"), frame)
            print(report(state["n"], people, frame.shape, decision, err))
            print(json.dumps(decision))
            state.update(last=decision, last_t=time.time())
        finally:
            state["busy"] = False

    print("SPACE or ENTER = capture + decide,  q = quit   (click the video window first)")
    while True:
        ok, frame = cam.read()
        if not ok:
            print(report(state["n"] + 1, [], None, decide_from_people([], None), "no camera frame"))
            break
        people = detect_people(frame)
        live = decide_from_people(people, frame.shape)
        # show the last Groq-checked decision for 5 s after a capture, then the live rule decision
        shown = state["last"] if state["last"] and time.time() - state["last_t"] < 5 else live

        key = cv2.waitKey(1) & 0xFF
        if key in KEYS_CAPTURE:
            if state["busy"]:
                print("(still waiting for Groq on the last capture)")
            else:
                state["busy"] = True
                threading.Thread(target=capture, args=(frame.copy(), people), daemon=True).start()
        elif key == ord("q"):
            break

        status = "asking Groq..." if state["busy"] else "SPACE/ENTER = capture   q = quit"
        _draw(frame, people, shown, status)
        cv2.imshow("Pathik vision decision", frame)
    cam.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
