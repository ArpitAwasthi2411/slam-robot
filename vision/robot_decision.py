"""
Robot decision from what the camera sees.

Two layers, so the robot is always safe and still "smart":

  1. decide_from_people(people, frame_shape)    FAST, local, every frame (no internet)
     Uses YOLO tracks from people.detect_people(): where each person is (bearing),
     how close (box height vs frame height), whether they are approaching.

  2. decide_with_llm(description, people, rule_decision)   SLOW, Groq, every few seconds
     Gives Groq the vision description (people.describe(frame, "groq")) plus the
     YOLO summary and asks for a decision as JSON.

  decide(...) merges them: the final action is the MORE CAREFUL of the two.
  The LLM can make the robot more careful, never less (safety rule).
  If Groq fails / times out / returns junk, the rule decision is used.

Output (a plain dict, JSON-ready):
  {"action": "STOP" | "SLOW_DOWN" | "CONTINUE",
   "speed_factor": 0.0 .. 1.0,          multiply the robot's normal speed by this
   "steer": "none" | "left" | "right",  suggested side to pass an obstacle/person
   "reason": "...", "source": "rules" | "llm" | "rules+llm",
   "people_in_path": int, "closest_person": {...} | None}

Run the live demo (webcam):  python robot_decision.py
  keys: q = quit, c = ask Groq now
Env vars (.env works too): GROQ_API_KEY, GROQ_VISION_MODEL (for describe),
  GROQ_DECISION_MODEL (default llama-3.1-8b-instant), CAMERA_INDEX (default 0),
  DESCRIBE_EVERY_S (default 5)
"""
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
DECISION_MODEL = os.getenv("GROQ_DECISION_MODEL", "llama-3.1-8b-instant")
GROQ_URL = "https://api.groq.com/openai/v1/chat/completions"
LLM_TIMEOUT_S = 10

ACTIONS = ("STOP", "SLOW_DOWN", "CONTINUE")
CAUTION = {"STOP": 2, "SLOW_DOWN": 1, "CONTINUE": 0}   # higher = more careful


def _decision(action, speed, reason, source="rules", steer="none", in_path=0, closest=None):
    return {
        "action": action,
        "speed_factor": round(max(0.0, min(1.0, speed)), 2),
        "steer": steer,
        "reason": reason,
        "source": source,
        "people_in_path": in_path,
        "closest_person": closest,
    }


# ---------------- LAYER 1: rules from YOLO ----------------

def decide_from_people(people, frame_shape):
    """Fast safety decision from detect_people() output. frame_shape = frame.shape."""
    if frame_shape is None or len(frame_shape) < 2 or frame_shape[0] <= 0:
        return _decision("STOP", 0.0, "no valid camera frame: stopping to be safe")

    frame_h = frame_shape[0]
    if not people:
        return _decision("CONTINUE", 1.0, "no people visible")

    info = []
    for p in people:
        x1, y1, x2, y2 = p["box"]
        ratio = max(0.0, (y2 - y1) / frame_h)
        info.append({
            "id": p.get("id"),
            "bearing_deg": p.get("bearing_deg", 0.0),
            "size_ratio": round(ratio, 2),
            "distance": "very close" if ratio >= VERY_CLOSE_RATIO else "near" if ratio >= NEAR_RATIO else "far",
            "in_path": abs(p.get("bearing_deg", 0.0)) <= PATH_HALF_ANGLE_DEG,
            "approaching": bool(p.get("approaching", False)),
        })

    closest = max(info, key=lambda i: i["size_ratio"])
    in_path = [i for i in info if i["in_path"]]
    n = len(in_path)

    # pass on the side away from the closest person
    steer = "right" if closest["bearing_deg"] < 0 else "left"

    for i in sorted(in_path, key=lambda i: -i["size_ratio"]):
        if i["distance"] == "very close":
            return _decision("STOP", 0.0, f"person {i['id']} very close in front", steer="none",
                             in_path=n, closest=closest)
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


# ---------------- LAYER 2: Groq decision ----------------

DECISION_PROMPT = """You decide what an indoor service robot should do RIGHT NOW, from its camera.
The robot drives forward slowly (max 0.2 m/s) in a college building.

Camera description (from a vision model):
{description}

Person detector (YOLO) summary, bearing <0 = left, >0 = right, in_path = in front of the robot:
{people}

Safety layer already decided: {rule_action} ({rule_reason}).

Rules: STOP if anything (person, object, stairs, door, wet floor) blocks the path or someone may step in.
SLOW_DOWN if people or obstacles are near the path. CONTINUE only if the path ahead is clear.
Never be less careful than the safety layer.
Reply with ONLY this JSON:
{{"action": "STOP" | "SLOW_DOWN" | "CONTINUE", "speed_factor": 0.0-1.0, "steer": "none" | "left" | "right", "reason": "one short sentence"}}"""


def _is_error_text(description):
    d = (description or "").strip()
    return (not d or d.startswith(("Vision API error", "Could not", "No valid image", "Unknown provider"))
            or "environment variable is missing" in d)


def decide_with_llm(description, people_summary, rule_decision):
    """Ask Groq for a decision. Returns a decision dict, or None if anything goes wrong."""
    api_key = os.getenv("GROQ_API_KEY")
    if not api_key or _is_error_text(description):
        return None

    prompt = DECISION_PROMPT.format(
        description=description.strip()[:1500],
        people=json.dumps(people_summary) if people_summary else "no people detected",
        rule_action=rule_decision["action"],
        rule_reason=rule_decision["reason"],
    )
    try:
        r = requests.post(
            GROQ_URL,
            headers={"Authorization": f"Bearer {api_key}", "Content-Type": "application/json"},
            json={
                "model": DECISION_MODEL,
                "messages": [{"role": "user", "content": prompt}],
                "temperature": 0.0,
                "max_tokens": 150,
                "response_format": {"type": "json_object"},
            },
            timeout=LLM_TIMEOUT_S,
        )
        r.raise_for_status()
        text = r.json()["choices"][0]["message"]["content"]
    except (requests.RequestException, KeyError, IndexError, ValueError):
        return None
    return parse_llm_reply(text)


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
        speed = float(data.get("speed_factor", 0.0 if action == "STOP" else 0.5))
    except (TypeError, ValueError):
        speed = 0.5
    if action == "STOP":
        speed = 0.0
    steer = str(data.get("steer", "none")).lower()
    if steer not in ("none", "left", "right"):
        steer = "none"
    return _decision(action, speed, str(data.get("reason", ""))[:200], source="llm", steer=steer)


# ---------------- MERGE ----------------

def merge(rule_decision, llm_decision):
    """The more careful action wins; speed is the lower of the two."""
    if llm_decision is None:
        return rule_decision
    if CAUTION[llm_decision["action"]] > CAUTION[rule_decision["action"]]:
        out = dict(llm_decision)
    else:
        out = dict(rule_decision)
        if llm_decision["action"] == rule_decision["action"]:
            out["reason"] = f"{rule_decision['reason']}; LLM: {llm_decision['reason']}"
    out["speed_factor"] = min(rule_decision["speed_factor"], llm_decision["speed_factor"])
    if out["action"] == "STOP":
        out["speed_factor"] = 0.0
    out["source"] = "rules+llm"
    out["people_in_path"] = rule_decision["people_in_path"]
    out["closest_person"] = rule_decision["closest_person"]
    return out


def people_summary(people, frame_shape):
    """Short YOLO summary for the LLM prompt."""
    if not people:
        return []
    h = frame_shape[0]
    return [{
        "id": p.get("id"),
        "bearing_deg": p.get("bearing_deg"),
        "in_path": abs(p.get("bearing_deg", 0.0)) <= PATH_HALF_ANGLE_DEG,
        "size_ratio": round((p["box"][3] - p["box"][1]) / h, 2),
        "approaching": p.get("approaching", False),
        "moving_away": p.get("moving_away", False),
    } for p in people]


def decide(frame, people, description=None):
    """One full decision. description = people.describe(frame, 'groq') text, or None for rules only."""
    shape = None if frame is None or getattr(frame, "size", 0) == 0 else frame.shape
    rule = decide_from_people(people, shape)
    if description is None or shape is None:
        return rule
    llm = decide_with_llm(description, people_summary(people, shape), rule)
    return merge(rule, llm)


# ---------------- LIVE DEMO (webcam) ----------------

COLOURS = {"STOP": (0, 0, 255), "SLOW_DOWN": (0, 200, 255), "CONTINUE": (0, 200, 0)}


def _draw(frame, people, decision, description):
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
    if description:
        cv2.putText(frame, "Groq: " + description.replace("\n", " ")[:100], (8, frame.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)


def main():
    import cv2
    from people import detect_people, describe   # loads YOLO

    cam = cv2.VideoCapture(int(os.getenv("CAMERA_INDEX", "0")))
    if not cam.isOpened():
        raise SystemExit("Cannot open the camera (try CAMERA_INDEX=1)")
    every = float(os.getenv("DESCRIBE_EVERY_S", "5"))

    # Groq runs in a background thread so the video and the safety rules never freeze
    state = {"llm": None, "description": None, "busy": False, "last": 0.0}

    def ask_groq(frame, people):
        try:
            desc = describe(frame, "groq")
            llm = decide_with_llm(desc, people_summary(people, frame.shape),
                                  decide_from_people(people, frame.shape))
            state.update(description=desc, llm=llm, last=time.time())
            print("\n--- Groq ---\n" + desc)
            print("LLM decision:", json.dumps(llm) if llm else "none (error/timeout): using rules only")
        finally:
            state["busy"] = False

    print("q = quit, c = ask Groq now")
    last_action = None
    while True:
        ok, frame = cam.read()
        if not ok:
            print(json.dumps(decide_from_people([], None)))
            break
        people = detect_people(frame)
        rule = decide_from_people(people, frame.shape)
        # an LLM decision older than 2 periods is stale: ignore it
        fresh_llm = state["llm"] if time.time() - state["last"] < 2 * every + 5 else None
        decision = merge(rule, fresh_llm)

        key = cv2.waitKey(1) & 0xFF
        if not state["busy"] and (key == ord("c") or time.time() - state["last"] > every):
            state["busy"] = True
            state["last"] = time.time()
            threading.Thread(target=ask_groq, args=(frame.copy(), people), daemon=True).start()

        if decision["action"] != last_action:
            print(json.dumps(decision))
            last_action = decision["action"]

        _draw(frame, people, decision, state["description"])
        cv2.imshow("Pathik vision decision", frame)
        if key == ord("q"):
            break
    cam.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
