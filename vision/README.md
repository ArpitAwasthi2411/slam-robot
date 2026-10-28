# Vision → robot decision (Rashi)

`robot_decision.py` turns what the camera sees into STOP / SLOW_DOWN / CONTINUE (+ speed factor, steer side).
It sits next to `people.py` (YOLO `detect_people()` + Groq `describe()`), which lives in the vision project.

- Layer 1, every frame: rules on the YOLO tracks (in path = |bearing| <= 15 deg, closeness = box height / frame height).
- Layer 2, every 5 s: Groq text model reads the vision description + YOLO summary, replies JSON.
- The more careful decision wins; Groq errors fall back to the rules.

```
python -m pytest -q test_robot_decision.py     # 11 tests, no camera/internet
python robot_decision.py                       # webcam demo (q quit, c ask Groq now)
```
Env: GROQ_API_KEY, GROQ_VISION_MODEL, GROQ_DECISION_MODEL (default llama-3.1-8b-instant), CAMERA_INDEX, DESCRIBE_EVERY_S.
