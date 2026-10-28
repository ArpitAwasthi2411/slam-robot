# Vision → robot decision (Rashi)

`robot_decision.py` turns what the camera sees into STOP / SLOW_DOWN / CONTINUE (+ speed factor, steer side).
It sits next to `people.py` (YOLO `detect_people()` + Groq `describe()`), which lives in the vision project.

- Layer 1, every frame: rules on the YOLO tracks (in path = |bearing| <= 15 deg, closeness = box height / frame height).
- Layer 2, on SPACE/ENTER: one Groq vision call returns scene + obstacles + decision as JSON.
- The more careful decision wins; Groq errors fall back to the rules.

```
python -m pytest -q test_robot_decision.py     # 11 tests, no camera/internet
python robot_decision.py                       # webcam demo (SPACE/ENTER capture + decide, q quit)
```
Env: GROQ_API_KEY, GROQ_VISION_MODEL, CAMERA_INDEX. Captures are saved to captures/.
