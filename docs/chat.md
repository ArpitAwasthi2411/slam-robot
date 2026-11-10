# Talking to Pathik: conversation, college knowledge and planning from talk

Code: `lidar_robot/chat.py` (the agent), `lidar_robot/knowledge.py` (the notebook), `commands.py` (`build`: the
checks every trip passes). App: **Send** (conversation) and **Operate → Knowledge** (the notebook).
Tests: `test/test_chat.py`. Measure: `tools/eval_chat.py`.

## What it does

- **Answers questions about the college and the department** from a notebook the team writes in the app
  ("Library timings", "Head of the department", "Project lab rules" …).
- **Answers about itself** from its live state: what it is doing, battery, where it is, why it stopped.
- **General talk**: greetings, "what is SLAM", study questions, in English, Hindi or Hinglish. It answers in
  the language the person used.
- **Remembers the talk** (last 6 turns per person, 15 minutes), so follow-ups work:
  "who is the HOD?" → "ok take me there".
- **Plans trips from talk**: "I want to print something" → the room that has the printer; several stops;
  "in the shortest order" is computed from the map, not guessed by the model.
- **Works without internet**: the best notebook entry is read out; otherwise it says it does not know.
- Voice button, app voice and typed text all use the same path. The robot speaks the answer.

## How one message is handled

```
text ─► offline rules ─► clear command? (go to Lab 3, stop, why did you stop, where is the printer)
           │ yes ──────────────────────────────────────────────► command path (docs/llm.md), no chat
           │ no / it is a question ("library timings?", "who is the head of the department?")
           ▼
        greeting, thanks, time, "what can you do"? ──► answered at once, offline
           ▼
        look up the notebook (top 4 entries) + places + robot state + the talk so far
           ▼
        ONE language-model call (strict JSON) ──► may ask for one narrower lookup ──► second call (max 2)
           ▼
        guards ──► reply (shown + spoken)        or        trip ──► the same checks as every command
```

A question that names a place ("library timings?") used to be read as "go to the library". Now a sentence that
starts with a question word or ends in "?" without a go/take/bring verb is a question.

## Guards against made-up answers

| Risk | Guard |
|---|---|
| Invents a teacher's name, a timing | College facts only from the notebook or the map. The answer must list the entries it used; ids are limited to the ones it was shown (JSON schema enum). "From the notebook" with no entry → replaced by "I don't know that yet". |
| Invents a number (phone, room, time) | In a notebook / robot answer, every number of 2+ digits must appear in what the model was given. Otherwise the answer is replaced by "I am not sure…". |
| Sends the robot to a place that does not exist | Targets are an enum of the map's names, then checked again by `CommandParser.build`. A trip with no valid place stays a spoken reply. |
| Marks a chat trip urgent | Priority 3 only when the person used an urgent word. |
| Drives because someone asked a question | Questions are routed to chat; from the robot's voice button the home-spot rules still apply to any trip. |
| "Stop" delayed by the model | "Stop" never reaches the conversation path. |
| Long or formatted answers on the speaker | Plain text, at most ~3 sentences, cut at a sentence end. |
| Model or internet fails | Notebook read-out; the reason is kept (`llm.error`) and shown in Operate → AI commands. |

General-knowledge answers are **not** checked against anything; the app labels them "general knowledge, not
checked". The notebook is the only source that is verified.

## Writing the notebook (Operate → Knowledge, admin)

- One fact per entry, 1–3 sentences, with the words people actually say in the tags ("hod", "timing", "fees").
- Write what is true for months. For things that change weekly, write where to ask instead.
- The robot is used by everyone who walks up to it: no phone numbers, no private details about people.
- **Try** box: type a question and see which entries the robot would read. If none match, add tags.
- Stored in `~/maps/knowledge.json` (not in Git). Three entries about the robot itself are built in.

Suggested first entries: department name and courses, HOD and office location, office / library / lab timings,
which lab is used for what, where to submit forms, who to ask about the robot, event of the week.

## Latency

- Clear commands and greetings: no model call.
- A question: one call (`chat_model`, default `openai/gpt-oss-120b`, `reasoning_effort: low`, ≤ 500 tokens).
- The app previews and then sends: the second request reuses the first answer (90 s).
- The call runs in a worker thread: the robot keeps driving and avoiding obstacles while it thinks.

## Settings (`config/robot_params.yaml`, navigator)

```yaml
chat_enabled: true
chat_model: openai/gpt-oss-120b
```

## Measuring it (for the report)

```bash
python3 tools/eval_chat.py                                   # offline path
GROQ_API_KEY=gsk_... python3 tools/eval_chat.py --csv results/chat.csv
```

14 cases on a fixed notebook: grounded answers, questions it must refuse to guess, general talk, follow-up
trips, and sentences that must not move the robot. It prints pass/fail, latency and how many answers the guards
blocked.

**Not yet tested against the real Groq service** (the build machine cannot reach it). The agent logic is tested
with a scripted model (`test_chat.py`); run the eval above with the key on the Pi or a laptop before the demo.
